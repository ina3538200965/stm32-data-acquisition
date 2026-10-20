/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   帧协议实现：CRC16 + 组帧 + 逐字节状态机解析
  *
  *  全部函数是纯计算：同样的输入在任何平台上得到同样的输出，
  *  因此可以被 test/ 目录里的用例覆盖 —— 这是"能量化验证"的前提。
  ******************************************************************************
  */
#include "protocol.h"

/* 状态机内部状态（不对外暴露，避免调用方写出依赖具体状态的耦合代码） */
#define ST_SOF0       0u
#define ST_SOF1       1u
#define ST_LEN        2u
#define ST_CMD        3u
#define ST_DATA       4u
#define ST_CRC_LO     5u
#define ST_CRC_HI     6u
#define ST_EOF        7u

#define CRC16_INIT    0xFFFFu
#define CRC16_POLY    0xA001u   /* 0x8005 的位反转形式，配合右移实现 */

/**
 * @brief  把一个新的字节并入已有的 CRC
 * @note   CRC 是可以"流式"计算的：不用把整帧攒齐再算，
 *         收一个字节算一次，收完就是最终结果。
 *         这个性质让状态机可以边收边算，不占额外缓冲。
 */
static uint16_t CRC16_Update(uint16_t crc, uint8_t byte)
{
    uint8_t j;

    crc ^= (uint16_t)byte;

    for (j = 0u; j < 8u; j++)
    {
        if ((crc & 0x0001u) != 0u)
        {
            crc = (uint16_t)((crc >> 1) ^ CRC16_POLY);
        }
        else
        {
            crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void ResetParser(PROTO_Parser_t *parser)
{
    parser->state   = ST_SOF0;
    parser->len     = 0u;
    parser->cmd     = 0u;
    parser->index   = 0u;
    parser->crcCalc = CRC16_INIT;
    parser->crcLo   = 0u;
}

uint16_t PROTO_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = CRC16_INIT;
    uint16_t i;

    if (buf == 0)
    {
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        crc = CRC16_Update(crc, buf[i]);
    }
    return crc;
}

uint16_t PROTO_Pack(uint8_t cmd, const uint8_t *payload, uint8_t len,
                    uint8_t *out, uint16_t outSize)
{
    uint16_t frameLen;
    uint16_t i;
    uint16_t crc;

    /* 参数防线：宁可返回失败，也不写坏内存 */
    if (out == 0)
    {
        return 0u;
    }
    if (len > PROTO_MAX_PAYLOAD)
    {
        return 0u;
    }
    if ((len > 0u) && (payload == 0))
    {
        return 0u;
    }

    frameLen = (uint16_t)(PROTO_OVERHEAD + len);
    if (outSize < frameLen)
    {
        return 0u;                      /* 缓冲区不够，一个字节都不写 */
    }

    i = 0u;
    out[i++] = (uint8_t)PROTO_SOF0;
    out[i++] = (uint8_t)PROTO_SOF1;
    out[i++] = len;
    out[i++] = cmd;

    while (i < (uint16_t)(4u + len))
    {
        out[i] = payload[i - 4u];
        i++;
    }

    /* CRC 覆盖范围：从 SOF0 到最后一个 PAYLOAD 字节（含帧头，不含 CRC 自身和帧尾） */
    crc = PROTO_CRC16(out, (uint16_t)(4u + len));

    /* 小端：低字节在前。和 Modbus 一致 */
    out[i++] = (uint8_t)(crc & 0xFFu);
    out[i++] = (uint8_t)((crc >> 8) & 0xFFu);

    out[i] = (uint8_t)PROTO_EOF;

    return frameLen;
}

void PROTO_ParserInit(PROTO_Parser_t *parser)
{
    if (parser != 0)
    {
        ResetParser(parser);
    }
}

PROTO_Status_t PROTO_Feed(PROTO_Parser_t *parser, uint8_t byte, PROTO_Frame_t *frame)
{
    PROTO_Status_t ret = PROTO_BUSY;

    if (parser == 0)
    {
        return PROTO_ERR_LEN;
    }

    switch (parser->state)
    {
        case ST_SOF0:
            /* 空闲态：一直找第一个同步字节 */
            if (byte == (uint8_t)PROTO_SOF0)
            {
                ResetParser(parser);
                parser->crcCalc = CRC16_Update(CRC16_INIT, byte);
                parser->state   = ST_SOF1;
            }
            break;

        case ST_SOF1:
            if (byte == (uint8_t)PROTO_SOF1)
            {
                parser->crcCalc = CRC16_Update(parser->crcCalc, byte);
                parser->state   = ST_LEN;
            }
            else if (byte == (uint8_t)PROTO_SOF0)
            {
                /* 允许 0xAA 0xAA 0x55：仍停在这个头位置，重新起算 */
                parser->crcCalc = CRC16_Update(CRC16_INIT, byte);
            }
            else
            {
                ResetParser(parser);        /* 不是完整帧头，丢弃重来 */
            }
            break;

        case ST_LEN:
            parser->crcCalc = CRC16_Update(parser->crcCalc, byte);
            if (byte > PROTO_MAX_PAYLOAD)
            {
                ResetParser(parser);
                ret = PROTO_ERR_LEN;        /* 长度非法，直接放弃这一帧 */
            }
            else
            {
                parser->len   = byte;
                parser->state = ST_CMD;
            }
            break;

        case ST_CMD:
            parser->crcCalc = CRC16_Update(parser->crcCalc, byte);
            parser->cmd   = byte;
            parser->index = 0u;
            parser->state = (parser->len == 0u) ? ST_CRC_LO : ST_DATA;
            break;

        case ST_DATA:
            parser->crcCalc = CRC16_Update(parser->crcCalc, byte);
            parser->payload[parser->index] = byte;
            parser->index++;
            if (parser->index >= parser->len)
            {
                parser->state = ST_CRC_LO;
            }
            break;

        case ST_CRC_LO:
            /* 帧里带来的 CRC，不参与计算 */
            parser->crcLo = byte;
            parser->state = ST_CRC_HI;
            break;

        case ST_CRC_HI:
        {
            uint16_t crcRecv = (uint16_t)(((uint16_t)byte << 8) | parser->crcLo);

            if (crcRecv != parser->crcCalc)
            {
                ResetParser(parser);
                ret = PROTO_ERR_CRC;
            }
            else
            {
                parser->state = ST_EOF;
            }
            break;
        }

        case ST_EOF:
            if (byte != (uint8_t)PROTO_EOF)
            {
                /* CRC 对了但帧尾不对：说明同步位置是巧合对上的，仍然丢弃 */
                ResetParser(parser);
                ret = PROTO_ERR_EOF;
            }
            else
            {
                if (frame != 0)
                {
                    uint16_t k;
                    frame->cmd = parser->cmd;
                    frame->len = parser->len;
                    for (k = 0u; k < parser->len; k++)
                    {
                        frame->payload[k] = parser->payload[k];
                    }
                }
                ResetParser(parser);        /* 收完立即复位，可连续解析下一帧 */
                ret = PROTO_OK;
            }
            break;

        default:
            ResetParser(parser);
            break;
    }

    return ret;
}
