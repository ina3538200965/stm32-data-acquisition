/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   帧协议实现（组帧 + 逐字节状态机解析）
  *
  *          全部函数是纯计算：同样的输入在任何平台上得到同样的输出，
  *          因此可以被 test/ 目录里的用例覆盖 —— 这是"能量化验证"的前提。
  ******************************************************************************
  */
#include "protocol.h"

/* 状态机内部状态（不对外暴露，避免调用方写出依赖具体状态的耦合代码） */
#define ST_SOF0     0u
#define ST_SOF1     1u
#define ST_LEN      2u
#define ST_CMD      3u
#define ST_DATA     4u
#define ST_XOR      5u

static void ResetParser(PROTO_Parser_t *parser)
{
    parser->state  = ST_SOF0;
    parser->len    = 0u;
    parser->cmd    = 0u;
    parser->index  = 0u;
    parser->xorBuf = 0u;
}

uint8_t PROTO_CalcXor(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t x = 0u;

    if (buf == 0)
    {
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        x ^= buf[i];
    }
    return x;
}

uint16_t PROTO_Pack(uint8_t cmd, const uint8_t *payload, uint8_t len,
                    uint8_t *out, uint16_t outSize)
{
    uint16_t frameLen;
    uint16_t i;

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
        return 0u;                      /* 缓冲区不够，不写任何字节 */
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

    out[i] = PROTO_CalcXor(out, (uint16_t)(4u + len));   /* 校验覆盖 SOF..PAYLOAD */
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
                parser->state  = ST_SOF1;
                parser->xorBuf = byte;
            }
            break;

        case ST_SOF1:
            if (byte == (uint8_t)PROTO_SOF1)
            {
                parser->xorBuf ^= byte;
                parser->state = ST_LEN;
            }
            else if (byte == (uint8_t)PROTO_SOF0)
            {
                /* 允许 0xAA 0xAA 0x55 这种情形：仍停在这个头位置 */
                parser->xorBuf = byte;
            }
            else
            {
                ResetParser(parser);        /* 不是完整帧头，丢弃重来 */
            }
            break;

        case ST_LEN:
            parser->xorBuf ^= byte;
            if (byte > PROTO_MAX_PAYLOAD)
            {
                ResetParser(parser);
                ret = PROTO_ERR_LEN;        /* 长度非法：直接放弃这一帧 */
            }
            else
            {
                parser->len   = byte;
                parser->state = ST_CMD;
            }
            break;

        case ST_CMD:
            parser->xorBuf ^= byte;
            parser->cmd   = byte;
            parser->index = 0u;
            parser->state = (parser->len == 0u) ? ST_XOR : ST_DATA;
            break;

        case ST_DATA:
            parser->xorBuf ^= byte;
            parser->payload[parser->index] = byte;
            parser->index++;
            if (parser->index >= parser->len)
            {
                parser->state = ST_XOR;
            }
            break;

        case ST_XOR:
            if (parser->xorBuf != byte)
            {
                ResetParser(parser);
                ret = PROTO_ERR_XOR;
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
                ResetParser(parser);        /* 收完立即复位，可以连续解析下一帧 */
                ret = PROTO_OK;
            }
            break;

        default:
            ResetParser(parser);
            break;
    }

    return ret;
}
