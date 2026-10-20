/**
  ******************************************************************************
  * @file    test_protocol.c
  * @brief   protocol 层单元测试（D6：CRC16 + 帧尾版）
  *
  *  覆盖目标：
  *   1) CRC16 与公开测试向量一致     2) 组帧各字段位置正确
  *   3) 参数非法时的防御              4) 逐字节状态机
  *   5) CRC 错误 / 帧尾错误 / 长度错误 6) 丢帧后重新同步
  *   7) 全长度往返 + 连续多帧 + 粘包
  *
  *  断言不带 printf：在 MCU 上运行时只需要一个计数器，
  *  结果可以从调试器的 Watch 窗口观察（见 APP_GetSelfTestFailures）。
  ******************************************************************************
  */
#include "test_protocol.h"
#include "protocol.h"
#include <string.h>

static TEST_Result_t s_result;

/* ---------------- 断言基础设施 ---------------- */

static void RecordFail(uint16_t line)
{
    if (s_result.failed < TEST_MAX_RECORDED_FAILS)
    {
        s_result.failLine[s_result.failed] = line;
    }
    s_result.failed++;
}

static void Expect(int condition, uint16_t line)
{
    s_result.total++;
    if (condition == 0)
    {
        RecordFail(line);
    }
}

/* 失败时把源码行号记下来，不必抓串口打印也能知道哪一行挂了 */
#define EXPECT(cond)  Expect((cond), (uint16_t)__LINE__)

/* ---------------- 用例 1：CRC16 必须和公开向量一致 ---------------- */

static void Test_CrcVector(void)
{
    /* CRC-16/Modbus 的标准测试向量："123456789" -> 0x4B37
       这条来自 Modbus 官方规范，自己写的 CRC 不跟它对齐就等于没验证 */
    const uint8_t s[9] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };

    EXPECT(PROTO_CRC16(s, 9u) == 0x4B37u);

    /* 空数据的结果是初值本身 */
    EXPECT(PROTO_CRC16(s, 0u) == 0xFFFFu);

    /* 空指针必须安全返回，不能崩 */
    EXPECT(PROTO_CRC16(0, 10u) == 0u);
}

/* ---------------- 用例 2：基本组帧，逐字段核对 ---------------- */

static void Test_PackBasic(void)
{
    uint8_t payload[3];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    uint16_t n;
    uint16_t crc;

    payload[0] = 0x11u;
    payload[1] = 0x22u;
    payload[2] = 0x33u;

    n = PROTO_Pack(0x08u, payload, 3u, buf, (uint16_t)sizeof(buf));
    EXPECT(n == (PROTO_OVERHEAD + 3u));         /* 7 + 3 = 10 */

    EXPECT(buf[0] == (uint8_t)PROTO_SOF0);
    EXPECT(buf[1] == (uint8_t)PROTO_SOF1);
    EXPECT(buf[2] == 3u);                        /* LEN 只数 payload */
    EXPECT(buf[3] == 0x08u);                     /* CMD */
    EXPECT(memcmp(&buf[4], payload, 3u) == 0);   /* PAYLOAD */

    /* CRC 覆盖 SOF0..PAYLOAD，共 4 + len = 7 字节 */
    crc = PROTO_CRC16(buf, 7u);
    EXPECT(buf[7] == (uint8_t)(crc & 0xFFu));          /* CRC 低字节在前 */
    EXPECT(buf[8] == (uint8_t)((crc >> 8) & 0xFFu));   /* CRC 高字节 */
    EXPECT(buf[9] == (uint8_t)PROTO_EOF);              /* 帧尾 */
}

/* ---------------- 用例 3：空载荷边界 ---------------- */

static void Test_PackEmptyPayload(void)
{
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    uint16_t n;

    n = PROTO_Pack(0x01u, 0, 0u, buf, (uint16_t)sizeof(buf));
    EXPECT(n == PROTO_OVERHEAD);
    EXPECT(buf[2] == 0u);
    EXPECT(buf[3] == 0x01u);
    EXPECT(buf[n - 1u] == (uint8_t)PROTO_EOF);

    /* payload 指针为空但 len > 0 -> 必须拒绝 */
    EXPECT(PROTO_Pack(0x01u, 0, 1u, buf, (uint16_t)sizeof(buf)) == 0u);
}

/* ---------------- 用例 4：非法参数防御 ---------------- */

static void Test_PackInvalidParam(void)
{
    uint8_t payload[2];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    uint8_t small[2];

    payload[0] = 0xAAu;
    payload[1] = 0x55u;

    EXPECT(PROTO_Pack(0x01u, payload, 2u, 0, 10u) == 0u);            /* 输出缓冲为空 */
    EXPECT(PROTO_Pack(0x01u, payload, PROTO_MAX_PAYLOAD + 1u, buf,
                      (uint16_t)sizeof(buf)) == 0u);                 /* 超载长度 */
    EXPECT(PROTO_Pack(0x01u, payload, 2u, small,
                      (uint16_t)sizeof(small)) == 0u);               /* 缓冲区不够 */
}

/* ---------------- 用例 5：整帧喂入状态机 ---------------- */

static void Test_FeedWholeFrame(void)
{
    uint8_t payload[4];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;
    uint16_t n;
    uint16_t i;
    PROTO_Status_t st;

    payload[0] = 0x01u;
    payload[1] = 0x02u;
    payload[2] = 0x03u;
    payload[3] = 0x04u;

    n = PROTO_Pack(0x7Fu, payload, 4u, buf, (uint16_t)sizeof(buf));
    PROTO_ParserInit(&parser);

    /* 前 n-1 个字节都应该返回 BUSY */
    for (i = 0u; i < (n - 1u); i++)
    {
        st = PROTO_Feed(&parser, buf[i], &frame);
        EXPECT(st == PROTO_BUSY);
    }

    /* 最后一个字节（帧尾）校验通过 */
    st = PROTO_Feed(&parser, buf[n - 1u], &frame);
    EXPECT(st == PROTO_OK);
    EXPECT(frame.cmd == 0x7Fu);
    EXPECT(frame.len == 4u);
    EXPECT(memcmp(frame.payload, payload, 4u) == 0);
}

/* ---------------- 用例 6：CRC 被打坏必须被抓到 ---------------- */

static void Test_FeedBadCrc(void)
{
    uint8_t payload[2];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;
    uint16_t n;
    uint16_t i;

    payload[0] = 0x10u;
    payload[1] = 0x20u;

    /* CRC 是在收完高字节（第 n-2 个）时判定的 —— 不等帧尾。
       这样设计的好处：坏帧能早一个字节被发现，接收方可以立刻丢弃，
       不用把整帧收完再判。下面的 n-2 不是笔误。 */

    /* (a) 打坏 CRC 低字节 */
    n = PROTO_Pack(0x03u, payload, 2u, buf, (uint16_t)sizeof(buf));
    buf[n - 3u] ^= 0xFFu;

    PROTO_ParserInit(&parser);
    for (i = 0u; i < (n - 2u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    EXPECT(PROTO_Feed(&parser, buf[n - 2u], &frame) == PROTO_ERR_CRC);

    /* (b) 打坏 CRC 高字节 */
    n = PROTO_Pack(0x03u, payload, 2u, buf, (uint16_t)sizeof(buf));
    buf[n - 2u] ^= 0x01u;

    PROTO_ParserInit(&parser);
    for (i = 0u; i < (n - 2u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    EXPECT(PROTO_Feed(&parser, buf[n - 2u], &frame) == PROTO_ERR_CRC);

    /* (c) 打坏一个 payload 字节，CRC 也该抓到 */
    n = PROTO_Pack(0x03u, payload, 2u, buf, (uint16_t)sizeof(buf));
    buf[4] ^= 0x08u;

    PROTO_ParserInit(&parser);
    for (i = 0u; i < (n - 2u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    EXPECT(PROTO_Feed(&parser, buf[n - 2u], &frame) == PROTO_ERR_CRC);

    /* 出错后解析器必须已复位，下一个正确帧能被正常接收 */
    n = PROTO_Pack(0x04u, payload, 2u, buf, (uint16_t)sizeof(buf));
    for (i = 0u; i < (n - 1u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    EXPECT(PROTO_Feed(&parser, buf[n - 1u], &frame) == PROTO_OK);
}

/* ---------------- 用例 7：CRC 对但帧尾错 ---------------- */

static void Test_FeedBadEof(void)
{
    uint8_t payload[2];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;
    uint16_t n;
    uint16_t i;

    payload[0] = 0x30u;
    payload[1] = 0x40u;

    n = PROTO_Pack(0x05u, payload, 2u, buf, (uint16_t)sizeof(buf));
    buf[n - 1u] = 0x00u;                     /* 只改帧尾，CRC 仍然是对的 */

    PROTO_ParserInit(&parser);
    for (i = 0u; i < (n - 1u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    EXPECT(PROTO_Feed(&parser, buf[n - 1u], &frame) == PROTO_ERR_EOF);
}

/* ---------------- 用例 8：非法长度 ---------------- */

static void Test_FeedBadLen(void)
{
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;

    PROTO_ParserInit(&parser);
    EXPECT(PROTO_Feed(&parser, (uint8_t)PROTO_SOF0, &frame) == PROTO_BUSY);
    EXPECT(PROTO_Feed(&parser, (uint8_t)PROTO_SOF1, &frame) == PROTO_BUSY);
    EXPECT(PROTO_Feed(&parser, (uint8_t)(PROTO_MAX_PAYLOAD + 1u), &frame) == PROTO_ERR_LEN);
}

/* ---------------- 用例 9：垃圾数据后的重新同步 ---------------- */

static void Test_ResyncAfterGarbage(void)
{
    uint8_t payload[2];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;
    uint16_t n;
    uint16_t i;

    payload[0] = 0xAAu;                    /* 故意让载荷里含 SOF 字节 */
    payload[1] = 0x55u;

    n = PROTO_Pack(0x05u, payload, 2u, buf, (uint16_t)sizeof(buf));
    PROTO_ParserInit(&parser);

    /* 先灌一段垃圾，状态机不能被带偏 */
    for (i = 0u; i < 7u; i++)
    {
        (void)PROTO_Feed(&parser, (uint8_t)(0x30u + i), &frame);
    }

    for (i = 0u; i < (n - 1u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    EXPECT(PROTO_Feed(&parser, buf[n - 1u], &frame) == PROTO_OK);
    EXPECT(frame.cmd == 0x05u);
    EXPECT(frame.len == 2u);
    EXPECT(memcmp(frame.payload, payload, 2u) == 0);
}

/* ---------------- 用例 10：粘包（两帧连着喂，中间无间隔） ---------------- */

static void Test_BackToBackFrames(void)
{
    uint8_t payload[3];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    uint8_t stream[PROTO_FRAME_MAX_LEN * 2u];
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;
    uint16_t n1;
    uint16_t n2;
    uint16_t i;
    uint16_t okCount = 0u;

    payload[0] = 0xA1u;
    payload[1] = 0xB2u;
    payload[2] = 0xC3u;

    n1 = PROTO_Pack(0x01u, payload, 3u, buf, (uint16_t)sizeof(buf));
    memcpy(&stream[0], buf, n1);

    n2 = PROTO_Pack(0x02u, payload, 3u, buf, (uint16_t)sizeof(buf));
    memcpy(&stream[n1], buf, n2);

    /* 连续喂两帧共 n1+n2 个字节，中间不插入任何等待 */
    PROTO_ParserInit(&parser);
    for (i = 0u; i < (n1 + n2); i++)
    {
        if (PROTO_Feed(&parser, stream[i], &frame) == PROTO_OK)
        {
            okCount++;
            /* 第一帧 cmd=0x01，第二帧 cmd=0x02 */
            EXPECT(frame.cmd == ((okCount == 1u) ? 0x01u : 0x02u));
            EXPECT(frame.len == 3u);
            EXPECT(memcmp(frame.payload, payload, 3u) == 0);
        }
    }
    EXPECT(okCount == 2u);          /* 必须解出完整两帧，不能吞帧 */
}

/* ---------------- 用例 11：全长度往返 ---------------- */

static void Test_RoundTripAllLengths(void)
{
    uint8_t len;
    uint8_t k;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;
    uint16_t n;
    uint16_t i;

    for (k = 0u; k < PROTO_MAX_PAYLOAD; k++)
    {
        payload[k] = (uint8_t)(k * 7u + 1u);
    }

    for (len = 0u; len <= PROTO_MAX_PAYLOAD; len++)
    {
        n = PROTO_Pack(0x42u, payload, len, buf, (uint16_t)sizeof(buf));
        EXPECT(n == (PROTO_OVERHEAD + len));

        PROTO_ParserInit(&parser);
        for (i = 0u; i < (n - 1u); i++)
        {
            EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
        }
        EXPECT(PROTO_Feed(&parser, buf[n - 1u], &frame) == PROTO_OK);
        EXPECT(frame.len == len);
        EXPECT(memcmp(frame.payload, payload, len) == 0);
    }
}

/* ---------------- 入口 ---------------- */

uint16_t TEST_Protocol_Run(void)
{
    uint16_t i;

    s_result.total  = 0u;
    s_result.failed = 0u;
    for (i = 0u; i < TEST_MAX_RECORDED_FAILS; i++)
    {
        s_result.failLine[i] = 0u;
    }

    Test_CrcVector();
    Test_PackBasic();
    Test_PackEmptyPayload();
    Test_PackInvalidParam();
    Test_FeedWholeFrame();
    Test_FeedBadCrc();
    Test_FeedBadEof();
    Test_FeedBadLen();
    Test_ResyncAfterGarbage();
    Test_BackToBackFrames();
    Test_RoundTripAllLengths();

    return s_result.failed;
}

void TEST_Protocol_GetResult(TEST_Result_t *out)
{
    uint16_t i;

    if (out == 0)
    {
        return;
    }
    out->total  = s_result.total;
    out->failed = s_result.failed;
    for (i = 0u; i < TEST_MAX_RECORDED_FAILS; i++)
    {
        out->failLine[i] = s_result.failLine[i];
    }
}
