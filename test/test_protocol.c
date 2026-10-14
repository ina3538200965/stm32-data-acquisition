/**
  ******************************************************************************
  * @file    test_protocol.c
  * @brief   protocol 层单元测试
  *
  *          覆盖目标：
  *          1) 组帧正确性     2) 参数非法时的防御
  *          3) 逐字节状态机   4) 校验错误 / 长度错误
  *          5) 丢帧后的重新同步 6) 全长度往返 + 连续多帧
  *
  *          断言不带 printf：在 MCU 上运行时只需要一个计数器，
  *          结果可以从调试器的 Watch 窗口观察（见 APP_GetSelfTestFailures）。
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

/* ---------------- 用例 1：基本组帧 ---------------- */

static void Test_PackBasic(void)
{
    uint8_t payload[3];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    uint16_t n;

    payload[0] = 0x11u;
    payload[1] = 0x22u;
    payload[2] = 0x33u;

    n = PROTO_Pack(0x08u, payload, 3u, buf, (uint16_t)sizeof(buf));
    EXPECT(n == (PROTO_OVERHEAD + 3u));
    EXPECT(buf[0] == (uint8_t)PROTO_SOF0);
    EXPECT(buf[1] == (uint8_t)PROTO_SOF1);
    EXPECT(buf[2] == 3u);                  /* LEN 只数 payload */
    EXPECT(buf[3] == 0x08u);               /* CMD */
    EXPECT(memcmp(&buf[4], payload, 3u) == 0);
    EXPECT(buf[n - 1u] == PROTO_CalcXor(buf, (uint16_t)(n - 1u)));
}

/* ---------------- 用例 2：空载荷边界 ---------------- */

static void Test_PackEmptyPayload(void)
{
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    uint16_t n;

    n = PROTO_Pack(0x01u, 0, 0u, buf, (uint16_t)sizeof(buf));
    EXPECT(n == PROTO_OVERHEAD);
    EXPECT(buf[2] == 0u);
    EXPECT(buf[3] == 0x01u);

    /* payload 指针为空但 len > 0 -> 必须拒绝 */
    EXPECT(PROTO_Pack(0x01u, 0, 1u, buf, (uint16_t)sizeof(buf)) == 0u);
}

/* ---------------- 用例 3：非法参数防御 ---------------- */

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

/* ---------------- 用例 4：整帧喂入状态机 ---------------- */

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

    /* 最后一个字节校验通过 */
    st = PROTO_Feed(&parser, buf[n - 1u], &frame);
    EXPECT(st == PROTO_OK);
    EXPECT(frame.cmd == 0x7Fu);
    EXPECT(frame.len == 4u);
    EXPECT(memcmp(frame.payload, payload, 4u) == 0);
}

/* ---------------- 用例 5：校验错误必须被抓到 ---------------- */

static void Test_FeedBadXor(void)
{
    uint8_t payload[2];
    uint8_t buf[PROTO_FRAME_MAX_LEN];
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;
    uint16_t n;
    uint16_t i;
    PROTO_Status_t st;

    payload[0] = 0x10u;
    payload[1] = 0x20u;

    n = PROTO_Pack(0x03u, payload, 2u, buf, (uint16_t)sizeof(buf));
    buf[n - 1u] ^= 0xFFu;                       /* 人为把校验字节打错 */

    PROTO_ParserInit(&parser);
    for (i = 0u; i < (n - 1u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    st = PROTO_Feed(&parser, buf[n - 1u], &frame);
    EXPECT(st == PROTO_ERR_XOR);

    /* 出错后解析器必须已复位，下一个正确帧能被正常接收 */
    n = PROTO_Pack(0x04u, payload, 2u, buf, (uint16_t)sizeof(buf));
    for (i = 0u; i < (n - 1u); i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == PROTO_BUSY);
    }
    EXPECT(PROTO_Feed(&parser, buf[n - 1u], &frame) == PROTO_OK);
}

/* ---------------- 用例 6：非法长度 ---------------- */

static void Test_FeedBadLen(void)
{
    PROTO_Parser_t parser;
    PROTO_Frame_t frame;

    PROTO_ParserInit(&parser);
    EXPECT(PROTO_Feed(&parser, (uint8_t)PROTO_SOF0, &frame) == PROTO_BUSY);
    EXPECT(PROTO_Feed(&parser, (uint8_t)PROTO_SOF1, &frame) == PROTO_BUSY);
    EXPECT(PROTO_Feed(&parser, (uint8_t)(PROTO_MAX_PAYLOAD + 1u), &frame) == PROTO_ERR_LEN);
}

/* ---------------- 用例 7：垃圾数据后的重新同步 ---------------- */

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

/* ---------------- 用例 8：全长度往返 + 连续两帧 ---------------- */

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

    /* 解析器收完一帧自动复位，紧接着能再收一帧 */
    n = PROTO_Pack(0x43u, payload, 5u, buf, (uint16_t)sizeof(buf));
    for (i = 0u; i < n; i++)
    {
        EXPECT(PROTO_Feed(&parser, buf[i], &frame) == ((i == (n - 1u)) ? PROTO_OK : PROTO_BUSY));
    }
    EXPECT(frame.cmd == 0x43u);
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

    Test_PackBasic();
    Test_PackEmptyPayload();
    Test_PackInvalidParam();
    Test_FeedWholeFrame();
    Test_FeedBadXor();
    Test_FeedBadLen();
    Test_ResyncAfterGarbage();
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
