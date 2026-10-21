/**
  ******************************************************************************
  * @file    test_ringbuf.c
  * @brief   ringbuf 单元测试（D7：串口 DMA 接收用的环形缓冲）
  *
  *  重点覆盖环形缓冲最容易写错的地方：
  *   1) Init 参数非法时的防御
  *   2) 写指针绕回 buffer 开头后数据还顺序正确（回绕）
  *   3) 刚好填满 vs 溢出一格，两者必须分得开
  *   4) PutMulti / GetMulti 批量接口的**部分成功**行为
  *   5) overflow 计数累加正确
  *
  *  用 size = 8 的小缓冲来测：回绕发生得快，8 次写入就能绕一圈，
  *  比用 256 更容易暴露下标算错的问题。
  ******************************************************************************
  */
#include "test_ringbuf.h"
#include "ringbuf.h"

static RB_TEST_Result_t s_result;

/* ---------------- 断言基础设施 ---------------- */

static void RecordFail(uint16_t line)
{
    if (s_result.failed < RB_TEST_MAX_RECORDED_FAILS)
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

#define EXPECT(cond)  Expect((cond), (uint16_t)__LINE__)

/* ---------------- 用例 1：Init 必须拒绝非法参数 ---------------- */

static void Test_InitInvalid(void)
{
    RB_t rb;
    static uint8_t buf[8];

    /* size 不是 2 的幂：0、3、100 都必须被拒 */
    EXPECT(RB_Init(&rb, buf, 0u)   == 1u);
    EXPECT(RB_Init(&rb, buf, 3u)   == 1u);
    EXPECT(RB_Init(&rb, buf, 100u) == 1u);

    /* 指针为空也要拒绝 */
    EXPECT(RB_Init(&rb, 0,   8u)   == 1u);
    EXPECT(RB_Init(0,   buf, 8u)   == 1u);

    /* 非法之后控制块处于"不可用"状态：所有读写都安全返回 0，不能崩 */
    EXPECT(RB_Init(&rb, buf, 3u) == 1u);
    EXPECT(RB_Used(&rb)          == 0u);
    EXPECT(RB_Free(&rb)          == 0u);
    EXPECT(RB_Put(&rb, 0xAAu)    == 0u);
    EXPECT(RB_Get(&rb, (uint8_t *)buf) == 0u);
}

/* ---------------- 用例 2：Init 成功后状态正确 ---------------- */

static void Test_InitOk(void)
{
    RB_t rb;
    static uint8_t buf[256];

    EXPECT(RB_Init(&rb, buf, 256u) == 0u);
    EXPECT(RB_Used(&rb) == 0u);
    EXPECT(RB_Free(&rb) == 256u);
    EXPECT(rb.overflow  == 0u);
}

/* ---------------- 用例 3：空缓冲读必须返回 0 且不改写目标 ---------------- */

static void Test_EmptyGet(void)
{
    RB_t rb;
    static uint8_t buf[8];
    uint8_t out = 0x5Au;
    uint8_t dst[4];

    RB_Init(&rb, buf, 8u);

    EXPECT(RB_Get(&rb, &out) == 0u);
    EXPECT(out == 0x5Au);                       /* 目标不该被动过 */
    EXPECT(RB_GetMulti(&rb, dst, 4u) == 0u);
}

/* ---------------- 用例 4：FIFO 顺序 ---------------- */

static void Test_FifoOrder(void)
{
    RB_t rb;
    static uint8_t buf[8];
    uint8_t v;
    uint16_t i;
    uint16_t ok = 1u;

    RB_Init(&rb, buf, 8u);

    for (i = 0u; i < 5u; i++)
    {
        EXPECT(RB_Put(&rb, (uint8_t)(0xA0u + i)) == 1u);
    }
    EXPECT(RB_Used(&rb) == 5u);

    for (i = 0u; i < 5u; i++)
    {
        EXPECT(RB_Get(&rb, &v) == 1u);
        if (v != (uint8_t)(0xA0u + i))
        {
            ok = 0u;                            /* 顺序错了 */
        }
    }
    EXPECT(ok == 1u);
    EXPECT(RB_Used(&rb) == 0u);
    EXPECT(RB_Free(&rb) == 8u);
}

/* ---------------- 用例 5：回绕 —— 环形缓冲的核心 ---------------- */

static void Test_WrapAround(void)
{
    RB_t rb;
    static uint8_t buf[8];
    uint8_t v;
    uint16_t i;
    uint16_t ok = 1u;

    RB_Init(&rb, buf, 8u);

    /* 反复"填满 -> 全部读出"三轮，写指针会绕过好几圈 */
    for (i = 0u; i < 24u; i++)
    {
        EXPECT(RB_Put(&rb, (uint8_t)i) == 1u);
        EXPECT(RB_Get(&rb, &v) == 1u);
        if (v != (uint8_t)i)
        {
            ok = 0u;
        }
    }
    EXPECT(ok == 1u);
    EXPECT(RB_Used(&rb) == 0u);

    /* 再来一轮：先塞满 8 个（正好绕一圈到原点），再整体读出 */
    ok = 1u;
    for (i = 0u; i < 8u; i++)
    {
        EXPECT(RB_Put(&rb, (uint8_t)(i + 1u)) == 1u);
    }
    EXPECT(RB_Used(&rb) == 8u);
    EXPECT(RB_Free(&rb) == 0u);

    for (i = 0u; i < 8u; i++)
    {
        EXPECT(RB_Get(&rb, &v) == 1u);
        if (v != (uint8_t)(i + 1u))
        {
            ok = 0u;
        }
    }
    EXPECT(ok == 1u);
    EXPECT(RB_Used(&rb) == 0u);
}

/* ---------------- 用例 6：刚好填满 vs 溢出一格 ---------------- */

static void Test_FillAndOverflow(void)
{
    RB_t rb;
    static uint8_t buf[8];
    uint8_t v;
    uint16_t i;

    RB_Init(&rb, buf, 8u);

    /* 第 8 个（最后一个）必须还能写进去：容量就是 8，不多不少 */
    for (i = 0u; i < 8u; i++)
    {
        EXPECT(RB_Put(&rb, (uint8_t)(0x10u + i)) == 1u);
    }
    EXPECT(RB_Used(&rb) == 8u);
    EXPECT(RB_Free(&rb) == 0u);

    /* 第 9 个溢出：返回 0，overflow +1，且**不能覆盖已有数据** */
    EXPECT(RB_Put(&rb, 0xFFu) == 0u);
    EXPECT(rb.overflow == 1u);
    EXPECT(RB_Used(&rb) == 8u);

    EXPECT(RB_Put(&rb, 0xFEu) == 0u);
    EXPECT(rb.overflow == 2u);

    /* 老数据完好无损 */
    EXPECT(RB_Get(&rb, &v) == 1u);
    EXPECT(v == 0x10u);
    EXPECT(RB_Get(&rb, &v) == 1u);
    EXPECT(v == 0x11u);
}

/* ---------------- 用例 7：PutMulti 的部分写入 ---------------- */

static void Test_PutMultiPartial(void)
{
    RB_t rb;
    static uint8_t buf[8];
    static const uint8_t src[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    uint8_t v;
    uint16_t n;
    uint16_t i;

    RB_Init(&rb, buf, 8u);

    /* 先占 5 格，只剩 3 格 */
    for (i = 0u; i < 5u; i++)
    {
        EXPECT(RB_Put(&rb, 0x30u) == 1u);
    }
    EXPECT(RB_Free(&rb) == 3u);

    /* 想写 10 个，只能写进去 3 个，剩下 7 个计入溢出 */
    n = RB_PutMulti(&rb, src, 10u);
    EXPECT(n == 3u);
    EXPECT(rb.overflow == 7u);
    EXPECT(RB_Used(&rb) == 8u);

    /* 写进去的必须是 src 的前 3 个 */
    for (i = 0u; i < 5u; i++)
    {
        EXPECT(RB_Get(&rb, &v) == 1u);          /* 先吐出占位的 5 个 */
        EXPECT(v == 0x30u);
    }
    for (i = 0u; i < 3u; i++)
    {
        EXPECT(RB_Get(&rb, &v) == 1u);
        EXPECT(v == src[i]);
    }
    EXPECT(RB_Used(&rb) == 0u);
}

/* ---------------- 用例 8：GetMulti 的批量读出 ---------------- */

static void Test_GetMulti(void)
{
    RB_t rb;
    static uint8_t buf[16];
    uint8_t dst[16];
    uint16_t n;
    uint16_t i;
    uint16_t ok = 1u;

    RB_Init(&rb, buf, 16u);

    for (i = 0u; i < 10u; i++)
    {
        EXPECT(RB_Put(&rb, (uint8_t)(0x40u + i)) == 1u);
    }

    /* 只要 4 个，就该只给 4 个 */
    n = RB_GetMulti(&rb, dst, 4u);
    EXPECT(n == 4u);
    EXPECT(RB_Used(&rb) == 6u);
    for (i = 0u; i < 4u; i++)
    {
        if (dst[i] != (uint8_t)(0x40u + i))
        {
            ok = 0u;
        }
    }
    EXPECT(ok == 1u);

    /* 再要 100 个，只能给剩下的 6 个 */
    n = RB_GetMulti(&rb, dst, 100u);
    EXPECT(n == 6u);
    EXPECT(RB_Used(&rb) == 0u);
    for (i = 0u; i < 6u; i++)
    {
        if (dst[i] != (uint8_t)(0x44u + i))
        {
            ok = 0u;
        }
    }
    EXPECT(ok == 1u);

    /* 空了以后 GetMulti 返回 0 */
    EXPECT(RB_GetMulti(&rb, dst, 8u) == 0u);
}

/* ---------------- 用例 9：Clear 只丢数据，不抹统计 ---------------- */

static void Test_Clear(void)
{
    RB_t rb;
    static uint8_t buf[8];
    uint8_t v;
    uint16_t i;

    RB_Init(&rb, buf, 8u);

    for (i = 0u; i < 8u; i++)
    {
        EXPECT(RB_Put(&rb, (uint8_t)i) == 1u);
    }
    EXPECT(RB_Put(&rb, 0xEEu) == 0u);           /* 制造一次溢出 */
    EXPECT(rb.overflow == 1u);

    RB_Clear(&rb);
    EXPECT(RB_Used(&rb) == 0u);
    EXPECT(RB_Free(&rb) == 8u);
    EXPECT(rb.overflow == 1u);                  /* 溢出计数要留着 */

    /* 清完还能正常用 */
    EXPECT(RB_Put(&rb, 0x77u) == 1u);
    EXPECT(RB_Get(&rb, &v) == 1u);
    EXPECT(v == 0x77u);
}

/* ---------------- 入口 ---------------- */

uint16_t TEST_RingBuf_Run(void)
{
    uint16_t i;

    s_result.total  = 0u;
    s_result.failed = 0u;
    for (i = 0u; i < RB_TEST_MAX_RECORDED_FAILS; i++)
    {
        s_result.failLine[i] = 0u;
    }

    Test_InitInvalid();
    Test_InitOk();
    Test_EmptyGet();
    Test_FifoOrder();
    Test_WrapAround();
    Test_FillAndOverflow();
    Test_PutMultiPartial();
    Test_GetMulti();
    Test_Clear();

    return s_result.failed;
}

void TEST_RingBuf_GetResult(RB_TEST_Result_t *out)
{
    uint16_t i;

    if (out == 0)
    {
        return;
    }
    out->total  = s_result.total;
    out->failed = s_result.failed;
    for (i = 0u; i < RB_TEST_MAX_RECORDED_FAILS; i++)
    {
        out->failLine[i] = s_result.failLine[i];
    }
}
