/**
  ******************************************************************************
  * @file    test_pipeline.c
  * @brief   D7 边界测试：半包 / 粘包 / 错帧分别被怎么处理
  *
  *  串口的本质是一条**字节流**，它根本没有"帧"这个概念。
  *  下面三种情况一定会发生，协议设计必须能兜住：
  *
  *    粘包  两帧连着到达（中间一个字节间隔都没有）
  *          -> 必须解出两帧，不能当成一坨
  *    半包  一帧被切成两半，后半段几十毫秒后才到
  *          -> 前半段不能误判成帧，要等到后半段拼上才出结果
  *    错帧  传输中某个字节被打坏，CRC 对不上
  *          -> 必须拒掉，而且要从下一个帧头自动重新同步
  *
  *  这里用一个 64 字节的小环形缓冲，比实际用的 256 更容易
  *  把"缓冲不够用"的情况也逼出来。
  ******************************************************************************
  */
#include "test_pipeline.h"
#include "ringbuf.h"
#include "protocol.h"
#include <string.h>

static PL_TEST_Result_t s_result;

/* 模拟设备端的接收缓冲与主循环里的解析器 */
static uint8_t        s_ringBuf[64];
static RB_t           s_ring;
static PROTO_Parser_t s_parser;

/* ---------------- 断言基础设施 ---------------- */

static void RecordFail(uint16_t line)
{
    if (s_result.failed < PL_TEST_MAX_RECORDED_FAILS)
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

/* ---------------- 模拟工具 ---------------- */

/* 模拟"DMA 中断把收到的数据搬进环形缓冲"这一步 */
static void RxPut(const uint8_t *src, uint16_t len)
{
    EXPECT(RB_PutMulti(&s_ring, src, len) == len);
}

/* 模拟"主循环取出一批数据喂给状态机"，返回这一轮解出了几帧 */
static uint16_t Pump(uint16_t *cmds, uint16_t maxCmds)
{
    uint8_t  buf[32];
    uint16_t n;
    uint16_t i;
    uint16_t got = 0u;
    PROTO_Frame_t frame;

    n = RB_GetMulti(&s_ring, buf, (uint16_t)sizeof(buf));

    for (i = 0u; i < n; i++)
    {
        if (PROTO_Feed(&s_parser, buf[i], &frame) == PROTO_OK)
        {
            if (got < maxCmds)
            {
                cmds[got] = frame.cmd;
            }
            got++;
        }
    }
    return got;
}

/* 每个用例前复位缓冲和状态机 */
static void Reset(void)
{
    RB_Init(&s_ring, s_ringBuf, (uint16_t)sizeof(s_ringBuf));
    RB_Clear(&s_ring);
    PROTO_ParserInit(&s_parser);
}

/* 造一条 GET_INFO 命令帧（7 字节），顺便返回它的长度 */
static uint16_t MakeInfoFrame(uint8_t *out, uint16_t outSize)
{
    return PROTO_Pack(CMD_GET_INFO, 0, 0u, out, outSize);
}

/* ---------------- 用例 1：粘包，两帧连着到 ---------------- */

static void Test_Sticky(void)
{
    uint8_t  stream[16];
    uint8_t  f[PROTO_FRAME_MAX_LEN];
    uint16_t flen;
    uint16_t cmds[4];

    Reset();
    flen = MakeInfoFrame(f, (uint16_t)sizeof(f));
    EXPECT(flen == 7u);

    /* 两帧背靠背，中间不留任何空隙 */
    memcpy(stream, f, flen);
    memcpy(&stream[flen], f, flen);

    RxPut(stream, (uint16_t)(flen * 2u));
    EXPECT(Pump(cmds, 4u) == 2u);           /* 必须解出 2 帧 */
    EXPECT(cmds[0] == CMD_GET_INFO);
    EXPECT(cmds[1] == CMD_GET_INFO);
    EXPECT(RB_Used(&s_ring) == 0u);
}

/* ---------------- 用例 2：半包，一帧来两次 ---------------- */

static void Test_HalfPacket(void)
{
    uint8_t  f[PROTO_FRAME_MAX_LEN];
    uint16_t flen;
    uint16_t cut;
    uint16_t cmds[4];

    Reset();
    flen = MakeInfoFrame(f, (uint16_t)sizeof(f));
    cut  = flen / 2u;

    /* 前半段到了：这时候还差着呢，绝不能解出帧 */
    RxPut(f, cut);
    EXPECT(Pump(cmds, 4u) == 0u);
    EXPECT(RB_Used(&s_ring) == 0u);

    /* 后半段姗姗来迟：拼上之后这一帧才该出结果 */
    RxPut(&f[cut], (uint16_t)(flen - cut));
    EXPECT(Pump(cmds, 4u) == 1u);
    EXPECT(cmds[0] == CMD_GET_INFO);
}

/* ---------------- 用例 3：一个字节一个字节地来 ---------------- */

static void Test_ByteByByte(void)
{
    uint8_t  f[PROTO_FRAME_MAX_LEN];
    uint16_t flen;
    uint16_t i;
    uint16_t got = 0u;
    uint16_t cmds[4];

    Reset();
    flen = MakeInfoFrame(f, (uint16_t)sizeof(f));

    /* 每来一个字节就跑一轮主循环，模拟 115200 下最慢的到达方式 */
    for (i = 0u; i < flen; i++)
    {
        EXPECT(RB_Put(&s_ring, f[i]) == 1u);
        got += Pump(cmds, 4u);
    }

    /* 只有最后一个字节喂进去之后，才会正好吐出一帧 */
    EXPECT(got == 1u);
    EXPECT(cmds[0] == CMD_GET_INFO);
}

/* ---------------- 用例 4：错帧被拒，且能自动重同步 ---------------- */

static void Test_BadCrcResync(void)
{
    uint8_t  f[PROTO_FRAME_MAX_LEN];
    uint8_t  bad[PROTO_FRAME_MAX_LEN];
    uint16_t flen;
    uint16_t cmds[4];

    Reset();
    flen = MakeInfoFrame(f, (uint16_t)sizeof(f));

    /* 打坏 CRC 的低字节（倒数第 3 个字节：CRC 2 字节 + 帧尾 1 字节） */
    memcpy(bad, f, flen);
    bad[flen - 3u] ^= 0xFFu;

    /* 坏帧 + 好帧连着发：坏的必须被扔掉，好的必须被留下 */
    RxPut(bad, flen);
    RxPut(f, flen);

    EXPECT(Pump(cmds, 4u) == 1u);       /* 只有好帧出结果 */
    EXPECT(cmds[0] == CMD_GET_INFO);
}

/* ---------------- 用例 5：混进垃圾数据也能重同步 ---------------- */

static void Test_GarbageResync(void)
{
    static const uint8_t garbage[7] = { 0xFF, 0x00, 0xAA, 0x55, 0x7F, 0x13, 0x99 };
    uint8_t  f[PROTO_FRAME_MAX_LEN];
    uint16_t flen;
    uint16_t cmds[4];

    Reset();
    flen = MakeInfoFrame(f, (uint16_t)sizeof(f));

    /* 垃圾本来就在（比如上电瞬间的毛刺、波特率不稳），
       协议层必须能靠帧头重新找到对齐位置 */
    RxPut(garbage, (uint16_t)sizeof(garbage));
    RxPut(f, flen);

    EXPECT(Pump(cmds, 4u) == 1u);
    EXPECT(cmds[0] == CMD_GET_INFO);
}

/* ---------------- 用例 6：环形缓冲装不下时的行为 ---------------- */

static void Test_RingOverflow(void)
{
    uint8_t  f[PROTO_FRAME_MAX_LEN];
    uint16_t flen;
    uint16_t i;
    uint16_t cmds[4];

    Reset();
    flen = MakeInfoFrame(f, (uint16_t)sizeof(f));

    /* 用垃圾把 64 字节的缓冲塞爆（实际只能装 64 字节） */
    for (i = 0u; i < 80u; i++)
    {
        RB_Put(&s_ring, 0x5Au);
    }
    EXPECT(s_ring.overflow == 16u);         /* 80 - 64 = 16 个被丢掉 */
    EXPECT(RB_Used(&s_ring) == 64u);

    /* 缓冲满了以后再来一帧，写不进去但不许崩、不许污染已有数据 */
    EXPECT(RB_PutMulti(&s_ring, f, flen) == 0u);
    EXPECT(RB_Used(&s_ring) == 64u);        /* 老数据必须还在 */
    EXPECT(s_ring.overflow == (uint32_t)(16u + flen));

    /* 把缓冲腾空，再来的帧依然能正常解析 —— 说明没被卡死 */
    RB_Clear(&s_ring);
    RxPut(f, flen);
    EXPECT(Pump(cmds, 4u) == 1u);
    EXPECT(cmds[0] == CMD_GET_INFO);
}

/* ---------------- 入口 ---------------- */

uint16_t TEST_Pipeline_Run(void)
{
    uint16_t i;

    s_result.total  = 0u;
    s_result.failed = 0u;
    for (i = 0u; i < PL_TEST_MAX_RECORDED_FAILS; i++)
    {
        s_result.failLine[i] = 0u;
    }

    Test_Sticky();
    Test_HalfPacket();
    Test_ByteByByte();
    Test_BadCrcResync();
    Test_GarbageResync();
    Test_RingOverflow();

    return s_result.failed;
}

void TEST_Pipeline_GetResult(PL_TEST_Result_t *out)
{
    uint16_t i;

    if (out == 0)
    {
        return;
    }
    out->total  = s_result.total;
    out->failed = s_result.failed;
    for (i = 0u; i < PL_TEST_MAX_RECORDED_FAILS; i++)
    {
        out->failLine[i] = s_result.failLine[i];
    }
}
