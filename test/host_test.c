/**
  ******************************************************************************
  * @file    host_test.c
  * @brief   PC 端测试入口 —— 证明 protocol / ringbuf / test 这三层真的脱离了硬件
  *
  *  注意本文件不加入 Keil 工程（它自带 main()，会和 User/main.c 冲突），
  *  只在命令行里用 gcc 编译运行：
  *
  *      cd test
  *      gcc -std=c99 -Wall -Wextra -pedantic \
  *          -I../protocol -I../driver \
  *          ../protocol/protocol.c ../driver/ringbuf.c \
  *          test_protocol.c test_ringbuf.c host_test.c \
  *          -o host_test.exe
  *      ./host_test.exe
  *
  *  退出码 = 失败断言数，0 表示全通过，方便接到 CI 里。
  ******************************************************************************
  */
#include <stdio.h>
#include "test_protocol.h"
#include "test_ringbuf.h"
#include "test_pipeline.h"

int main(void)
{
    TEST_Result_t pr;
    RB_TEST_Result_t rr;
    PL_TEST_Result_t plr;
    uint16_t fails = 0u;
    uint16_t total = 0u;
    uint16_t i;

    /* ---- protocol 层 ---- */
    fails += TEST_Protocol_Run();
    TEST_Protocol_GetResult(&pr);
    total += pr.total;

    printf("[protocol] assertions: %u, failed: %u\r\n",
           (unsigned)pr.total, (unsigned)pr.failed);
    for (i = 0u; (i < TEST_MAX_RECORDED_FAILS) && (i < pr.failed); i++)
    {
        printf("  -> fail at test_protocol.c line %u\r\n", (unsigned)pr.failLine[i]);
    }
    printf("[protocol] %s\r\n", (pr.failed == 0u) ? "ALL PASSED" : "FAILED");

    /* ---- ringbuf 层 ---- */
    fails += TEST_RingBuf_Run();
    TEST_RingBuf_GetResult(&rr);
    total += rr.total;

    printf("[ringbuf ] assertions: %u, failed: %u\r\n",
           (unsigned)rr.total, (unsigned)rr.failed);
    for (i = 0u; (i < RB_TEST_MAX_RECORDED_FAILS) && (i < rr.failed); i++)
    {
        printf("  -> fail at test_ringbuf.c line %u\r\n", (unsigned)rr.failLine[i]);
    }
    printf("[ringbuf ] %s\r\n", (rr.failed == 0u) ? "ALL PASSED" : "FAILED");

    /* ---- 接收链路集成：字节流 -> 环形缓冲 -> 状态机（D7 边界测试）---- */
    fails += TEST_Pipeline_Run();
    TEST_Pipeline_GetResult(&plr);
    total += plr.total;

    printf("[pipeline] assertions: %u, failed: %u\r\n",
           (unsigned)plr.total, (unsigned)plr.failed);
    for (i = 0u; (i < PL_TEST_MAX_RECORDED_FAILS) && (i < plr.failed); i++)
    {
        printf("  -> fail at test_pipeline.c line %u\r\n", (unsigned)plr.failLine[i]);
    }
    printf("[pipeline] %s\r\n", (plr.failed == 0u) ? "ALL PASSED" : "FAILED");

    printf("[total   ] assertions: %u, failed: %u\r\n",
           (unsigned)total, (unsigned)fails);
    return (int)fails;
}
