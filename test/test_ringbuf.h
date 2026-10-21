/**
  ******************************************************************************
  * @file    test_ringbuf.h
  * @brief   ringbuf 模块的单元测试接口
  *
  *  和 test_protocol 一样脱离硬件：能编译进 MCU 跑上电自测，
  *  也能在 PC 上用 gcc 直接编译运行（见 test/host_test.c 顶部注释）。
  *
  *  这里刻意用独立的结果类型 RB_TEST_Result_t 而不复用 TEST_Result_t ——
  *  两个测试模块互不依赖，任何一方改头文件都不会连累另一方。
  ******************************************************************************
  */
#ifndef __TEST_RINGBUF_H
#define __TEST_RINGBUF_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#define RB_TEST_MAX_RECORDED_FAILS 16u

typedef struct
{
    uint16_t total;                                         /* 断言总数 */
    uint16_t failed;                                        /* 失败断言数 */
    uint16_t failLine[RB_TEST_MAX_RECORDED_FAILS];          /* 失败所在源码行号 */
} RB_TEST_Result_t;

/**
 * @brief  运行 ringbuf 全部用例
 * @param  None
 * @retval 失败断言数，0 表示全部通过
 */
uint16_t TEST_RingBuf_Run(void);

/**
 * @brief  取最后一次运行的详细结果
 * @param  out: 结果结构体指针
 * @retval None
 */
void TEST_RingBuf_GetResult(RB_TEST_Result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_RINGBUF_H */
