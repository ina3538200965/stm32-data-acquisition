/**
  ******************************************************************************
  * @file    test_protocol.h
  * @brief   protocol 层的单元测试接口
  *
  *          这块测试代码同样不依赖硬件：既能编译进 MCU 跑上电自测，
  *          也能在 PC 上用 gcc 直接编译运行（见 host_test.c 顶部注释）。
  ******************************************************************************
  */
#ifndef __TEST_PROTOCOL_H
#define __TEST_PROTOCOL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/* 最多记录的失败行数，防止失败洪水把静态数组撑爆 */
#define TEST_MAX_RECORDED_FAILS 16u

typedef struct
{
    uint16_t total;                                     /* 断言总数 */
    uint16_t failed;                                    /* 失败断言数 */
    uint16_t failLine[TEST_MAX_RECORDED_FAILS];         /* 失败所在的源码行号 */
} TEST_Result_t;

/**
 * @brief  运行 protocol 全部用例
 * @param  None
 * @retval 失败断言数，0 表示全部通过
 */
uint16_t TEST_Protocol_Run(void);

/**
 * @brief  取最后一次运行的详细结果
 * @param  out: 结果结构体指针
 * @retval None
 */
void TEST_Protocol_GetResult(TEST_Result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_PROTOCOL_H */
