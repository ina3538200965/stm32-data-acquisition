/**
  ******************************************************************************
  * @file    test_pipeline.h
  * @brief   接收链路集成测试：字节流 -> 环形缓冲 -> 协议状态机
  *
  *  D7 的交付物就是这张清单上的一句话：
  *      "边界测试记录：半包、粘包、错帧如何被过滤"
  *
  *  这里的测试把 driver_usart 之外的整条链路都串起来了：
  *      串口字节流 ──► 环形缓冲 ──► 逐字节状态机 ──► 完整帧
  *  ringbuf 和 protocol 两层都不依赖硬件，所以能在 PC 上直接跑。
  *  真正贴硬件的那一层（DMA 本身）由 host/monitor.py --boundary 在实机上验证，
  *  两边都过，才能说这条链路是完整可信的。
  ******************************************************************************
  */
#ifndef __TEST_PIPELINE_H
#define __TEST_PIPELINE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#define PL_TEST_MAX_RECORDED_FAILS 16u

typedef struct
{
    uint16_t total;
    uint16_t failed;
    uint16_t failLine[PL_TEST_MAX_RECORDED_FAILS];
} PL_TEST_Result_t;

/**
 * @brief  运行接收链路的全部边界用例
 * @retval 失败断言数，0 表示全部通过
 */
uint16_t TEST_Pipeline_Run(void);

/**
 * @brief  取最后一次运行的详细结果
 */
void TEST_Pipeline_GetResult(PL_TEST_Result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_PIPELINE_H */
