/**
  ******************************************************************************
  * @file    driver_adc.h
  * @brief   ADC 驱动：TIM3 更新事件(TRGO) 硬件触发 ADC1 多通道扫描
  *
  *  数据流：TIM3 溢出 -> TRGO 脉冲 -> ADC1 启动 -> 逐通道转换
  *          -> 每通道 EOC 中断 -> 中断里读 DR 存进 g_adcBuf
  *
  *  @note   所有"中断写、主循环读"的变量必须加 volatile：
  *          否则编译器会把主循环里的读取优化成只读一次，永远看不到更新。
  ******************************************************************************
  */
#ifndef __DRIVER_ADC_H
#define __DRIVER_ADC_H

#include "stm32f10x.h"

/* 采样通道数：PA0 / PA1 / PA2 */
#define ADC_CH_NUM      3u

/* ---- 中断写和主循环读 都用volatile ---- */
extern volatile uint16_t g_adcBuf[ADC_CH_NUM];   /* 三路原始值，12 位右对齐 0~4095 */
extern volatile uint8_t  g_adcConvFlag;          /* 1 = 一整轮采样完成 */
extern volatile uint32_t g_adcRoundCount;        /* 累计采样轮数，用于丢帧统计 */

/**
 * @brief  初始化 ADC1 + TIM3(10ms 触发) + PA8 打点引脚 + NVIC
 * @param  None
 * @retval None
 */
void DRV_ADC_Init(void);

#endif /* __DRIVER_ADC_H */
