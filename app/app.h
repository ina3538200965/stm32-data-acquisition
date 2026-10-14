/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层：业务逻辑入口（与具体硬件无关）
  *
  *          应用层只调用 driver 提供的"能力"，不碰寄存器、不 include stm32f10x.h。
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  应用层初始化：初始化所有驱动、跑上电自测
 * @param  None
 * @retval None
 */
void APP_Init(void);

/**
 * @brief  应用层主循环体，被 main() 的 while(1) 反复调用
 * @param  None
 * @retval None
 */
void APP_Task(void);

/**
 * @brief  获取上电自测的失败用例数
 * @param  None
 * @retval 0 = 全部通过；其它值 = 失败用例个数；0xFFFF = 尚未运行完成
 */
uint16_t APP_GetSelfTestFailures(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
