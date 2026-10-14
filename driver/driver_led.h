/**
  ******************************************************************************
  * @file    driver_led.h
  * @brief   On-board LED low-level driver (硬件相关层)
  *
  *          这一层是唯一允许直接操作寄存器的地方。
  *          换芯片（F103 -> F407 / STM32 -> GD32）时，只需要重写本文件，
  *          app / protocol / test 三层不需要改动 —— 这就是分层要达成的目标。
  ******************************************************************************
  */
#ifndef __DRIVER_LED_H
#define __DRIVER_LED_H

#include <stdint.h>

/**
 * @brief LED 逻辑电平
 *        蓝丸板 PC13 是灌电流接法：引脚拉低 = 点亮。
 *        上层只认 LED_ON / LED_OFF，不认 0/1 的物理含义 —— 把"反逻辑"关在 driver 里。
 */
#define LED_ON      ((uint8_t)0)     /* GPIO 输出低电平 -> LED 亮 */
#define LED_OFF     ((uint8_t)1)     /* GPIO 输出高阻/高电平 -> LED 灭 */

/* ========================= 对外接口 ========================= */

/**
 * @brief  初始化 LED 所用 GPIO（开时钟 + 配置引脚）
 * @param  None
 * @retval None
 */
void DRV_LED_Init(void);

/**
 * @brief  设置 LED 开关状态
 * @param  state: LED_ON 或 LED_OFF
 * @retval None
 */
void DRV_LED_Set(uint8_t state);

/**
 * @brief  LED 状态翻转
 * @param  None
 * @retval None
 */
void DRV_LED_Toggle(void);

/**
 * @brief  读取当前 LED 状态（软件镜像值，不去读寄存器）
 * @param  None
 * @retval LED_ON 或 LED_OFF
 */
uint8_t DRV_LED_GetState(void);

#endif /* __DRIVER_LED_H */
