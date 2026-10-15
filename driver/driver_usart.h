/**
  ******************************************************************************
  * @file    driver_usart.h
  * @brief   USART1 调试串口（PA9-TX / PA10-RX，115200 8N1）
  *
  *  printf 已重定向到 USART1，主循环里可直接 printf。
  *  前提：Keil 的 Options -> Target -> Use MicroLIB 必须勾选（本工程已开）。
  ******************************************************************************
  */
#ifndef __DRIVER_USART_H
#define __DRIVER_USART_H

#include "stm32f10x.h"

/**
 * @brief  初始化 USART1，波特率 115200
 * @param  None
 * @retval None
 */
void DRV_USART_Init(void);

#endif /* __DRIVER_USART_H */
