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

/**
 * @brief  发送一段原始字节（发二进制协议帧用，不走 printf 格式化）
 * @param  buf  数据首地址
 * @param  len  字节数
 * @note   阻塞式：115200 下每字节约 87 µs，一帧 20 字节约 1.7 ms。
 *         这个耗时必须心里有数 —— 帧发得多的话要考虑改成 DMA 发送。
 */
void DRV_USART_SendBuf(const uint8_t *buf, uint16_t len);

#endif /* __DRIVER_USART_H */
