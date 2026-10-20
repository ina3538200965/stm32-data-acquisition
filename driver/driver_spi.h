/**
  ******************************************************************************
  * @file    driver_spi.h
  * @brief   SPI1 主机驱动（PA5=SCK，PA6=MISO，PA7=MOSI）
  *
  *  这一层只做"交换一个字节"，不关心接的是什么器件，也不管片选（CS）。
  *  CS 由器件层自己用普通 GPIO 控 —— 一条 SPI 总线上挂多个器件时，
  *  每个器件有自己的 CS，靠它来选通。用硬件 NSS 只能挂一个，不灵活。
  *
  *  ⚠️ SPI 和 I2C 最大的区别：SPI 没有"地址"概念，也没有应答位。
  *     主机想读数据，必须主动发一个字节（通常发 0xFF 这种无效值）把数据"换"回来。
  *     所以这里的接口叫 SwapByte（交换），不是 Send 也不是 Receive。
  ******************************************************************************
  */
#ifndef __DRIVER_SPI_H
#define __DRIVER_SPI_H

#include "stm32f10x.h"

/**
 * @brief  初始化 SPI1：PA5/PA6/PA7 复用推挽，18 MHz，模式 0
 * @param  None
 * @retval None
 * @note   SPI1 挂 APB2（72 MHz），4 分频 = 18 MHz。W25Q64 能跑到 80 MHz，
 *         这里降速是留余量给长杜邦线。
 */
void DRV_SPI_Init(void);

/**
 * @brief  交换一个字节：发出 txData，同时收回从机返回的字节
 * @param  txData  要发的字节。读数据时一般填 0xFF（Dummy Byte）
 * @retval 从机返回的字节
 */
uint8_t DRV_SPI_SwapByte(uint8_t txData);

#endif /* __DRIVER_SPI_H */
