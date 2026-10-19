/**
  ******************************************************************************
  * @file    driver_i2c.h
  * @brief   I2C1 主机驱动（PB6 = SCL，PB7 = SDA，400 kHz 快速模式）
  *
  *  这一层只做"总线读写"，不关心接的是什么器件。
  *  D5 加 I2C OLED 时可以直接复用，不用再写一遍时序。
  ******************************************************************************
  */
#ifndef __DRIVER_I2C_H
#define __DRIVER_I2C_H

#include "stm32f10x.h"

/**
 * @brief  初始化 I2C1：PB6/PB7 复用开漏，400 kHz
 * @param  None
 * @retval None
 */
void DRV_I2C_Init(void);

/**
 * @brief  向器件的一个寄存器写一个字节
 * @param  devAddr  7 位器件地址（不含读写位），如 0x68
 * @param  regAddr  寄存器地址
 * @param  data     要写入的数据
 * @retval 0 = 成功，非 0 = 失败（超时或总线异常）
 */
uint8_t DRV_I2C_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data);

/**
 * @brief  读器件的一个寄存器
 * @param  devAddr  7 位器件地址
 * @param  regAddr  寄存器地址
 * @param  pData    读出的数据存放处
 * @retval 0 = 成功，非 0 = 失败
 */
uint8_t DRV_I2C_ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *pData);

/**
 * @brief  从指定寄存器开始连续读多个字节（器件地址自动递增时必须用这个）
 * @param  devAddr  7 位器件地址
 * @param  regAddr  起始寄存器地址
 * @param  pBuf     接收缓冲区
 * @param  len      要读的字节数
 * @retval 0 = 成功，非 0 = 失败
 */
uint8_t DRV_I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *pBuf, uint16_t len);

#endif /* __DRIVER_I2C_H */
