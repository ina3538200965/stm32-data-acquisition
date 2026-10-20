/**
  ******************************************************************************
  * @file    driver_w25q64.h
  * @brief   W25Q64（8 MB SPI Flash）器件层驱动
  *
  *  接线（与 OLED / MPU6500 不冲突，SPI 是独立总线）：
  *      PA4 = CS（软件 GPIO 控制）  PA5 = SCK
  *      PA6 = MISO（DO）           PA7 = MOSI（DI）
  *
  *  ⚠️ Flash 和 EEPROM/RAM 最大的三点区别，写驱动前必须先想清楚：
  *
  *  1. **只能把 1 写成 0，不能把 0 写成 1**。要改回 1 只能擦除。
  *     所以"改写"实际上是「擦除 → 写入」两步，擦除的最小单位是 4 KB sector，
  *     哪怕你只想改 1 个字节，也得擦掉整个 4 KB。
  *
  *  2. **写之前必须先发 Write Enable（0x06）**，而且这个使能只管一次操作，
  *     每次写/擦之前都要重新发一次。写完芯片自动清零 WEL 位。
  *
  *  3. **有擦写寿命**，典型 10 万次。每秒擦写一次的话，一天多就废了。
  *     所以运行时不能高频写，要做磨损均衡或降低频率（本项目用降频方案）。
  *
  *  地址是 24 位，容量 8 MB = 0x000000 ~ 0x7FFFFF。
  ******************************************************************************
  */
#ifndef __DRIVER_W25Q64_H
#define __DRIVER_W25Q64_H

#include "stm32f10x.h"

/* ---- 容量与几何参数 ---- */
#define W25Q64_PAGE_SIZE        256u        /* 一次 Page Program 最多写这么多 */
#define W25Q64_SECTOR_SIZE      4096u       /* 最小擦除单位 */
#define W25Q64_TOTAL_SIZE       0x800000u   /* 8 MB */

/* JEDEC ID：厂商 0xEF（华邦），器件 0x4017 = W25Q64 */
#define W25Q64_MANUF_ID         0xEFu
#define W25Q64_JEDEC_ID         0x4017u

/**
 * @brief  初始化：开 SPI、读 JEDEC ID 确认器件在线
 * @param  None
 * @retval 0 = 成功
 *         1 = ID 读不到（没接 / 线错了）
 *         2 = 厂商 ID 不是 0xEF
 */
uint8_t DRV_W25Q64_Init(void);

/**
 * @brief  读 JEDEC ID
 * @param  pManuf  厂商 ID 存放处（应为 0xEF）
 * @param  pDevID  器件 ID 存放处（16 位，W25Q64 = 0x4017）
 * @retval 0 = 成功，非 0 = 失败
 */
uint8_t DRV_W25Q64_ReadID(uint8_t *pManuf, uint16_t *pDevID);

/**
 * @brief  从任意地址读任意长度（没有页限制，读可以一直往下走）
 * @param  addr  24 位地址
 * @param  pBuf  接收缓冲区
 * @param  len   字节数
 * @retval 0 = 成功，非 0 = 失败
 */
uint8_t DRV_W25Q64_Read(uint32_t addr, uint8_t *pBuf, uint16_t len);

/**
 * @brief  写一页（最多 256 字节，且**不能跨页**）
 * @param  addr  起始地址
 * @param  pBuf  数据
 * @param  len   字节数，1~256
 * @retval 0 = 成功
 *         1 = 参数非法
 *         2 = 跨页（调用方应拆开分两次写）
 * @note   **调用前目标区域必须是已擦除状态**（全 0xFF），否则写不进去。
 */
uint8_t DRV_W25Q64_PageProgram(uint32_t addr, const uint8_t *pBuf, uint16_t len);

/**
 * @brief  自动分页写，跨页时拆成多次 Page Program
 * @param  addr  起始地址
 * @param  pBuf  数据
 * @param  len   字节数
 * @retval 0 = 成功，非 0 = 失败
 * @note   同样要求目标区域已擦除。它只帮你处理分页，不帮你擦除 ——
 *         擦除要不要做、什么时候做，是调用方的策略决定。
 */
uint8_t DRV_W25Q64_Write(uint32_t addr, const uint8_t *pBuf, uint16_t len);

/**
 * @brief  擦除一个 4 KB sector
 * @param  addr  该 sector 内的任意地址都行
 * @retval 0 = 成功，非 0 = 失败
 * @note   擦除是毫秒级操作（典型 30~50 ms，最长 400 ms），**必须在主循环里做**。
 */
uint8_t DRV_W25Q64_SectorErase(uint32_t addr);

/**
 * @brief  等芯片忙完（状态寄存器 BUSY 位清零）
 * @retval 0 = 空闲，非 0 = 超时
 */
uint8_t DRV_W25Q64_WaitBusy(void);

#endif /* __DRIVER_W25Q64_H */
