/**
  ******************************************************************************
  * @file    driver_usart.h
  * @brief   USART1 调试串口（PA9-TX / PA10-RX，115200 8N1）
  *
  *  printf 已重定向到 USART1，主循环里可直接 printf。
  *  前提：Keil 的 Options -> Target -> Use MicroLIB 必须勾选（本工程已开）。
  *
  *  ── D7：接收改成了 DMA + 环形缓冲 ──────────────────────────────────────
  *  两级缓冲，各管一件事：
  *
  *    上位机 ──► USART 硬件 ──DMA─► s_dmaBuf[128] ──中断搬─► ring[256] ──► 主循环
  *              （硬件自动）      （无需 CPU）      （中断里做）      （慢慢消费）
  *
  *  为什么要有两级，不能只用 DMA 那一层：
  *    1. DMA 缓冲只有 128 字节，直接从它里面解析的话，主循环一旦被
  *       printf / OLED 刷新占住几十毫秒，新来的数据就会把它覆盖掉。
  *    2. 128 这个长度也放不下几帧 —— 协议帧最长 39 字节，三帧就满了。
  *  所以中断里只做一件事：把数据从 DMA 缓冲**尽快搬走**，
  *  搬到一个更大的环形缓冲里排队，主循环什么时候有空什么时候来取。
  *
  *  这也是为什么 DMA 用 Circular（循环）模式：
  *  DMA 一旦启动就永远不停，不存在"重启 DMA 那几微秒数据没人收"的窗口。
  ******************************************************************************
  */
#ifndef __DRIVER_USART_H
#define __DRIVER_USART_H

#include "stm32f10x.h"

/* USART1_RX 在 STM32F103 上固定映射到 DMA1_Channel5（硬件连死的，不可改） */
#define DRV_USART_DMA_SIZE   128u   /* DMA 循环缓冲，必须是 2 的幂 */

/* 环形缓冲的容量是**算出来的**，不是拍脑袋定的：
     主循环最慢的一轮 = printf 一行(~7 ms) + OLED 刷新(~28 ms) ≈ 35 ms
     115200 波特率下 1 字节约 87 us，35 ms 里串口最多能收
         35 ms / 87 us ≈ 400 字节
   所以缓冲必须能扛住 400 字节不被消费的窗口，取 2 的幂 -> 512。
   真要是不够用，DRV_USART_RxGetStat 里的 ringOverflow 会非 0，看得出来。
   必须是 2 的幂。 */
#define DRV_USART_RING_SIZE  512u

/**
 * @brief  接收通道的运行统计（D10 稳定性测试要盯着这些数字）
 */
typedef struct
{
    uint32_t idleCount;     /**< IDLE 中断次数 ≈ 上位机发来的批数 */
    uint32_t dmaHtCount;    /**< DMA 半满中断次数 */
    uint32_t dmaTcCount;    /**< DMA 满中断次数 */
    uint32_t dmaBytes;      /**< 从 DMA 缓冲搬进 ring 的总字节数 */
    uint32_t ringOverflow;  /**< ring 满导致丢弃的字节数（主循环太慢的证据） */
    uint32_t hwOverrun;     /**< USART 硬件溢出 ORE 次数，正常应恒为 0 */
    uint16_t ringPeak;      /**< ring 历史最大占用，用来判断容量够不够 */
} DRV_USART_RxStat_t;

/**
 * @brief  初始化 USART1，并配好 DMA 接收 + IDLE 中断
 * @param  None
 * @retval None
 * @note   调用之后接收就自动跑起来了，不需要主循环轮询 ——
 *         数据会自己进环形缓冲，主循环只要调 DRV_USART_RxRead 取即可。
 */
void DRV_USART_Init(void);

/**
 * @brief  发送一段原始字节（发二进制协议帧用，不走 printf 格式化）
 * @param  buf  数据首地址
 * @param  len  字节数
 * @note   阻塞式：115200 下每字节约 87 us，一帧 20 字节约 1.7 ms。
 *         这个耗时必须心里有数 —— 帧发得多的话要考虑改成 DMA 发送。
 */
void DRV_USART_SendBuf(const uint8_t *buf, uint16_t len);

/**
 * @brief  环形缓冲里当前有多少字节等着取
 * @param  None
 * @retval 可读字节数
 */
uint16_t DRV_USART_RxAvailable(void);

/**
 * @brief  从环形缓冲取出一批字节
 * @param  dst  目标缓冲区
 * @param  max  最多取多少字节
 * @retval 实际取到的字节数
 * @note   一次搬一批比一个一个取省事也更快。取出来之后
 *         再逐字节喂给协议状态机即可（见 APP_HandleRx）。
 */
uint16_t DRV_USART_RxRead(uint8_t *dst, uint16_t max);

/**
 * @brief  取接收统计
 * @param  out  统计结构体指针
 * @retval None
 */
void DRV_USART_RxGetStat(DRV_USART_RxStat_t *out);

#endif /* __DRIVER_USART_H */
