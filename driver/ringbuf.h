/**
  ******************************************************************************
  * @file    ringbuf.h
  * @brief   环形缓冲区（Ring Buffer / Circular Queue），纯软件模块
  *
  *  刻意做成**零硬件依赖**：不 include 任何芯片头文件，只用到 stdint.h。
  *  好处有两个：
  *    1. 能在 PC 上用 gcc 直接编译跑单元测试（见 test/test_ringbuf.c）
  *    2. 换任何单片机都能直接用
  *
  *  用法：缓冲区内存由**调用方**提供（静态数组），本模块不 malloc。
  *
  *  ── 为什么不需要关中断保护 ────────────────────────────────────────────
  *  这是给「单生产者 / 单消费者」用的：
  *      生产者（串口接收中断）只改 head
  *      消费者（主循环）  只改 tail
  *  两边各写各的变量，不存在"两个上下文同时写同一个变量"的情况，
  *  所以天然不需要临界区，也就不用开关中断、不影响中断延迟。
  *
  *  head / tail 用 uint32_t **单调递增、永不取模**，
  *  真正写进数组的才用掩码取低位：
  *      idx  = head & (size - 1)
  *      used = head - tail
  *  这样做有两个好处：
  *    - 满和空不需要牺牲额外的一格去区分：head==tail 就是空，used==size 就是满
  *    - used 直接相减就得到，不用取模，没有除法指令（Cortex-M0 上这点很明显）
  *
  *  head/tail 是 volatile uint32_t，结构体按 4 字节对齐，
  *  Cortex-M3 对**对齐的 32 位访问是单条指令**，读写本身是原子的。
  *
  *  最坏情况下的时序偏差（生产者刚读完 tail 就被打断之类）只会让
  *  算出来的 used / free **偏小**，偏小的结果是"少写一个、少读一个"，
  *  不会写出界也不会读到脏数据 —— 是安全的方向。
  *
  *  head 是 uint32，115200 波特率满负荷约 11.5 KB/s，
  *  回绕一圈需要约 4.3 天。但**回绕不影响正确性**：
  *  used 用的是无符号减法，模 2^32 自洽，跨过 0 也照样算对。
  ******************************************************************************
  */
#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>

/**
 * @brief  环形缓冲控制块
 * @note   调用方负责提供 pBuf 指向的数组，数组长度必须是 2 的幂（256/512/1024...）
 */
typedef struct
{
    uint8_t          *pBuf;      /**< 缓冲区首地址，长度 = mask + 1 */
    uint16_t          mask;      /**< 下标掩码 = size - 1，要求 size 是 2 的幂 */
    volatile uint32_t head;      /**< 写下标，单调递增；只有生产者改 */
    volatile uint32_t tail;      /**< 读下标，单调递增；只有消费者改 */
    volatile uint32_t overflow;  /**< 因缓冲区满而丢弃的字节数；只有生产者改 */
} RB_t;

/**
 * @brief  初始化环形缓冲
 * @param  p        控制块指针
 * @param  buf      调用方提供的缓冲区首地址
 * @param  sizePow2 缓冲区长度，**必须是 2 的幂**
 * @retval 0 = 成功，1 = 参数非法（sizePow2 不是 2 的幂，或指针为空）
 * @note   非法时控制块会被置为"不可用"状态，所有读写函数都安全返回 0
 */
uint8_t RB_Init(RB_t *p, uint8_t *buf, uint16_t sizePow2);

/**
 * @brief  缓冲区里已存了多少字节
 * @param  p  控制块指针
 * @retval 已存字节数，0 ~ size
 */
uint16_t RB_Used(RB_t *p);

/**
 * @brief  还能再写入多少字节
 * @param  p  控制块指针
 * @retval 剩余空间
 */
uint16_t RB_Free(RB_t *p);

/**
 * @brief  写入一个字节
 * @param  p  控制块指针
 * @param  b  要写的字节
 * @retval 1 = 写入成功，0 = 缓冲区满，字节被丢弃（overflow 计数 +1）
 */
uint16_t RB_Put(RB_t *p, uint8_t b);

/**
 * @brief  写入多个字节
 * @param  p    控制块指针
 * @param  src  数据源首地址
 * @param  len  想写多少字节
 * @retval 实际写入的字节数（空间不足时会小于 len，缺的部分计入 overflow）
 * @note   逐字节判满比 Put 一次一次的调用省去重复计算 used
 */
uint16_t RB_PutMulti(RB_t *p, const uint8_t *src, uint16_t len);

/**
 * @brief  读出一个字节
 * @param  p      控制块指针
 * @param  pByte  存放读出的字节
 * @retval 1 = 读成功，0 = 缓冲区空（*pByte 不会被改写）
 */
uint16_t RB_Get(RB_t *p, uint8_t *pByte);

/**
 * @brief  读出多个字节
 * @param  p    控制块指针
 * @param  dst  目标缓冲区首地址
 * @param  max  最多读多少字节
 * @retval 实际读出的字节数
 * @note   一次搬一堆再逐个处理，比反复调用 RB_Get 省掉重复的 used 计算
 */
uint16_t RB_GetMulti(RB_t *p, uint8_t *dst, uint16_t max);

/**
 * @brief  清空缓冲区（丢弃所有已存数据）
 * @param  p  控制块指针
 * @note   只有消费者该调用它；overflow 计数不会被清零
 */
void RB_Clear(RB_t *p);

#endif /* __RINGBUF_H */
