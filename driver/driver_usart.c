/**
  * @file    driver_usart.c
  * @brief   USART1 调试串口 + printf 重定向
  *
  *  波特率计算：
  *    USART1 挂 APB2，PCLK2 = 72 MHz
  *    波特率 = 外设时钟 / (16 x USARTDIV)
  *    115200 = 72000000 / (16 x 39.0625)  -> 
  */
  
#include "driver_usart.h"
#include "ringbuf.h"
#include <stdio.h>

/* ---- D7 接收通道的两级缓冲 ---- */
static uint8_t  s_dmaBuf[DRV_USART_DMA_SIZE];    /* DMA 直接往这里写 */
static uint8_t  s_ringBuf[DRV_USART_RING_SIZE];  /* 中断搬到这边排队 */
static RB_t     s_rxRing;
static uint16_t s_dmaLast;                       /* 上次搬运时 DMA 写到哪了 */

static volatile uint32_t s_idleCount;
static volatile uint32_t s_dmaHtCount;
static volatile uint32_t s_dmaTcCount;
static volatile uint32_t s_dmaBytes;
static volatile uint32_t s_hwOverrun;
static volatile uint16_t s_ringPeak;

/**
 * @brief  初始化 USART1：PA9 复用推挽 TX，PA10 浮空输入 RX
 * @param  None
 * @retval None
 */
void DRV_USART_Init(void)
{
	/* 开GPIO、USART时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	/* 创结构体 */
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	DMA_InitTypeDef  dma;
	NVIC_InitTypeDef nvic;

    /* TX: PA9 复用推挽输出 */
	gpio.GPIO_Mode  =GPIO_Mode_AF_PP;
	gpio.GPIO_Pin   =GPIO_Pin_9;
	gpio.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);

    /* RX: PA10 浮空输入 */
	gpio.GPIO_Mode  =GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin   =GPIO_Pin_10;
	GPIO_Init(GPIOA,&gpio);

	usart.USART_BaudRate             =115200;
	usart.USART_Mode                 =USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_WordLength           =USART_WordLength_8b;
	usart.USART_StopBits             =USART_StopBits_1;
	usart.USART_Parity               =USART_Parity_No;
	usart.USART_HardwareFlowControl  =USART_HardwareFlowControl_None;
	USART_Init(USART1, &usart);

    USART_Cmd(USART1, ENABLE);

    /* ---- 接收：DMA Circular + IDLE 中断 + 环形缓冲（D7）----
       思路是让硬件自己收，CPU 只在"该搬了"的时候动一下：
         · DMA Circular：DMA 永不停歇，硬件收到就自动写进 s_dmaBuf
         · IDLE 中断    ：上位机一帧发完，总线空闲立刻通知我们，延迟最低
         · DMA HT/TC    ：兜底。上位机连着猛发时 IDLE 可能一直不来，
                          没有这个的话 128 字节会被后面的数据覆盖
       三者都只做同一件事：调 USART_PumpDma 把新数据搬进 ring。 */

    (void)RB_Init(&s_rxRing, s_ringBuf, DRV_USART_RING_SIZE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);

    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);   /* 源：串口数据寄存器 */
    dma.DMA_MemoryBaseAddr     = (uint32_t)s_dmaBuf;        /* 目的：内存数组 */
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize         = DRV_USART_DMA_SIZE;        /* 单元个数 = 字节数 */
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable; /* 源地址固定 */
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;      /* 目的地址递增 */
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Circular;         /* 循环，不重启 */
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &dma);

    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC | DMA_IT_HT, ENABLE);

    /* 这里是**两个中断共用一个生产者角色**（USART1 的 IDLE 和 DMA 的 HT/TC
       都会去改 head），所以两者必须配成**同一个抢占优先级**。
       抢占优先级相同就不会互相打断，head 的读-改-写不会被撕裂成两步；
       要是给它们设了不同优先级，高优先级那个可能打断低优先级那个的
       head++，环形缓冲的免锁前提就被破坏了。 */
    nvic.NVIC_IRQChannel                   = DMA1_Channel5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* USART 侧也要开接收 DMA 请求 —— 和 ADC 一样，外设和 DMA 两端都要使能 */
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    NVIC_Init(&nvic);

    DMA_Cmd(DMA1_Channel5, ENABLE);
}

/**
 * @brief  发送一段原始字节
 * @note   顺序是"先写再等"：写 DR 后硬件把数据搬进移位寄存器时 TXE 会置 1，
 *         等到 TXE=1 就说明可以写下一个字节了。
 *         上一个字节实际还在往外发，但那不影响 —— TXE 保证的是 DR 已空。
 */
void DRV_USART_SendBuf(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == 0)
    {
        return;
    }

    for (i = 0u; i < len; i++)
    {
        USART_SendData(USART1, buf[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
            /* 等这个字节进入移位寄存器 */
        }
    }
}

/**
 * @brief  printf 重定向：让 printf 走 USART1
 * @note   只有勾选了 MicroLIB，printf 才会调用这个 fputc
 * @retval 返回写入的字符
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    USART_SendData(USART1, (uint8_t)ch);
    /* TXE 置位表示数据已进入发送移位寄存器，可以写下一个字节 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}

/* ==================== D7：DMA 接收 ==================== */

/**
 * @brief  把 DMA 缓冲里新到的字节搬进环形缓冲
 * @retval None
 * @note   只在中断里调用。
 *         CNDTR 是"还剩几个字节没搬"的递减计数，所以
 *             DMA_SIZE - CNDTR = DMA 下一个要写的下标（0 ~ 127）
 *         它和上次记录的 s_dmaLast 之间夹着的那段，就是这段时间新收到的。
 *         因为 DMA 是 Circular，新数据很可能绕回了缓冲开头，
 *         所以要分"没绕过 / 绕过了"两种情况处理。
 */
static void USART_PumpDma(void)
{
    uint16_t cnt;
    uint16_t cur;
    uint16_t total;
    uint16_t used;

    cnt = (uint16_t)DMA_GetCurrDataCounter(DMA1_Channel5);
    cur = (uint16_t)(DRV_USART_DMA_SIZE - cnt);

    /* CNDTR 归零那一瞬等价于回到开头，用掩码把 128 折算回 0 */
    cur &= (uint16_t)(DRV_USART_DMA_SIZE - 1u);

    if (cur == s_dmaLast)
    {
        return;                                     /* 没有新数据 */
    }

    if (cur > s_dmaLast)
    {
        /* 没绕回，一整段连续内存 */
        total = (uint16_t)(cur - s_dmaLast);
        RB_PutMulti(&s_rxRing, &s_dmaBuf[s_dmaLast], total);
    }
    else
    {
        /* 绕回了开头，分成"尾巴 + 开头"两段搬 */
        total = (uint16_t)((DRV_USART_DMA_SIZE - s_dmaLast) + cur);
        RB_PutMulti(&s_rxRing, &s_dmaBuf[s_dmaLast],
                    (uint16_t)(DRV_USART_DMA_SIZE - s_dmaLast));
        RB_PutMulti(&s_rxRing, &s_dmaBuf[0], cur);
    }

    s_dmaLast = cur;
    s_dmaBytes += total;

    /* 记录历史最大占用 —— 这个值接近 256 就说明主循环快扛不住了 */
    used = RB_Used(&s_rxRing);
    if (used > s_ringPeak)
    {
        s_ringPeak = used;
    }
}

/**
 * @brief  环形缓冲里有多少字节等着取
 * @param  None
 * @retval 可读字节数
 */
uint16_t DRV_USART_RxAvailable(void)
{
    return RB_Used(&s_rxRing);
}

/**
 * @brief  从环形缓冲取出一批字节
 * @param  dst  目标缓冲区
 * @param  max  最多取多少
 * @retval 实际取到的字节数
 */
uint16_t DRV_USART_RxRead(uint8_t *dst, uint16_t max)
{
    return RB_GetMulti(&s_rxRing, dst, max);
}

/**
 * @brief  取接收统计
 * @param  out  统计结构体指针
 * @retval None
 */
void DRV_USART_RxGetStat(DRV_USART_RxStat_t *out)
{
    if (out == 0)
    {
        return;
    }
    out->idleCount    = (uint32_t)s_idleCount;
    out->dmaHtCount   = (uint32_t)s_dmaHtCount;
    out->dmaTcCount   = (uint32_t)s_dmaTcCount;
    out->dmaBytes     = (uint32_t)s_dmaBytes;
    out->ringOverflow = (uint32_t)s_rxRing.overflow;
    out->hwOverrun    = (uint32_t)s_hwOverrun;
    out->ringPeak     = s_ringPeak;
}

/**
 * @brief  USART1 中断：只处理总线空闲 IDLE
 * @param  None
 * @retval None
 * @note   **IDLE 标志清不掉是新手最常见的坑**：
 *         它对 USART_ClearFlag 无效，手册规定的清法是依次"读 SR、读 DR"。
 *         这里读 DR 没有副作用 —— DMA 模式下收到的字节早已被 DMA 读走，
 *         DR 是空的，这一读纯粹为了清标志。
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        (void)USART1->SR;
        (void)USART1->DR;

        s_idleCount++;
        USART_PumpDma();
    }

    /* 硬件溢出：上一个字节还没被取走，新的又塞进来了。
       DMA Circular 全程在线时理论上不该发生，计数恒 >0 说明配置有问题。 */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART1->SR;
        (void)USART1->DR;
        s_hwOverrun++;
    }
}

/**
 * @brief  DMA1 通道 5 中断：半满 / 满，给 IDLE 兜底
 * @param  None
 * @retval None
 * @note   上位机连续猛发时 IDLE 可能迟迟不来，靠 HT/TC 保证
 *         最多攒到半个缓冲就一定搬一次，128 字节不会被覆盖。
 *         正常低速通信（几帧/秒）下这两个计数应该几乎不动，
 *         全是 IDLE 在干活。
 */
void DMA1_Channel5_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC5) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC5);
        s_dmaTcCount++;
        USART_PumpDma();
    }

    if (DMA_GetITStatus(DMA1_IT_HT5) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_HT5);
        s_dmaHtCount++;
        USART_PumpDma();
    }
}
