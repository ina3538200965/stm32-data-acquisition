/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层：启动各驱动，并把采样结果按 1 秒节拍打印出来
  *
  *  这里体现两条面试常考的规矩：
  *    1) 中断里只置标志、绝不 printf —— printf 不可重入且耗时，会破坏实时性
  *    2) "中断写、主循环读"的共享变量必须 volatile
  ******************************************************************************
  */
#include "app.h"
#include "driver_led.h"
#include "driver_usart.h"
#include "driver_adc.h"
#include "test_protocol.h"
#include <stdio.h>

/* 10 ms 节拍计数，由 stm32f10x_it.c 里的 SysTick_Handler 累加 */
volatile uint32_t g_tick10ms = 0;

/* 上电自测结果：0 = 全通过 */
static uint16_t s_selfTestFailures = 0xFFFFu;

/* CPU 空闲计数：主循环每转一圈 +1。CPU 越忙，一秒内累的数越小 */
static uint32_t s_idleCount = 0;
static uint32_t s_lastTick  = 0;
static uint32_t s_lastRound = 0;

void APP_Init(void)
{
    /* 1. 中断优先级分组，必须在使用任何 NVIC_Init 之前调用一次 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 2. 调试串口（printf 依赖它） */
    DRV_USART_Init();

    /* 3. LED 与 ADC 采样 */
    DRV_LED_Init();
    DRV_ADC_Init();

    /* 4. 上电自测：协议层不依赖硬件，可以先跑一遍 */
    s_selfTestFailures = TEST_Protocol_Run();
    printf("boot ok, selftest fail=%u\r\n", s_selfTestFailures);

    /* 5. SysTick 10 ms 节拍，用于 1 秒统计上报。
          用 10 ms 而不是 1 ms，是为了少打扰 CPU —— 否则每秒 1000 次中断会
          污染 D3 要测的"CPU 占用率"基线。 */
    SysTick_Config(SystemCoreClock / 100u);
}

void APP_Task(void)
{
    /* 空闲计数：这一行就是 D3 的 CPU 占用率基准 */
    s_idleCount++;

    /* 一整轮采样完成 —— 打印原始值 */
    if (g_adcConvFlag)
    {
        g_adcConvFlag = 0;
        printf("CH0=%4u CH1=%4u CH2=%4u\r\n",
               g_adcBuf[0], g_adcBuf[1], g_adcBuf[2]);
    }

    /* 每 100 个节拍 = 1 秒，上报一次统计 */
    if (g_tick10ms - s_lastTick >= 100u)
    {
        /* rounds/s 才是采样率：10 ms 一轮时它应该是 100。
           total 是开机累计值，用来和上位机收包数对比查丢帧（D10 用）。 */
        uint32_t roundsPerSec = g_adcRoundCount - s_lastRound;

        s_lastTick  = g_tick10ms;
        s_lastRound = g_adcRoundCount;
        printf("[1s] idle=%lu rounds/s=%lu total=%lu\r\n",
               s_idleCount, roundsPerSec, g_adcRoundCount);
        s_idleCount = 0;
    }

    DRV_LED_Set(LED_ON);
}

uint16_t APP_GetSelfTestFailures(void)
{
    return s_selfTestFailures;
}
