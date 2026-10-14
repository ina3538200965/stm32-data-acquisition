/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层实现：把"点亮 LED"这件事表达成业务逻辑
  *
  *          设计要点：这里的每一行都应该能对着面试官讲清楚"为什么"，
  *          而不是从示例工程里抄过来。
  ******************************************************************************
  */
#include "app.h"
#include "driver_led.h"
#include "test_protocol.h"

/* 上电自测结果：0 = 全通过。放在静态变量里，调试时可以在 Watch 窗口直接看，
   不依赖串口打印 —— 这也是"验证必须可观测"的最低实现。 */
static uint16_t s_selfTestFailures = 0xFFFFu;

void APP_Init(void)
{
    /* 1. 硬件层就绪 */
    DRV_LED_Init();

    /* 2. 上电自测：协议层不依赖硬件，可以在 MCU 启动阶段就跑一遍。
          纯软件模块能被"先验证再使用"，正是分层带来的直接好处。 */
    s_selfTestFailures = TEST_Protocol_Run();
}

void APP_Task(void)
{
    /* 业务逻辑：点亮板上 LED */
    DRV_LED_Set(LED_ON);
}

uint16_t APP_GetSelfTestFailures(void)
{
    return s_selfTestFailures;
}
