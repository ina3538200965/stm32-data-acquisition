/**
  ******************************************************************************
  * @file    driver_led.c
  * @brief   On-board LED driver implementation for STM32F103C8 (Blue Pill)
  *
  *          硬件事实（面试时可以把这段讲出来）：
  *          - PC13 的输出能力很弱（与 RTC/TAMPER 共用，最大 3mA），所以只能
  *            工作在 GPIO_Mode_Out_OD 开漏模式，靠外部上拉 + 灌电流点亮 LED，
  *            不能用推挽直接往外灌。
  *          - 因此 LED 是"低电平点亮"，但这件事不允许泄露到上层：上层只说 ON/OFF。
  ******************************************************************************
  */
#include "driver_led.h"
#include "stm32f10x.h"

/* ---------- 硬件绑定：换板子只改这三行 ---------- */
#define LED_GPIO_PORT       GPIOC
#define LED_GPIO_PIN        GPIO_Pin_13
#define LED_GPIO_CLK        RCC_APB2Periph_GPIOC

/* LED 状态软件镜像：避免反复读寄存器，也让 Toggle 不依赖读出是否正确 */
static uint8_t s_ledState = LED_OFF;

void DRV_LED_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* 1. 开 APB2 上 GPIOC 的时钟 —— 外设第一步永远是开时钟 */
    RCC_APB2PeriphClockCmd(LED_GPIO_CLK, ENABLE);

    /* 2. 配置 PC13 为低速开漏输出 */
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;     /* PC13 只支持开漏 */
    gpio.GPIO_Pin   = LED_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;      /* LED 不需要翻转速度，2MHz 更省干扰 */
    GPIO_Init(LED_GPIO_PORT, &gpio);

    /* 3. 默认熄灭 */
    s_ledState = LED_OFF;
    GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)s_ledState);
}

void DRV_LED_Set(uint8_t state)
{
    s_ledState = (state == LED_ON) ? LED_ON : LED_OFF;
    GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)s_ledState);
}

void DRV_LED_Toggle(void)
{
    DRV_LED_Set((uint8_t)(s_ledState ^ 0x01u));
}

uint8_t DRV_LED_GetState(void)
{
    return s_ledState;
}
