/**
  ******************************************************************************
  * @file    driver_adc.c
  * @brief   TIM3 TRGO 触发 ADC1 多通道扫描（10 ms/轮）
  *
  *  时钟链路（决定采样周期，改配置前先核对）：
  *    HSE 8M -> PLL x9 -> SYSCLK 72M -> AHB /1 -> HCLK 72M
  *    -> APB1 /2 -> PCLK1 36M，但 TIM2/3/4 时钟 = PCLK1 x2 = 72M
  *    -> 72M / (PSC+1)720 / (ARR+1)1000 = 100 Hz，即每 10 ms 溢出一次
  *
  *  ADC 时钟（决定采样精度）：
  *    PCLK2 72M / 6 = 12 MHz，必须 <= 14 MHz。默认复位值是 /2 = 36M，超频会导致
  *    采样电容来不及充电、转换结果系统性偏小，所以 RCC_ADCCLKConfig 不能省。
  ******************************************************************************
  */
#include "driver_adc.h"

/* ==================== 全局变量（中断写 / 主循环读，必须 volatile） ==================== */
volatile uint16_t g_adcBuf[ADC_CH_NUM] = {0};
volatile uint8_t  g_adcConvFlag        = 0;  //至1 = 一轮完成
volatile uint32_t g_adcRoundCount      = 0;  //循环轮数

/* 当前正在收的通道。*/
static volatile uint8_t s_chIndex = 0;

/**
 * @brief  ADC1 + TIM3 初始化
 * @param  None
 * @retval None
 */
void DRV_ADC_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    ADC_InitTypeDef         adc;
    NVIC_InitTypeDef        nvic;

    /* ---- 1. 开时钟 ---- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  /* PA0/1/2 采样 + PA8 打点 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);

    /* ADC 时钟分频：72M/6 = 12 MHz（上限 14 MHz） */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* ---- 2. PA0 / PA1 / PA2 设为模拟输入 ---- */
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &gpio);

    /* ---- 3. PA8 推挽输出：每轮采样开始时翻转一次---- */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* ---- 4. TIM3 时基：10 ms 溢出一次 ---- */
    tim.TIM_Prescaler     = 719;                 /* 720 - 1 */
    tim.TIM_Period        = 999;                 /* 1000 - 1 */
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);

    /* 更新事件(溢出)从 TRGO 输出脉冲，内部直连 ADC */
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

    /* ---- 5. ADC1 配置 ---- */
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = ENABLE;                          /* 多通道扫描 */
    adc.ADC_ContinuousConvMode = DISABLE;                         /* 关连续：由 TRGO 触发 */
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;    /* 外部触发源 */
    adc.ADC_DataAlign          = ADC_DataAlign_Right;             /* 右对齐 */
    adc.ADC_NbrOfChannel       = ADC_CH_NUM;
    ADC_Init(ADC1, &adc);

    /* 规则组顺序与采样时间
       239.5 + 12.5 = 252 个 ADCCLK 周期 = 21 us/通道，相对 10 ms 周期可以忽略。
       电位器/光敏这类内阻较大的信号源，采样时间宁长勿短。 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);

    ADC_ExternalTrigConvCmd(ADC1, ENABLE);   /* 允许外部触发启动转换 */
    ADC_Cmd(ADC1, ENABLE);                   /* 使能 ADC1 */

    /* ---- 6. 校准（使能 ADC 之后做，否则校准不生效） ---- */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    /* ---- 7. EOC 中断 + NVIC ---- */
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    nvic.NVIC_IRQChannel                   = ADC1_2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* ---- 8. 最后启动 TIM3：确保 ADC 已就绪，不会丢掉触发 ---- */
    TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief  ADC1/2 中断服务函数
 * @note   扫描模式下【每转换完一个通道】进一次中断，所以一次中断只能读一个值。
 * @retval None
 */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        /* 每一轮的第一个通道：翻转 PA8 打点 */
        if (s_chIndex == 0)
        {
            GPIOA->ODR ^= GPIO_Pin_8;
        }

        /* 读 DR 的同时自动清除 EOC 标志 */
        g_adcBuf[s_chIndex] = (uint16_t)ADC_GetConversionValue(ADC1);

        s_chIndex++;
        if (s_chIndex >= ADC_CH_NUM)
        {
            s_chIndex = 0;
            g_adcConvFlag = 1;     /* 一整轮完成，通知主循环 */
            g_adcRoundCount++;     /* 轮数 +1，D10 用来和上位机计数对比查丢帧 */
        }
    }
}
