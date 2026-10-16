/**
  ******************************************************************************
  * @file    driver_adc.c
  * @brief   TIM3 TRGO 触发 ADC1 多通道扫描 + DMA 循环搬运（10 ms/轮）
  *
  *  时钟链路（决定采样周期，改配置前先核对）：
  *    HSE 8M -> PLL x9 -> SYSCLK 72M -> AHB /1 -> HCLK 72M
  *    -> APB1 /2 -> PCLK1 36M，但 TIM2/3/4 时钟 = PCLK1 x2 = 72M
  *    -> 72M / (PSC+1)720 / (ARR+1)1000 = 100 Hz，即每 10 ms 溢出一次
  *
  *  ADC 时钟（决定采样精度）：
  *    PCLK2 72M / 6 = 12 MHz，必须 <= 14 MHz。默认复位值是 /2 = 36M，超频会导致
  *    采样电容来不及充电、转换结果系统性偏小，所以 RCC_ADCCLKConfig 不能省。
  *
  *  为什么 F1 多通道必须用 DMA（硬件限制，不是代码风格问题）：
  *    扫描模式下 EOC 只在【整组转换结束】时置位一次，而不是每通道一次
  *    （F4 才有 EOCS 位可选）。所以用"EOC 中断里读 DR"的方式，每次读到的
  *    都是最后一个通道的值，表现为所有通道读数完全相同。
  *    ADC 每转完一个通道会发一次 DMA 请求，由 DMA 按内存自增顺序自动搬运，
  *    整轮搬完才产生一次中断，CPU 占用随之下降。
  ******************************************************************************
  */
#include "driver_adc.h"

/* ==================== 全局变量（DMA 写 / 主循环读，必须 volatile） ==================== */
volatile uint16_t g_adcBuf[ADC_CH_NUM] = {0};
volatile uint8_t  g_adcConvFlag        = 0;  /* 1 = 一整轮搬完 */
volatile uint32_t g_adcRoundCount      = 0;  /* 累计轮数 */

/**
 * @brief  ADC1 + TIM3 + DMA1_Channel1 初始化
 * @param  None
 * @retval None
 */
void DRV_ADC_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    ADC_InitTypeDef         adc;
    DMA_InitTypeDef         dma;
    NVIC_InitTypeDef        nvic;

    /* ---- 1. 开时钟 ---- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  /* PA0/1/2 采样 + PA8 打点 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);   /* DMA1 挂 AHB */

    /* ADC 时钟分频：72M/6 = 12 MHz（上限 14 MHz） */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* ---- 2. PA0 / PA1 / PA2 设为模拟输入 ---- */
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &gpio);

    /* ---- 3. PA8 推挽输出：每轮搬完翻转一次 ---- */
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
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;  /* 外部触发源 */
    adc.ADC_DataAlign          = ADC_DataAlign_Right;            /* 右对齐 */
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

    /* ---- 7. DMA1_Channel1：ADC1 的固定搭档（硬件映射，不可改通道号） ---- */
    DMA_DeInit(DMA1_Channel1);

    dma.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);   /* 源：ADC 数据寄存器 */
    dma.DMA_MemoryBaseAddr     = (uint32_t)g_adcBuf;  /* 目的：内存数组 */
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;      /* 外设 -> 内存 */
    dma.DMA_BufferSize         = ADC_CH_NUM;                /* 单元个数，不是字节数 */
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;  /* 外设地址固定 */
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;        /* 内存地址递增 */
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;   /* 搬完自动回头覆盖 */
    dma.DMA_Priority           = DMA_Priority_High;   /* 设置优先级 */
    dma.DMA_M2M                = DMA_M2M_Disable;     /* 外设触发，非软件触发 */
    DMA_Init(DMA1_Channel1, &dma);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);    /* 传输完成才中断一次 */
    DMA_Cmd(DMA1_Channel1, ENABLE);

    /* ADC 端也要开：每转完一个通道发 DMA 请求。两端都开才有数据 */
    ADC_DMACmd(ADC1, ENABLE);

    /* ---- 8. NVIC：只保留 DMA TC 这一个中断源 ---- */
    nvic.NVIC_IRQChannel                   = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* ---- 9. 最后启动 TIM3：确保 ADC 与 DMA 已就绪，不会丢掉触发 ---- */
    TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief  DMA1 通道 1 中断：一整轮 3 通道数据搬完
 * @note   与 ADC 的 EOC 不同，DMA 的 TC 标志不会自动清除，必须手动清。
 * @retval None
 */
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);

        GPIOA->ODR ^= GPIO_Pin_8;   /* 打点：一轮翻一次，相邻上升沿间隔 = 20 ms */
        g_adcConvFlag  = 1;         /* 通知主循环 */
        g_adcRoundCount++;          /* D10 与上位机收包数对比查丢帧 */
    }
}
