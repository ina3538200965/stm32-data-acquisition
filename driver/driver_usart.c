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
#include <stdio.h>

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
