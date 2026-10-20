/**
  ******************************************************************************
  * @file    driver_spi.c
  * @brief   SPI1 主机驱动实现
  *
  *  时钟：SPI1 挂 APB2，PCLK2 = 72 MHz，4 分频 -> SCK = 18 MHz。
  *
  *  模式选择：W25Q64 支持模式 0（CPOL=0 空闲低电平，CPHA=0 第一个边沿采样）
  *            和模式 3。这里用模式 0，也是绝大多数 SPI 器件的默认模式。
  *            CPOL/CPHA 配错的表现是"读出来的全是 0xFF 或 0x00"。
  *
  *  为什么 MISO 也配成复用推挽：主机模式下 MISO 是输入，但引脚要交给 SPI
  *  外设接管，所以配 AF_PP（外设会把它切成输入）。配成普通输入浮空也能工作，
  *  但复用模式才是数据手册推荐的做法。
  ******************************************************************************
  */
#include "driver_spi.h"

/* 等待超时上限。和 I2C 层一个道理：死等会把整个系统拖死 */
#define SPI_TIMEOUT     50000u

void DRV_SPI_Init(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef  spi;

    /* 1. 开时钟：GPIOA 和 SPI1 都在 APB2 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    /* 2. 引脚
          SCK / MOSI 复用推挽；MISO 也用复用推挽（主机模式下由外设接管方向） */
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 3. SPI 外设配置 */
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;  /* 全双工 */
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;      /* 模式 0：空闲时 SCK 为低 */
    spi.SPI_CPHA              = SPI_CPHA_1Edge;    /* 模式 0：第一个边沿采样 */
    spi.SPI_NSS               = SPI_NSS_Soft;      /* 片选用软件 GPIO 控制 */
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  /* 72 / 4 = 18 MHz */
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;  /* 高位先发，和 W25Q64 一致 */
    spi.SPI_CRCPolynomial     = 7u;                /* 硬件 CRC 没开，填默认值 */
    SPI_Init(SPI1, &spi);

    SPI_Cmd(SPI1, ENABLE);
}

uint8_t DRV_SPI_SwapByte(uint8_t txData)
{
    uint32_t timeout = SPI_TIMEOUT;

    /* 等发送缓冲区空。TXE=1 表示上一个字节已经移进移位寄存器，可以写下一个 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (--timeout == 0u) { return 0xFFu; }
    }
    SPI_I2S_SendData(SPI1, txData);

    timeout = SPI_TIMEOUT;
    /* 等接收完成。SPI 是同步的：发一个字节的同时必然收回一个字节，
       所以这里等的是"这次交换结束"，不是"从机主动发来数据" */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (--timeout == 0u) { return 0xFFu; }
    }

    return SPI_I2S_ReceiveData(SPI1);
}
