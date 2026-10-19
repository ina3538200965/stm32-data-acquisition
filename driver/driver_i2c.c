/**
  ******************************************************************************
  * @file    driver_i2c.c
  * @brief   I2C1 主机驱动实现
  *
  *  时钟：I2C1 挂 APB1，PCLK1 = 36 MHz。
  *        400 kHz 快速模式下的 CCR 与 TRISE 由标准库根据 I2C_ClockSpeed 自动算好。
  *
  *  ⚠️ STM32F1 硬件 I2C 的已知问题：
  *    SR2 里的 BUSY 标志在个别时序下会永久置位（尤其是通信中被打断时），
  *    之后所有起始信号都发不出去，现象是"读一次失败后再也读不出数据"。
  *    本驱动的所有等待都带超时计数，超时后软件复位 I2C 外设自行恢复。
  *    这是用硬件 I2C 必须付的代价 —— 换来的是时序由硬件保证、不占 CPU。
  ******************************************************************************
  */
#include "driver_i2c.h"

/* 等待超时上限。72 MHz 下 5 万次循环约 1 ms，足够覆盖 400 kHz 的一个字节传输 */
#define I2C_TIMEOUT     50000u

/**
 * @brief  总线异常后的恢复：软件复位 I2C 外设
 * @note   这不是官方推荐做法，但 F1 的硬件 I2C 卡死时只有这一招最省事
 */
static void I2C_Recover(void)
{
    I2C_Cmd(I2C1, DISABLE);
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
    I2C_Cmd(I2C1, ENABLE);
}

/**
 * @brief  等待总线空闲（BUSY 标志清零）
 */
static uint8_t I2C_WaitIdle(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
    {
        if (--timeout == 0u)
        {
            I2C_Recover();
            return 1u;
        }
    }
    return 0u;
}

/**
 * @brief  等待某个 I2C 事件发生，带超时
 */
static uint8_t I2C_WaitEvent(uint32_t event)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (I2C_CheckEvent(I2C1, event) != SUCCESS)
    {
        if (--timeout == 0u)
        {
            I2C_Recover();
            return 1u;
        }
    }
    return 0u;
}

void DRV_I2C_Init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  i2c;

    /* 1. 开时钟：GPIOB 在 APB2，I2C1 在 APB1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

    /* 2. PB6 / PB7 复用开漏
          必须是 AF_OD：I2C 是"线与"总线，靠上拉电阻把线拉高，
          任何器件拉低就整条线为低。推挽输出会造成两个器件同时驱动而短路。 */
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* 3. I2C 外设配置 */
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;          /* 快速模式下 Tlow/Thigh = 2 */
    i2c.I2C_OwnAddress1         = 0x00u;                    /* 主机模式，自身地址无意义 */
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed          = 400000u;                  /* 400 kHz 快速模式 */
    I2C_Init(I2C1, &i2c);

    I2C_Cmd(I2C1, ENABLE);
}

uint8_t DRV_I2C_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    /* 起始条件 -> EV5 */
    if (I2C_WaitIdle()) { return 1u; }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) { return 2u; }

    /* 发器件地址 + 写方向 -> EV6 */
    I2C_Send7bitAddress(I2C1, devAddr << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) { return 3u; }

    /* 发寄存器地址 -> EV8 */
    I2C_SendData(I2C1, regAddr);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) { return 4u; }

    /* 发数据 -> EV8_2 */
    I2C_SendData(I2C1, data);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) { return 5u; }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0u;
}

uint8_t DRV_I2C_ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *pData)
{
    return DRV_I2C_ReadMulti(devAddr, regAddr, pData, 1u);
}

uint8_t DRV_I2C_ReadMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *pBuf, uint16_t len)
{
    if (I2C_WaitIdle()) { return 1u; }

    /* ---- 第一段：写器件地址 + 寄存器地址，告诉器件"我要从哪读" ---- */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) { return 2u; }

    I2C_Send7bitAddress(I2C1, devAddr << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) { return 3u; }

    I2C_SendData(I2C1, regAddr);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) { return 4u; }

    /* ---- 第二段：重复起始（Restart），切成读方向 ----
           I2C 的读操作必须重新发起始条件，不能直接从写切到读。 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) { return 5u; }

    I2C_Send7bitAddress(I2C1, devAddr << 1, I2C_Direction_Receiver);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) { return 6u; }

    /* ---- 第三段：连续接收 ----
           每收到一个字节主机要回 ACK，最后一个字节回 NACK 并紧接着 STOP，
           通知从机"到此为止"。不这样做从机会继续发，总线状态就乱了。 */
    while (len > 0u)
    {
        if (len == 1u)
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }

        if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            I2C_AcknowledgeConfig(I2C1, ENABLE);   /* 出错也把 ACK 恢复，避免影响下次 */
            return 7u;
        }

        *pBuf = I2C_ReceiveData(I2C1);
        pBuf++;
        len--;
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 0u;
}
