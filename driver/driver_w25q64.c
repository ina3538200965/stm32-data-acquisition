/**
  ******************************************************************************
  * @file    driver_w25q64.c
  * @brief   W25Q64 器件层实现
  *
  *  每条指令的帧格式都是：拉低 CS → 发指令码 → 发地址/数据 → 拉高 CS。
  *  **CS 必须包住整条指令**，两条指令之间 CS 要拉高，否则芯片不认。
  *
  *  地址是 24 位，按"高字节在前"发。
  ******************************************************************************
  */
#include "driver_w25q64.h"
#include "driver_spi.h"

/* ---- 指令表 ---- */
#define CMD_WRITE_ENABLE    0x06u
#define CMD_READ_STATUS1    0x05u
#define CMD_READ_DATA       0x03u
#define CMD_PAGE_PROGRAM    0x02u
#define CMD_SECTOR_ERASE    0x20u
#define CMD_JEDEC_ID        0x9Fu

/* 状态寄存器 1 的位 */
#define STATUS_BUSY         0x01u

/* 片选引脚：PA4，软件控制 */
#define W25Q64_CS_PORT      GPIOA
#define W25Q64_CS_PIN       GPIO_Pin_4

/* 等待芯片就绪的超时。擦除一个 sector 最长可能到 400 ms，
   循环次数要留够：18 MHz 下每次交换约 8 位 / 18 MHz ≈ 0.5 µs，
   300 万次 ≈ 1.5 秒，足够覆盖最坏情况 */
#define W25Q64_BUSY_TIMEOUT 3000000u

static void W25Q64_CS_Low(void)
{
    GPIO_ResetBits(W25Q64_CS_PORT, W25Q64_CS_PIN);
}

static void W25Q64_CS_High(void)
{
    GPIO_SetBits(W25Q64_CS_PORT, W25Q64_CS_PIN);
}

/**
 * @brief  写使能
 * @note   这个"使能"只对紧跟着的下一条写/擦指令生效，之后芯片自动清零。
 *         所以每次 Page Program 和 Sector Erase 之前都要重新发一遍。
 */
static void W25Q64_WriteEnable(void)
{
    W25Q64_CS_Low();
    DRV_SPI_SwapByte(CMD_WRITE_ENABLE);
    W25Q64_CS_High();
}

uint8_t DRV_W25Q64_WaitBusy(void)
{
    uint32_t timeout = W25Q64_BUSY_TIMEOUT;
    uint8_t  status;

    W25Q64_CS_Low();
    DRV_SPI_SwapByte(CMD_READ_STATUS1);

    /* 关键：读状态寄存器可以连续读，CS 保持低电平，
       芯片会一直把当前状态送出来，不用重复发指令码 */
    do
    {
        status = DRV_SPI_SwapByte(0xFFu);
        if (--timeout == 0u)
        {
            W25Q64_CS_High();
            return 1u;
        }
    } while ((status & STATUS_BUSY) != 0u);

    W25Q64_CS_High();
    return 0u;
}

uint8_t DRV_W25Q64_ReadID(uint8_t *pManuf, uint16_t *pDevID)
{
    uint8_t  m;
    uint16_t d;

    if (DRV_W25Q64_WaitBusy() != 0u) { return 1u; }

    W25Q64_CS_Low();
    DRV_SPI_SwapByte(CMD_JEDEC_ID);
    m = DRV_SPI_SwapByte(0xFFu);                 /* 厂商 ID */
    d  = (uint16_t)DRV_SPI_SwapByte(0xFFu) << 8; /* 器件 ID 高字节 */
    d |= (uint16_t)DRV_SPI_SwapByte(0xFFu);      /* 器件 ID 低字节 */
    W25Q64_CS_High();

    *pManuf = m;
    *pDevID = d;
    return 0u;
}

uint8_t DRV_W25Q64_Init(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t  manuf;
    uint16_t devID;

    /* 1. SPI 总线 */
    DRV_SPI_Init();

    /* 2. CS 引脚：普通推挽输出，软件控制。
          不用硬件 NSS 是因为硬件 NSS 只能管一个从机 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = W25Q64_CS_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    W25Q64_CS_High();                            /* 空闲时 CS 必须为高 */

    /* 3. 读 ID 确认在线。读不到通常是 SCK/MOSI/MISO 接错或模块没供电 */
    if (DRV_W25Q64_ReadID(&manuf, &devID) != 0u) { return 1u; }
    if (manuf != W25Q64_MANUF_ID) { return 2u; }

    return 0u;
}

uint8_t DRV_W25Q64_Read(uint32_t addr, uint8_t *pBuf, uint16_t len)
{
    uint16_t i;

    if (DRV_W25Q64_WaitBusy() != 0u) { return 1u; }

    W25Q64_CS_Low();
    DRV_SPI_SwapByte(CMD_READ_DATA);
    DRV_SPI_SwapByte((uint8_t)((addr >> 16) & 0xFFu));
    DRV_SPI_SwapByte((uint8_t)((addr >>  8) & 0xFFu));
    DRV_SPI_SwapByte((uint8_t)( addr        & 0xFFu));

    /* 读没有页限制，可以一路往下读，芯片内部地址自动递增 */
    for (i = 0u; i < len; i++)
    {
        pBuf[i] = DRV_SPI_SwapByte(0xFFu);
    }
    W25Q64_CS_High();

    return 0u;
}

uint8_t DRV_W25Q64_PageProgram(uint32_t addr, const uint8_t *pBuf, uint16_t len)
{
    uint16_t i;

    if (len == 0u || len > W25Q64_PAGE_SIZE)                 { return 1u; }
    /* 不能跨页：一页 256 字节对齐，跨了后面的字节会绕回本页开头覆盖前面写的 */
    if (((addr & 0xFFu) + len) > W25Q64_PAGE_SIZE)           { return 2u; }

    if (DRV_W25Q64_WaitBusy() != 0u) { return 3u; }

    W25Q64_WriteEnable();

    W25Q64_CS_Low();
    DRV_SPI_SwapByte(CMD_PAGE_PROGRAM);
    DRV_SPI_SwapByte((uint8_t)((addr >> 16) & 0xFFu));
    DRV_SPI_SwapByte((uint8_t)((addr >>  8) & 0xFFu));
    DRV_SPI_SwapByte((uint8_t)( addr        & 0xFFu));

    for (i = 0u; i < len; i++)
    {
        DRV_SPI_SwapByte(pBuf[i]);
    }
    W25Q64_CS_High();

    /* 写也是毫秒级，等它做完再返回 */
    return DRV_W25Q64_WaitBusy();
}

uint8_t DRV_W25Q64_Write(uint32_t addr, const uint8_t *pBuf, uint16_t len)
{
    uint16_t first;
    uint8_t  ret;

    while (len > 0u)
    {
        /* 本页还剩多少空间 */
        first = W25Q64_PAGE_SIZE - (uint16_t)(addr & 0xFFu);
        if (first > len) { first = len; }

        ret = DRV_W25Q64_PageProgram(addr, pBuf, first);
        if (ret != 0u) { return ret; }

        addr += first;
        pBuf += first;
        len  -= first;
    }

    return 0u;
}

uint8_t DRV_W25Q64_SectorErase(uint32_t addr)
{
    if (DRV_W25Q64_WaitBusy() != 0u) { return 1u; }

    W25Q64_WriteEnable();

    W25Q64_CS_Low();
    DRV_SPI_SwapByte(CMD_SECTOR_ERASE);
    /* 擦除按 sector 对齐，传 sector 内任意地址都擦同一块 */
    DRV_SPI_SwapByte((uint8_t)((addr >> 16) & 0xFFu));
    DRV_SPI_SwapByte((uint8_t)((addr >>  8) & 0xFFu));
    DRV_SPI_SwapByte((uint8_t)( addr        & 0xFFu));
    W25Q64_CS_High();

    return DRV_W25Q64_WaitBusy();
}
