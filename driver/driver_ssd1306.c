/**
  ******************************************************************************
  * @file    driver_ssd1306.c
  * @brief   SSD1306 OLED 驱动实现（I2C 版）
  *
  *  I2C 上SSD1306 的通信格式：
  *      起始 → 地址+W → 控制字节 → 数据/命令字节... → 停止
  *  控制字节只有两种值：
  *      0x00 = 后面这些字节是"命令"
  *      0x40 = 后面这些字节是"显存数据"
  *  所以 DRV_I2C_WriteReg(addr, 0x00, cmd) 就是发一条命令，
  *       DRV_I2C_WriteMulti(addr, 0x40, buf, len) 就是连续写显存。
  *
  *  为什么不在中断里刷屏：整屏 1024 字节要几毫秒，是阻塞操作，
  *  只能放主循环。这条规矩和 MPU6500 的读取一样。
  ******************************************************************************
  */
#include "driver_ssd1306.h"
#include "driver_i2c.h"
#include "font6x8.h"

/**
 * @brief  上电后的粗略延时
 * @note   和 MPU6500 那份一样是空转，故意不抽公共函数 ——
 *         驱动之间互相依赖会让"单独替换某个器件"变麻烦。
 */
static void SSD1306_DelayMs(uint16_t ms)
{
    volatile uint32_t i;
    while (ms--)
    {
        for (i = 0u; i < 7200u; i++)
        {
            /* 空转 */
        }
    }
}

/* 显存：8 页 x 128 列 = 1024 字节。放在 .bss，不占 Flash */
static uint8_t s_gram[SSD1306_PAGES][SSD1306_WIDTH];

/**
 * @brief  上电初始化命令序列（128x64 通用）
 * @note   顺序照抄数据手册的应用笔记。0xD5/0xA8/0xDA 这类带参数的命令，
 *         参数必须紧跟在命令后面发，不能分开。
 */
static const uint8_t s_initCmd[] =
{
    0xAEu,                      /* 关显示（配置期间必须关，否则会闪） */
    0xD5u, 0x80u,               /* 显示时钟分频/振荡频率 */
    0xA8u, 0x3Fu,               /* 多路复用比 = 64 - 1 */
    0xD3u, 0x00u,               /* 显示偏移 = 0 */
    0x40u,                      /* 显示起始行 = 0（0x40 起算） */
    0x8Du, 0x14u,               /* 使能电荷泵（不写这行屏幕不亮） */
    0x20u, 0x02u,               /* 页地址模式（本项目用的模式） */
    0xA1u,                      /* 段重映射：左右翻转，配合模块朝向 */
    0xC8u,                      /* COM 扫描方向：上下翻转 */
    0xDAu, 0x12u,               /* COM 引脚硬件配置（128x64 必用 0x12） */
    0x81u, 0xCFu,               /* 对比度 */
    0xD9u, 0xF1u,               /* 预充电周期 */
    0xDBu, 0x30u,               /* VCOMH 电平 */
    0xA4u,                      /* 显示内容跟随 RAM（不用全屏点亮） */
    0xA6u,                      /* 正常显示，不反色 */
    0xAFu                       /* 开显示 */
};

uint8_t DRV_SSD1306_Init(void)
{
    uint16_t i;

    /* 上电后等模块内部电荷泵稳定，急着发 I2C 有时不应答 */
    SSD1306_DelayMs(100u);

    /* 先探一次：SSD1306 没有 WHO_AM_I，只能靠"发命令有没有 ACK"判断在不在。
       这一步能把"没接 / 地址错"和"配置错"区分开。 */
    if (DRV_I2C_WriteReg(SSD1306_ADDR, 0x00u, 0xAEu) != 0u)
    {
        return 1u;
    }

    for (i = 0u; i < sizeof(s_initCmd); i++)
    {
        if (DRV_I2C_WriteReg(SSD1306_ADDR, 0x00u, s_initCmd[i]) != 0u)
        {
            return 1u;
        }
    }

    DRV_SSD1306_Clear();
    return DRV_SSD1306_Refresh();
}

void DRV_SSD1306_Clear(void)
{
    uint8_t p, c;

    for (p = 0u; p < SSD1306_PAGES; p++)
    {
        for (c = 0u; c < SSD1306_WIDTH; c++)
        {
            s_gram[p][c] = 0x00u;
        }
    }
}

uint8_t DRV_SSD1306_Refresh(void)
{
    uint8_t p;

    for (p = 0u; p < SSD1306_PAGES; p++)
    {
        /* 每页要先设"从第几页、第几列开始写"，之后连续写 128 个字节 */
        if (DRV_I2C_WriteReg(SSD1306_ADDR, 0x00u, 0xB0u | p) != 0u) { return 1u; }
        if (DRV_I2C_WriteReg(SSD1306_ADDR, 0x00u, 0x00u) != 0u)      { return 2u; }  /* 列地址低 4 位 = 0 */
        if (DRV_I2C_WriteReg(SSD1306_ADDR, 0x00u, 0x10u) != 0u)      { return 3u; }  /* 列地址高 4 位 = 0 */

        /* 整页 128 字节一次发完，只发一次起始条件 */
        if (DRV_I2C_WriteMulti(SSD1306_ADDR, 0x40u, s_gram[p], SSD1306_WIDTH) != 0u) { return 4u; }
    }

    return 0u;
}

void DRV_SSD1306_ShowChar(uint8_t col, uint8_t page, char ch)
{
    uint8_t i;
    const uint8_t *pDot;

    /* 越界直接丢弃，不做处理 —— 驱动层不替上层猜意图 */
    if (page >= SSD1306_PAGES) { return; }

    /* 字库只覆盖 32~126，其余统一画成 '?'，一眼看出数据有问题 */
    if ((uint8_t)ch < 32u || (uint8_t)ch > 126u) { ch = '?'; }

    pDot = Font6x8[(uint8_t)ch - 32u];

    for (i = 0u; i < SSD1306_CHAR_W; i++)
    {
        if (col + i < SSD1306_WIDTH)
        {
            s_gram[page][col + i] = pDot[i];
        }
    }
}

void DRV_SSD1306_ShowString(uint8_t col, uint8_t page, const char *str)
{
    while (*str != '\0')
    {
        /* 剩下的宽度不够画一个字符就停，避免半个字压在边界上 */
        if (col + SSD1306_CHAR_W > SSD1306_WIDTH) { break; }

        DRV_SSD1306_ShowChar(col, page, *str);
        col += SSD1306_CHAR_W;
        str++;
    }
}

void DRV_SSD1306_Invert(uint8_t col, uint8_t page, uint8_t chars)
{
    uint16_t i;
    uint16_t n = (uint16_t)chars * SSD1306_CHAR_W;

    if (page >= SSD1306_PAGES) { return; }

    for (i = 0u; i < n; i++)
    {
        if (col + i < SSD1306_WIDTH)
        {
            s_gram[page][col + i] ^= 0xFFu;
        }
    }
}
