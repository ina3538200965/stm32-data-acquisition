/**
  ******************************************************************************
  * @file    driver_ssd1306.h
  * @brief   0.96 寸 OLED（SSD1306，128x64，4 脚 I2C）驱动
  *
  *  与 MPU6500 挂在同一条 I2C1 总线上（PB6/PB7），直接复用 driver_i2c，
  *  本文件里没有任何 I2C 时序代码 —— 这就是总线层/器件层分开的好处。
  *
  *  ⚠️ I2C 地址：多数模块是 0x3C，少数是 0x3D（由模块背面电阻决定）。
  *     如果 Init 返回 1（通信失败），把 SSD1306_ADDR 改成 0x3D 试一次。
  *
  *  显存模型（页地址模式）：
  *     屏幕 64 行分成 8 页，每页 8 行；每页 128 列。
  *     显存里一个字节 = 一列上的 8 个像素，bit0 是这页最上面那一行。
  *     所以 s_gram[page][col] 的 bit0 对应屏幕第 (page*8+0) 行。
  ******************************************************************************
  */
#ifndef __DRIVER_SSD1306_H
#define __DRIVER_SSD1306_H

#include "stm32f10x.h"

/* 7 位 I2C 地址 */
#define SSD1306_ADDR        0x3Cu

#define SSD1306_WIDTH       128u
#define SSD1306_HEIGHT      64u
#define SSD1306_PAGES       8u          /* 64 / 8 */
#define SSD1306_CHAR_W      6u          /* 字体宽（含 1 列间隔） */

/**
 * @brief  初始化 OLED：发初始化命令序列并清屏
 * @param  None
 * @retval 0 = 成功
 *         1 = I2C 通信失败（没接 / 地址不对）
 */
uint8_t DRV_SSD1306_Init(void);

/**
 * @brief  清空显存（不影响屏幕，要配合 Refresh 才生效）
 */
void DRV_SSD1306_Clear(void);

/**
 * @brief  把显存整屏刷到 OLED
 * @param  None
 * @retval 0 = 成功，非 0 = 失败
 * @note   1024 字节分 8 次 WriteMulti，400 kHz 下约 3 ms。
 *         只在数据变化后调用，不要每轮都刷。
 */
uint8_t DRV_SSD1306_Refresh(void);

/**
 * @brief  在指定位置画一个字符
 * @param  col   像素列，0 ~ 127
 * @param  page  页，0 ~ 7（page 0 是屏幕最上面 8 行）
 * @param  ch    要显示的字符，ASCII 32~126，其余画成 '?'
 */
void DRV_SSD1306_ShowChar(uint8_t col, uint8_t page, char ch);

/**
 * @brief  在指定位置画字符串，超出右边界自动截断
 * @param  col   起始像素列
 * @param  page  页
 * @param  str   以 '\0' 结尾的字符串
 */
void DRV_SSD1306_ShowString(uint8_t col, uint8_t page, const char *str);

/**
 * @brief  反显一段区域（黑底白字 / 白底黑字切换），用于做高亮选中效果
 * @param  col   起始列
 * @param  page  页
 * @param  chars 要反显的字符个数
 */
void DRV_SSD1306_Invert(uint8_t col, uint8_t page, uint8_t chars);

#endif /* __DRIVER_SSD1306_H */
