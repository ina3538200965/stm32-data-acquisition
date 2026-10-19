/**
  ******************************************************************************
  * @file    driver_mpu6500.h
  * @brief   MPU6500 六轴传感器（加速度 + 陀螺仪 + 片内温度）驱动
  *
  *  ⚠️ WHO_AM_I 的取值（三种芯片都兼容本驱动，但 ID 不同）：
  *       MPU6050 = 0x68    江科大例程按这个判断
  *       MPU6500 = 0x70
  *       MPU9250 = 0x71    （9250 = 6500 + 内部 AK8963 磁力计）
  *     三者的数据寄存器地址（0x3B 起）和 I2C 地址（0x68/0x69）完全兼容，
  *     只有 ID 不同。所以判断时要把三个值都接受，再把实际读到的打印出来。
  ******************************************************************************
  */
#ifndef __DRIVER_MPU6500_H
#define __DRIVER_MPU6500_H

#include "stm32f10x.h"

/* 7 位 I2C 地址：AD0 接地为 0x68，接 VCC 为 0x69（江科大模块通常接地） */
#define MPU6500_ADDR        0x68u

/* WHO_AM_I 的可能取值，三者的数据与地址布局兼容 */
#define MPU6500_ID          0x70u
#define MPU9250_ID          0x71u
#define MPU6050_ID          0x68u

/* ---- 寄存器地址 ---- */
#define MPU6500_SMPLRT_DIV  0x19u
#define MPU6500_CONFIG      0x1Au
#define MPU6500_GYRO_CFG    0x1Bu
#define MPU6500_ACCEL_CFG   0x1Cu
#define MPU6500_ACCEL_OUT   0x3Bu   /* 从此处起连续 14 字节：加速度6 + 温度2 + 陀螺仪6 */
#define MPU6500_PWR_MGMT_1  0x6Bu
#define MPU6500_PWR_MGMT_2  0x6Cu
#define MPU6500_WHO_AM_I    0x75u

/* ---- 量程与灵敏度（当前配置：加速度 ±2g，陀螺仪 ±250°/s） ---- */
#define MPU6500_ACCEL_SENS  16384.0f    /* LSB / g   */
#define MPU6500_GYRO_SENS   131.0f      /* LSB / °/s */

/**
 * @brief  一次读回的原始数据
 */
typedef struct
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t tempRaw;      /* 温度原始值，需换算成摄氏度 */
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} MPU6500_Data_t;

/**
 * @brief  初始化 MPU6500：复位、唤醒、配采样率与量程
 * @param  None
 * @retval 0 = 成功
 *         1 = I2C 通信失败
 *         2 = WHO_AM_I 不是 0x70（器件不匹配或没接好）
 */
uint8_t DRV_MPU6500_Init(void);

/**
 * @brief  陀螺仪零偏校准
 * @param  None
 * @retval 0 = 成功，非 0 = I2C 通信失败
 * @note   【调用前提：模块必须静止不动】
 *         MEMS 陀螺仪静止时输出不为 0（出厂零偏可达 ±5 °/s），
 *         直接积分成角度会以每秒几度的速度漂移。
 *         做法：静止时连采 N 次求平均，把这个平均值当作零偏存下来，
 *         之后每次读数都减掉它。这是传感器驱动里必做的一步。
 */
uint8_t DRV_MPU6500_Calibrate(void);

/**
 * @brief  取当前使用的陀螺仪零偏（校准后才有意义）
 */
void DRV_MPU6500_GetBias(int16_t *pBx, int16_t *pBy, int16_t *pBz);

/**
 * @brief  取实际读到的 WHO_AM_I
 * @param  None
 * @retval 0x70 = 6500，0x71 = 9250，0x68 = 6050，0 = 还没初始化成功
 * @note   用这个判断模块上焊的到底是哪颗芯片
 */
uint8_t DRV_MPU6500_GetID(void);

/**
 * @brief  读一次六轴 + 温度原始数据
 * @param  pData  数据存放处
 * @retval 0 = 成功，非 0 = I2C 通信失败
 */
uint8_t DRV_MPU6500_Read(MPU6500_Data_t *pData);

/**
 * @brief  温度原始值换算成摄氏度
 * @param  raw  原始值
 * @retval 摄氏度（整数）
 * @note   MPU6500：T = raw / 333.87 + 21
 *         （MPU6050 是 raw / 340 + 36.53，两者不一样，别混用）
 */
int16_t DRV_MPU6500_RawToCelsius(int16_t raw);

#endif /* __DRIVER_MPU6500_H */
