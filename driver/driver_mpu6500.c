/**
  ******************************************************************************
  * @file    driver_mpu6500.c
  * @brief   MPU6500 驱动实现
  *
  *  初始化流程（顺序不能乱）：
  *    1. 读 WHO_AM_I 确认器件在线（0x70）
  *    2. PWR_MGMT_1 写 0x80 复位，等 100 ms
  *    3. PWR_MGMT_1 写 0x01 唤醒并选 PLL 时钟源（用内部 8 MHz 会有温漂）
  *    4. 配采样率、低通滤波、量程
  *    5. PWR_MGMT_2 全部清零，六轴都打开
  *
  *  采样率怎么定：
  *    陀螺仪输出率 = 1 kHz（开了 DLPF 之后）
  *    实际采样率 = 1 kHz / (SMPLRT_DIV + 1)
  *    这里 SMPLRT_DIV = 9 -> 100 Hz，正好和 ADC 的 10 ms 一轮对齐。
  ******************************************************************************
  */
#include "driver_mpu6500.h"
#include "driver_i2c.h"

/**
 * @brief  粗略毫秒延时
 * @note   72 MHz 下内层每次循环约 4~5 个周期，7200 次约 1 ms。
 *         这里只用在初始化期间的复位等待，精度要求不高。
 */
static void MPU6500_DelayMs(uint16_t ms)
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

/* 实际读到的 WHO_AM_I。模块上丝印常写"MPU-9250/6500"，两种芯片都可能焊，
   所以判断时都接受，再把真实值存下来供上层打印。 */
static uint8_t s_deviceID = 0u;

/* 陀螺仪零偏。s_calibrated 为 0 时 Read 不减 bias，
   所以校准函数里读到的是原始值，不会自己减自己。 */
static int16_t s_gyroBiasX = 0;
static int16_t s_gyroBiasY = 0;
static int16_t s_gyroBiasZ = 0;
static uint8_t s_calibrated = 0u;

/* 校准采样数。传感器输出率 100 Hz，每次间隔 11 ms，50 次约 0.55 秒 */
#define MPU6500_CAL_SAMPLES     50u

uint8_t DRV_MPU6500_Calibrate(void)
{
    MPU6500_Data_t d;
    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;
    uint16_t i;

    for (i = 0u; i < MPU6500_CAL_SAMPLES; i++)
    {
        if (DRV_MPU6500_Read(&d) != 0u)
        {
            return 1u;
        }
        sumX += d.gyroX;
        sumY += d.gyroY;
        sumZ += d.gyroZ;

        /* 间隔必须大于 10 ms，否则读到的是同一帧数据，平均出来没有意义 */
        MPU6500_DelayMs(11u);
    }

    s_gyroBiasX = (int16_t)(sumX / (int32_t)MPU6500_CAL_SAMPLES);
    s_gyroBiasY = (int16_t)(sumY / (int32_t)MPU6500_CAL_SAMPLES);
    s_gyroBiasZ = (int16_t)(sumZ / (int32_t)MPU6500_CAL_SAMPLES);
    s_calibrated = 1u;

    return 0u;
}

void DRV_MPU6500_GetBias(int16_t *pBx, int16_t *pBy, int16_t *pBz)
{
    *pBx = s_gyroBiasX;
    *pBy = s_gyroBiasY;
    *pBz = s_gyroBiasZ;
}

uint8_t DRV_MPU6500_Init(void)
{
    uint8_t id = 0u;

    /* 1. 先确认器件在线。这一步能区分"没接好"和"配置错了" */
    if (DRV_I2C_ReadReg(MPU6500_ADDR, MPU6500_WHO_AM_I, &id) != 0u)
    {
        return 1u;
    }
    if ((id != MPU6500_ID) && (id != MPU9250_ID) && (id != MPU6050_ID))
    {
        return 2u;
    }
    s_deviceID = id;

    /* 2. 复位。复位后所有寄存器回到默认值，必须等器件自己准备好 */
    if (DRV_I2C_WriteReg(MPU6500_ADDR, MPU6500_PWR_MGMT_1, 0x80u) != 0u) { return 1u; }
    MPU6500_DelayMs(100u);

    /* 3. 唤醒 + 选时钟源为陀螺仪 PLL（0x01） */
    if (DRV_I2C_WriteReg(MPU6500_ADDR, MPU6500_PWR_MGMT_1, 0x01u) != 0u) { return 1u; }

    /* 4. CONFIG = 0x01：开 DLPF，陀螺仪输出率 1 kHz */
    if (DRV_I2C_WriteReg(MPU6500_ADDR, MPU6500_CONFIG, 0x01u) != 0u) { return 1u; }

    /* 5. SMPLRT_DIV = 9：1000 / (9+1) = 100 Hz */
    if (DRV_I2C_WriteReg(MPU6500_ADDR, MPU6500_SMPLRT_DIV, 0x09u) != 0u) { return 1u; }

    /* 6. 量程：陀螺仪 ±250 °/s，加速度 ±2 g */
    if (DRV_I2C_WriteReg(MPU6500_ADDR, MPU6500_GYRO_CFG,  0x00u) != 0u) { return 1u; }
    if (DRV_I2C_WriteReg(MPU6500_ADDR, MPU6500_ACCEL_CFG, 0x00u) != 0u) { return 1u; }

    /* 7. PWR_MGMT_2 = 0：六轴全部使能（默认是 0，显式写一遍防止残留配置） */
    if (DRV_I2C_WriteReg(MPU6500_ADDR, MPU6500_PWR_MGMT_2, 0x00u) != 0u) { return 1u; }

    /* 8. 等输出稳定后做零偏校准。
          ⚠️ 这一步要求模块静止 —— 上电时如果正在晃动模块，校准出的零偏是错的。 */
    MPU6500_DelayMs(50u);
    if (DRV_MPU6500_Calibrate() != 0u) { return 3u; }

    return 0u;
}

uint8_t DRV_MPU6500_GetID(void)
{
    return s_deviceID;
}

uint8_t DRV_MPU6500_Read(MPU6500_Data_t *pData)
{
    uint8_t buf[14];

    /* 从 0x3B 连续读 14 字节：MPU6500 的地址指针会自动递增，
       所以一次读比分六次读快得多，也避免了"高低字节分两次读导致数据错配" */
    if (DRV_I2C_ReadMulti(MPU6500_ADDR, MPU6500_ACCEL_OUT, buf, 14u) != 0u)
    {
        return 1u;
    }

    /* 每个轴是 16 位补码，高字节在前（大端） */
    pData->accelX = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    pData->accelY = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    pData->accelZ = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    pData->tempRaw = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    pData->gyroX  = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    pData->gyroY  = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    pData->gyroZ  = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    /* 扣掉零偏。校准前 s_calibrated = 0，这里不生效 */
    if (s_calibrated)
    {
        pData->gyroX -= s_gyroBiasX;
        pData->gyroY -= s_gyroBiasY;
        pData->gyroZ -= s_gyroBiasZ;
    }

    return 0u;
}

int16_t DRV_MPU6500_RawToCelsius(int16_t raw)
{
    /* T = raw / 333.87 + 21，用整数除法取近似，误差不到 0.1 °C */
    return (int16_t)(raw / 334 + 21);
}
