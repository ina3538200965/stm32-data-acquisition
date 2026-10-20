/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   自定义串口帧协议（纯软件层，D6）
  *
  *  这一层刻意做到"零硬件依赖"：不 include 任何 stm32f10x / CMSIS 头文件，
  *  所以它能直接用 gcc 编译，跑在 PC 上的单元测试里（见 test/host_test.c）。
  *  这是"协议可以脱离硬件验证"的前提，也是面试时能拿来讲的工程点。
  *
  *  ============================ 帧格式 ============================
  *
  *  +-------+-------+-----+-----+---------+--------+-------+-----+
  *  | SOF0  | SOF1  | LEN | CMD | PAYLOAD | CRC16  | EOF   |     |
  *  | 0xAA  | 0x55  | 0~N |     | N bytes | 2B 小端| 0x0D  |     |
  *  +-------+-------+-----+-----+---------+--------+-------+-----+
  *     1B      1B     1B    1B     N B       2B       1B
  *
  *  为什么每个字段这么定：
  *
  *  - **帧头 2 字节**：单字节帧头在数据里出现的概率太高（1/256），
  *    两个特定字节连在一起的概率降到 1/65536，误同步概率大幅下降。
  *    选 0xAA(10101010) 和 0x55(01010101) 是因为它们是交替位，
  *    在物理层上最容易识别，也是业界惯例。
  *
  *  - **LEN 只描述 PAYLOAD**：接收方拿到 LEN 就知道还要收多少字节，
  *    **不依赖帧尾来定位帧边界**。这样即使 PAYLOAD 里出现了 0xAA 0x55，
  *    也不会被误判成新帧头 —— 因为我们是按长度跳过的。
  *
  *  - **CRC16 用 Modbus 多项式**：不是照抄 Modbus 协议，只是复用它的 CRC 算法
  *    （多项式 0xA001，初值 0xFFFF）。选它是因为有公开的测试向量可以验证正确性，
  *    面试被问"你怎么确认 CRC 算对了"时能答得上来。
  *
  *  - **帧尾 0x0D**：相当于一个额外的"闸门"。CRC 通过了但帧尾不对，
  *    说明同步位置还是错的，多一道防线。CRC 漏检率约 1/65536，
  *    加上帧尾后误帧率再降一个数量级。
  *
  *  ============================ 为什么不用转义 ============================
  *
  *  另一派做法是"转义"（数据里出现帧头就插一个转义字节）。没选它是因为：
  *    1. 转义会让帧长度可变，接收方无法预知要收多少，环形缓冲管理变复杂
  *    2. 转义要在发送端逐字节扫描，多一次遍历
  *    3. 有 LEN 字段已经能保证按长度跳过，不需要转义
  *  代价是：一旦帧头字节被破坏，会丢一帧（靠 CRC + 帧尾兜住，不会解错）
  *
  *  ============================ 解析方式 ============================
  *
  *  逐字节状态机（PROTO_Feed）：串口一次来 1 个字节也能处理，
  *  不需要缓冲整帧再解析。这是 D7 做 DMA + 环形缓冲的基础。
  ******************************************************************************
  */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/* ========================= 协议常量 ========================= */

#define PROTO_SOF0          0xAAu
#define PROTO_SOF1          0x55u
#define PROTO_EOF           0x0Du
#define PROTO_MAX_PAYLOAD   32u

/* 除 PAYLOAD 外的固定开销：SOF0 SOF1 LEN CMD + CRC16(2) + EOF = 7 */
#define PROTO_OVERHEAD      7u
#define PROTO_FRAME_MAX_LEN (PROTO_OVERHEAD + PROTO_MAX_PAYLOAD)   /* 39 */

/* ========================= 命令字 ========================= */
/* 0x01~0x0F：下位机主动上报 */
#define CMD_ADC_DATA        0x01u   /* 三路 ADC 采样值 */
#define CMD_MPU_DATA        0x02u   /* 六轴 + 温度 */
#define CMD_STATUS          0x03u   /* 心跳 / 系统状态 */

/* 0x10~0x1F：上位机下发命令（D7 之后启用） */
#define CMD_SET_RATE        0x10u   /* 设置采样周期 */
#define CMD_GET_INFO        0x11u   /* 查询版本信息 */
#define CMD_ACK             0x7Fu   /* 通用应答 */

/* ========================= 数据类型 ========================= */

typedef enum
{
    PROTO_OK       = 0,   /* 收满一帧，CRC 与帧尾都对 */
    PROTO_BUSY     = 1,   /* 字节已吸收，帧还没收完，继续喂 */
    PROTO_ERR_CRC  = 2,   /* CRC 校验失败 */
    PROTO_ERR_LEN  = 3,   /* LEN 超出 PROTO_MAX_PAYLOAD，已复位等新帧 */
    PROTO_ERR_EOF  = 4    /* CRC 对了但帧尾不对，说明同步位置是错的 */
} PROTO_Status_t;

/* 解析完成后输出的帧 */
typedef struct
{
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} PROTO_Frame_t;

/* 解析器实例：支持多路串口各自持有一个，互不干扰（无全局静态变量） */
typedef struct
{
    uint8_t  state;                       /* 内部状态机状态 */
    uint8_t  len;                         /* 当前帧的 payload 长度 */
    uint8_t  cmd;
    uint8_t  index;                       /* payload 已收字节数 */
    uint16_t crcCalc;                     /* 边收边算的 CRC（收到的字节） */
    uint8_t  crcLo;                       /* 帧里带来的 CRC 低字节 */
    uint8_t  payload[PROTO_MAX_PAYLOAD];
} PROTO_Parser_t;

/* ========================= 对外接口 ========================= */

/**
 * @brief  CRC16（Modbus：多项式 0xA001，初值 0xFFFF，低位在前）
 * @param  buf  数据首地址
 * @param  len  字节数
 * @retval CRC 值
 * @note   标准测试向量：对字符串 "123456789" 的结果是 0x4B37。
 *         单测里就是用这条来验证实现没写错的 ——
 *         自己写的 CRC 如果不跟公开向量对一遍，等于没验证。
 */
uint16_t PROTO_CRC16(const uint8_t *buf, uint16_t len);

/**
 * @brief  组帧：填好帧头/长度/命令/数据/CRC/帧尾
 * @param  cmd      命令字
 * @param  payload  载荷，len==0 时可为 NULL
 * @param  len      载荷长度，必须 <= PROTO_MAX_PAYLOAD
 * @param  out      输出缓冲区
 * @param  outSize  输出缓冲区容量
 * @retval 成功返回帧长度（字节数）；参数非法或缓冲区不足返回 0
 */
uint16_t PROTO_Pack(uint8_t cmd, const uint8_t *payload, uint8_t len,
                    uint8_t *out, uint16_t outSize);

/**
 * @brief  初始化解析器（使用前必须调用一次）
 */
void PROTO_ParserInit(PROTO_Parser_t *parser);

/**
 * @brief  喂一个字节进状态机
 * @param  parser  解析器实例
 * @param  byte    收到的字节
 * @param  frame   帧输出缓冲；仅在返回 PROTO_OK 时被写入
 * @retval PROTO_Status_t
 * @note   调用方只要看返回值：
 *           OK    -> frame 里有完整一帧，拿去处理
 *           BUSY  -> 什么都不用做，继续喂下一个字节
 *           其他  -> 这一帧废了，协议层已自动复位，继续喂即可
 *         这样串口中断里不用写任何分支逻辑。
 */
PROTO_Status_t PROTO_Feed(PROTO_Parser_t *parser, uint8_t byte, PROTO_Frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
