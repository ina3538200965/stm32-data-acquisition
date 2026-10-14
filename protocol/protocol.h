/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   通用串口帧协议（纯软件层）
  *
  *          这一层刻意做到"零硬件依赖"：不 include 任何 stm32f10x / CMSIS 头文件，
  *          所以它可以直接用 gcc 编译，跑在 PC 上的单元测试里（见 test/host_test.c）。
  *
  *  帧格式：
  *  +--------+--------+-----+-----+-----------+-----+
  *  | SOF0   | SOF1   | LEN | CMD | PAYLOAD   | XOR |
  *  | 0xAA   | 0x55   | 0~N |     | N bytes   | 1B  |
  *  +--------+--------+-----+-----+-----------+-----+
  *
  *  - LEN 只描述 PAYLOAD 长度，不含头尾，最大 PROTO_MAX_PAYLOAD
  *  - XOR = 从 SOF0 到最后一个 PAYLOAD 字节的逐字节异或（含 SOF）
  *  - 解析是逐字节状态机：串口一次来 1 个字节也能处理，不需要缓冲整帧
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
#define PROTO_MAX_PAYLOAD   32u

/* 不含 PAYLOAD 之外的开销：SOF0 SOF1 LEN CMD + XOR = 5 */
#define PROTO_OVERHEAD      5u
#define PROTO_FRAME_MAX_LEN (PROTO_OVERHEAD + PROTO_MAX_PAYLOAD)   /* 37 */

/* ========================= 数据类型 ========================= */

typedef enum
{
    PROTO_OK      = 0,   /* 收满一帧且校验通过 */
    PROTO_BUSY    = 1,   /* 字节已吸收，帧还没收完，继续喂 */
    PROTO_ERR_XOR = 2,   /* 校验失败 */
    PROTO_ERR_LEN = 3    /* LEN 超出 PROTO_MAX_PAYLOAD，已复位等待新帧 */
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
    uint8_t state;                        /* 内部状态机状态 */
    uint8_t len;                          /* 当前帧的 payload 长度 */
    uint8_t cmd;
    uint8_t index;                        /* payload 已收字节数 */
    uint8_t xorBuf;                       /* 累加的校验值 */
    uint8_t payload[PROTO_MAX_PAYLOAD];
} PROTO_Parser_t;

/* ========================= 对外接口 ========================= */

/**
 * @brief  计算一段数据的异或校验
 * @param  buf: 数据首地址
 * @param  len: 字节数
 * @retval 异或结果
 */
uint8_t PROTO_CalcXor(const uint8_t *buf, uint16_t len);

/**
 * @brief  组帧
 * @param  cmd:     命令字
 * @param  payload: 载荷，可为 NULL（当 len == 0 时）
 * @param  len:     载荷长度，必须 <= PROTO_MAX_PAYLOAD
 * @param  out:     输出缓冲区
 * @param  outSize: 输出缓冲区容量
 * @retval 成功返回帧长度；参数非法或缓冲区不足返回 0
 */
uint16_t PROTO_Pack(uint8_t cmd, const uint8_t *payload, uint8_t len,
                    uint8_t *out, uint16_t outSize);

/**
 * @brief  初始化解析器（使用前必须调用一次）
 * @param  parser: 解析器实例
 * @retval None
 */
void PROTO_ParserInit(PROTO_Parser_t *parser);

/**
 * @brief  喂一个字节进状态机
 * @param  parser: 解析器实例
 * @param  byte:   收到的字节
 * @param  frame:  帧输出缓冲；仅在返回 PROTO_OK 时被写入
 * @retval PROTO_Status_t
 */
PROTO_Status_t PROTO_Feed(PROTO_Parser_t *parser, uint8_t byte, PROTO_Frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
