/**
  ******************************************************************************
  * @file    ringbuf.c
  * @brief   环形缓冲区实现（零硬件依赖，详见 ringbuf.h 顶部的设计说明）
  ******************************************************************************
  */
#include "ringbuf.h"

/**
 * @brief  判断一个 16 位数是不是 2 的幂
 * @note   2 的幂只有一位是 1，所以 n & (n-1) 必然为 0。
 *         0 会被误判成 2 的幂，所以单独排除。
 */
static uint8_t RB_IsPow2(uint16_t n)
{
    return (n != 0u) && ((n & (uint16_t)(n - 1u)) == 0u) ? 1u : 0u;
}

uint8_t RB_Init(RB_t *p, uint8_t *buf, uint16_t sizePow2)
{
    if ((p == 0) || (buf == 0) || (RB_IsPow2(sizePow2) == 0u))
    {
        /* 参数非法：置成不可用状态，后续所有操作都安全返回 0 */
        if (p != 0)
        {
            p->pBuf     = 0;
            p->mask     = 0u;
            p->head     = 0u;
            p->tail     = 0u;
            p->overflow = 0u;
        }
        return 1u;
    }

    p->pBuf     = buf;
    p->mask     = (uint16_t)(sizePow2 - 1u);
    p->head     = 0u;
    p->tail     = 0u;
    p->overflow = 0u;
    return 0u;
}

uint16_t RB_Used(RB_t *p)
{
    /* pBuf 为 0 表示没初始化成功（见 RB_Init），此时一切都不该可用 */
    if (p->pBuf == 0)
    {
        return 0u;
    }

    /* 无符号相减：即使 head 已经绕回越过 tail 也照样算对 */
    return (uint16_t)(p->head - p->tail);
}

uint16_t RB_Free(RB_t *p)
{
    /* 没初始化成功就别用 mask+1 去推算容量了 ——
       mask 是 0 时会被算成 size=1，凭空多出一格可用空间 */
    if (p->pBuf == 0)
    {
        return 0u;
    }

    return (uint16_t)((uint16_t)(p->mask + 1u) - RB_Used(p));
}

uint16_t RB_Put(RB_t *p, uint8_t b)
{
    if (p->pBuf == 0)
    {
        return 0u;
    }

    if (RB_Used(p) >= (uint16_t)(p->mask + 1u))
    {
        /* 满了。这里选择**丢弃新来的字节**而不是覆盖旧的 ——
           旧数据是一段完整报文的前半截，覆盖它会导致整段报文作废；
           丢新的只是一帧不完整，状态机会靠下一个帧头重新同步。 */
        p->overflow++;
        return 0u;
    }

    p->pBuf[p->head & p->mask] = b;
    p->head++;
    return 1u;
}

uint16_t RB_PutMulti(RB_t *p, const uint8_t *src, uint16_t len)
{
    uint16_t i;
    uint16_t room;

    if ((p->pBuf == 0) || (src == 0))
    {
        return 0u;
    }

    room = RB_Free(p);
    if (room < len)
    {
        /* 能塞多少塞多少，塞不下的全部计入溢出 */
        p->overflow += (uint32_t)(len - room);
        len = room;
    }

    for (i = 0u; i < len; i++)
    {
        p->pBuf[p->head & p->mask] = src[i];
        p->head++;
    }
    return len;
}

uint16_t RB_Get(RB_t *p, uint8_t *pByte)
{
    if ((p->pBuf == 0) || (pByte == 0))
    {
        return 0u;
    }

    if (RB_Used(p) == 0u)
    {
        return 0u;
    }

    *pByte = p->pBuf[p->tail & p->mask];
    p->tail++;
    return 1u;
}

uint16_t RB_GetMulti(RB_t *p, uint8_t *dst, uint16_t max)
{
    uint16_t i;
    uint16_t used;

    if ((p->pBuf == 0) || (dst == 0))
    {
        return 0u;
    }

    used = RB_Used(p);
    if (used == 0u)
    {
        return 0u;
    }

    if (max < used)
    {
        used = max;
    }

    for (i = 0u; i < used; i++)
    {
        dst[i] = p->pBuf[p->tail & p->mask];
        p->tail++;
    }
    return used;
}

void RB_Clear(RB_t *p)
{
    if (p->pBuf == 0)
    {
        return;
    }

    /* 把 tail 追平到 head 就是空；head 不动，避免生产者正在写时被踩到 */
    p->tail = p->head;
}
