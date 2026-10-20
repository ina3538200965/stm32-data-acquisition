# -*- coding: utf-8 -*-
"""
串口帧校验工具（D8 上位机的雏形）

用法：
    python frame_check.py "AA 55 0A 01 6E 3B 00 00 ... 0D"
    python frame_check.py                      # 不带参数则用内置示例

功能：
    1. 从字节流里按帧头 + LEN 定位每一帧（文本噪声会自动跳过）
    2. 验 CRC16（Modbus）和帧尾
    3. 按命令字解出 payload 并打印成人能看懂的数值
    4. 统计：收到几帧、CRC 错几帧、帧尾错几帧

这个脚本和 MCU 侧的 protocol.c 用的是同一套算法，
所以两边对得上就说明协议实现是正确且一致的。
"""
import sys

CRC16_INIT = 0xFFFF
CRC16_POLY = 0xA001

SOF0 = 0xAA
SOF1 = 0x55
EOF = 0x0D
MAX_PAYLOAD = 32

CMD_ADC_DATA = 0x01
CMD_MPU_DATA = 0x02
CMD_STATUS = 0x03

CMD_NAME = {
    CMD_ADC_DATA: "ADC",
    CMD_MPU_DATA: "MPU",
    CMD_STATUS: "STATUS",
}


def crc16(data):
    """CRC-16/Modbus：多项式 0xA001，初值 0xFFFF"""
    crc = CRC16_INIT
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ CRC16_POLY) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


def u16(buf, off):
    return buf[off] | (buf[off + 1] << 8)


def s16(buf, off):
    v = u16(buf, off)
    return v - 0x10000 if v >= 0x8000 else v


def u32(buf, off):
    return (buf[off] | (buf[off + 1] << 8)
            | (buf[off + 2] << 16) | (buf[off + 3] << 24))


def pack(cmd, payload):
    """按帧格式组一帧，返回 bytes。D7 上位机下发命令时也用它"""
    body = bytes([SOF0, SOF1, len(payload), cmd]) + bytes(payload)
    crc = crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF, EOF])


def parse_payload(cmd, p):
    """按命令字解析 payload，返回可读字符串"""
    if cmd == CMD_ADC_DATA and len(p) >= 14:
        # seq = 帧序号（每帧 +1，查丢帧）；round = 累计采样轮数（查采样率）
        seq = u32(p, 0)
        rnd = u32(p, 4)
        ch0, ch1, ch2 = u16(p, 8), u16(p, 10), u16(p, 12)
        return ("seq=%-6u round=%-8u CH0=%-5u CH1=%-5u CH2=%-5u"
                % (seq, rnd, ch0, ch1, ch2))

    if cmd == CMD_MPU_DATA and len(p) >= 14:
        ax, ay, az = s16(p, 0), s16(p, 2), s16(p, 4)
        gx, gy, gz = s16(p, 6), s16(p, 8), s16(p, 10)
        temp_raw = s16(p, 12)
        temp_c = temp_raw // 334 + 21
        return ("ax=%-6d ay=%-6d az=%-6d | gx=%-5d gy=%-5d gz=%-5d | T=%dC"
                % (ax, ay, az, gx, gy, gz, temp_c))

    return "raw=%s" % " ".join("%02X" % b for b in p)


def scan_frames(buf):
    """
    扫描字节流，找出所有帧。
    返回 (帧列表, 统计字典)。帧里带 ok 标记和错误信息。
    """
    frames = []
    stat = {"ok": 0, "crc_err": 0, "eof_err": 0, "len_err": 0, "truncated": 0}

    i = 0
    n = len(buf)
    while i < n - 1:
        # 找帧头
        if not (buf[i] == SOF0 and buf[i + 1] == SOF1):
            i += 1
            continue

        # 帧头后至少需要 LEN CMD 两个字节
        if i + 3 > n:
            break
        length = buf[i + 2]
        cmd = buf[i + 3]

        if length > MAX_PAYLOAD:
            stat["len_err"] += 1
            frames.append({"ok": False, "err": "LEN=%u 非法" % length, "pos": i})
            i += 2
            continue

        total = 4 + length + 3          # SOF..CMD + payload + CRC(2) + EOF
        if i + total > n:
            stat["truncated"] += 1
            frames.append({"ok": False, "err": "数据不完整（缺 %u 字节）"
                           % (i + total - n), "pos": i})
            break

        payload = buf[i + 4: i + 4 + length]
        crc_recv = u16(buf, i + 4 + length)
        crc_calc = crc16(buf[i: i + 4 + length])
        eof_byte = buf[i + total - 1]

        item = {
            "pos": i,
            "cmd": cmd,
            "len": length,
            "payload": payload,
            "crc_recv": crc_recv,
            "crc_calc": crc_calc,
            "eof": eof_byte,
            "total": total,
        }

        if crc_recv != crc_calc:
            stat["crc_err"] += 1
            item["ok"] = False
            item["err"] = "CRC 错：收到 %04X，算出 %04X" % (crc_recv, crc_calc)
        elif eof_byte != EOF:
            stat["eof_err"] += 1
            item["ok"] = False
            item["err"] = "帧尾错：%02X（应为 %02X）" % (eof_byte, EOF)
        else:
            stat["ok"] += 1
            item["ok"] = True
            item["err"] = ""

        frames.append(item)
        i += total

    return frames, stat


def main():
    if len(sys.argv) > 1:
        try:
            buf = bytes(int(x, 16) for x in sys.argv[1].split())
        except ValueError as e:
            print("hex 解析失败：%s" % e)
            return 1
    else:
        # 内置示例：现场用 pack 组帧，CRC 永远自洽，不会因改协议而失效。
        # 数值取自 2026-10-20 实机抓到的那两帧。
        # 帧前后故意拼 printf 文本噪声，演示扫描时能自动跳过。
        noise = b"[1s] idle=477397 rounds=100 | f=2 e=0\r\n"
        adc = pack(CMD_ADC_DATA,
                   b"\x6e\x3b\x00\x00"      # seq   = 15214
                   b"\x71\x03\x00\x00"      # round = 881
                   b"\xf8\x04\xe1\x07\x3d\x08")   # CH0=1272 CH1=2017 CH2=2109
        mpu = pack(CMD_MPU_DATA,
                   b"\x9c\xff\x30\x00\x70\x3f\x05\x00\xfd\xff\x03\x00\x18\x08")
        buf = noise + adc + noise + mpu

    print("输入 %u 字节" % len(buf))
    print("-" * 78)

    frames, stat = scan_frames(buf)

    for f in frames:
        name = CMD_NAME.get(f.get("cmd", 0), "CMD_%02X" % f.get("cmd", 0))
        if f["ok"]:
            print("[OK  ] @%-4u %-6s LEN=%-3u CRC=%04X  %s"
                  % (f["pos"], name, f["len"], f["crc_recv"],
                     parse_payload(f["cmd"], f["payload"])))
        else:
            print("[FAIL] @%-4u %s" % (f["pos"], f["err"]))

    print("-" * 78)
    print("完整帧 %u，CRC 错 %u，帧尾错 %u，长度非法 %u，半包 %u"
          % (stat["ok"], stat["crc_err"], stat["eof_err"],
             stat["len_err"], stat["truncated"]))

    return 0 if (stat["crc_err"] == 0 and stat["eof_err"] == 0
                 and stat["len_err"] == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
