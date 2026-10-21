# -*- coding: utf-8 -*-
"""
上位机终端版（D8）：实时接收串口帧并打印

用法（D7 之后上位机也能下发命令了）：
    python monitor.py --list              # 只列出可用串口
    python monitor.py --selftest          # 不开串口，用假数据自检解析与丢帧判据
    python monitor.py COM5                # 指定串口，开始接收并打印
    python monitor.py COM5 --send get_info
    python monitor.py COM5 --send set_rate:500
    python monitor.py COM5 --boundary     # 跑半包/粘包/错帧边界测试
    python monitor.py                     # 不指定则自动选第一个串口
    按 Ctrl+C 结束，结束时会打印统计汇总

关键设计：**流式解析**。
串口 read() 每次返回的字节数是不确定的，一帧很可能被切成两半。
所以这里用了和 MCU 侧 PROTO_Feed 一样的逐字节状态机 ——
不是"攒够一帧再解析"，而是来一个字节处理一个字节。

这样上位机和下位机跑的是同一套逻辑，出问题容易对账。
"""
import sys
import time
import serial
import serial.tools.list_ports as list_ports

from frame_check import (crc16, parse_payload, parse_ack, pack,
                         CMD_ADC_DATA, CMD_MPU_DATA,
                         CMD_STATUS, CMD_SET_RATE, CMD_GET_INFO, CMD_ACK,
                         CMD_NAME)

BAUDRATE = 115200

CRC16_INIT = 0xFFFF
SOF0 = 0xAA
SOF1 = 0x55
PROTO_EOF = 0x0D
MAX_PAYLOAD = 32


class FrameParser:
    """与 MCU 侧 PROTO_Parser_t 对应的状态机"""

    ST_SOF0, ST_SOF1, ST_LEN, ST_CMD, ST_DATA, ST_CRC_LO, ST_CRC_HI, ST_EOF = range(8)

    # 返回值与 PROTO_Status_t 对应
    OK, BUSY, ERR_CRC, ERR_LEN, ERR_EOF = 0, 1, 2, 3, 4

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = self.ST_SOF0
        self.len = 0
        self.cmd = 0
        self.index = 0
        self.crc_calc = CRC16_INIT
        self.crc_lo = 0
        self.payload = bytearray()

    @staticmethod
    def _crc_update(crc, byte):
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) & 0xFFFF if crc & 1 else (crc >> 1) & 0xFFFF
        return crc

    def feed(self, byte):
        """
        喂一个字节。返回 (状态, frame 或 None)
        frame = {"cmd":..., "payload": bytes}
        """
        if self.state == self.ST_SOF0:
            if byte == SOF0:
                self.reset()
                self.crc_calc = self._crc_update(CRC16_INIT, byte)
                self.state = self.ST_SOF1
            return self.BUSY, None

        if self.state == self.ST_SOF1:
            if byte == SOF1:
                self.crc_calc = self._crc_update(self.crc_calc, byte)
                self.state = self.ST_LEN
            elif byte == SOF0:
                self.crc_calc = self._crc_update(CRC16_INIT, byte)
            else:
                self.reset()
            return self.BUSY, None

        if self.state == self.ST_LEN:
            self.crc_calc = self._crc_update(self.crc_calc, byte)
            if byte > MAX_PAYLOAD:
                self.reset()
                return self.ERR_LEN, None
            self.len = byte
            self.state = self.ST_CMD
            return self.BUSY, None

        if self.state == self.ST_CMD:
            self.crc_calc = self._crc_update(self.crc_calc, byte)
            self.cmd = byte
            self.index = 0
            self.payload = bytearray()
            self.state = self.ST_DATA if self.len > 0 else self.ST_CRC_LO
            return self.BUSY, None

        if self.state == self.ST_DATA:
            self.crc_calc = self._crc_update(self.crc_calc, byte)
            self.payload.append(byte)
            self.index += 1
            if self.index >= self.len:
                self.state = self.ST_CRC_LO
            return self.BUSY, None

        if self.state == self.ST_CRC_LO:
            self.crc_lo = byte
            self.state = self.ST_CRC_HI
            return self.BUSY, None

        if self.state == self.ST_CRC_HI:
            crc_recv = (byte << 8) | self.crc_lo
            if crc_recv != self.crc_calc:
                self.reset()
                return self.ERR_CRC, None
            self.state = self.ST_EOF
            return self.BUSY, None

        if self.state == self.ST_EOF:
            if byte != PROTO_EOF:
                self.reset()
                return self.ERR_EOF, None
            frame = {"cmd": self.cmd, "payload": bytes(self.payload)}
            self.reset()
            return self.OK, frame

        self.reset()
        return self.BUSY, None


class SeqChecker:
    """
    靠 ADC 帧里的 seq 查丢帧。

    这里踩过一个坑，记下来免得再犯：
    最早 seq 直接填的是 MCU 的「累计采样轮数」g_adcRoundCount。
    ADC 是 100Hz 采样、1Hz 上报，所以每帧 seq 必然跳 100，
    上位机按「跳号即丢帧」一算，就报出每秒丢 99 帧的假警。
    序号的语义必须是「这是第几帧」，和采样率彻底解耦 ——
    现在 MCU 侧单独维护 s_adcFrameSeq，采样轮数另占一个字段。

    seq 每帧 +1，所以：
        seq == last + 1  正常
        seq >  last + 1  中间丢了 seq-last-1 帧
        seq <= last      MCU 复位过，重新起头
    """

    def __init__(self):
        self.last_seq = -1
        self.lost = 0
        self.restarts = 0
        # 首/末帧的采样轮数，用来反推真实采样率
        self.first_round = -1
        self.last_round = -1
        self.first_t = 0.0
        self.last_t = 0.0

    def feed(self, seq, rnd):
        """返回一句提示（要打印），没问题就返回 None"""
        t_now = time.time()
        msg = None

        if self.last_seq < 0 or seq <= self.last_seq:
            if self.last_seq >= 0:
                msg = "        ^^ MCU 复位，序号 %u -> %u，重新计数" % (self.last_seq, seq)
                self.restarts += 1
            self.first_round, self.first_t = rnd, t_now
        elif seq > self.last_seq + 1:
            n = seq - self.last_seq - 1
            self.lost += n
            msg = "        ^^ 丢帧：%u -> %u，缺 %u 帧" % (self.last_seq, seq, n)

        self.last_seq = seq
        self.last_round = rnd
        self.last_t = t_now
        return msg

    def sample_rate(self):
        """用首尾两帧的轮数差 / 时间差算真实采样率，比逐帧算稳"""
        if self.last_round > self.first_round and self.last_t > self.first_t:
            return (self.last_round - self.first_round) / (self.last_t - self.first_t)
        return None


def list_ports_all():
    return list(list_ports.comports())


def adc_frame(seq, rnd):
    """造一个 ADC 帧（自测用），数值随便填"""
    p = (seq.to_bytes(4, "little") + rnd.to_bytes(4, "little")
         + bytes([0xF8, 0x04, 0xE1, 0x07, 0x3D, 0x08]))
    return pack(CMD_ADC_DATA, p)


def selftest():
    """不开串口，用假数据把「解析 + 丢帧判据」跑一遍"""
    print("自测：解析与丢帧判据")
    print("-" * 74)

    noise = b"[1s] idle=477397 rounds=100 | f=2 e=0\r\n"

    # seq 0,1 正常；故意跳过 2、3 直接发 4（应判丢 2 帧）；再发 5（正常）
    stream = noise
    for s in (0, 1, 4, 5):
        stream += adc_frame(s, s * 100) + noise

    # 打坏一帧 payload（CRC 必然对不上，应报 CRC 错）
    bad = bytearray(adc_frame(6, 600))
    bad[6] ^= 0xFF
    stream += bytes(bad)

    # seq 回绕到 0，模拟 MCU 复位
    stream += adc_frame(0, 0)

    parser = FrameParser()
    checker = SeqChecker()
    stat = {"ok": 0, "crc": 0}

    for b in stream:
        st, frame = parser.feed(b)
        if st == FrameParser.OK:
            stat["ok"] += 1
            name = CMD_NAME.get(frame["cmd"], "CMD_%02X" % frame["cmd"])
            print("[%-6s] %s" % (name, parse_payload(frame["cmd"], frame["payload"])))
            if frame["cmd"] == CMD_ADC_DATA and len(frame["payload"]) >= 14:
                seq = int.from_bytes(frame["payload"][0:4], "little")
                rnd = int.from_bytes(frame["payload"][4:8], "little")
                msg = checker.feed(seq, rnd)
                if msg:
                    print(msg)
        elif st == FrameParser.ERR_CRC:
            stat["crc"] += 1
            print("[CRC 错] 第 %u 次" % stat["crc"])

    print("-" * 74)
    print("完整帧 %u，CRC 错 %u" % (stat["ok"], stat["crc"]))
    print("丢帧 %u，MCU 复位 %u 次" % (checker.lost, checker.restarts))

    # 期望：4 个正常 ADC 帧（0,1,4,5）+ 1 个复位后的 0 = 5 帧
    # CRC 错 1；丢帧 2（缺 seq 2、3）；复位 1
    ok = (stat["ok"] == 5 and stat["crc"] == 1
          and checker.lost == 2 and checker.restarts == 1)
    print("自测 %s（期望：完整帧 5 / CRC 错 1 / 丢帧 2 / 复位 1）"
          % ("通过" if ok else "失败"))
    return 0 if ok else 1


def build_cmd(spec):
    """把 'get_info' / 'set_rate:500' 这样的描述变成一帧字节流"""
    if spec == "get_info":
        return pack(CMD_GET_INFO, b"")

    if spec.startswith("set_rate"):
        try:
            ms = int(spec.split(":")[1])
        except (IndexError, ValueError):
            raise ValueError("set_rate 要写成 set_rate:500（单位毫秒）")
        return pack(CMD_SET_RATE, ms.to_bytes(2, "little"))

    raise ValueError("不认识的命令：%s（可用 get_info / set_rate:N）" % spec)


def drain(ser, parser):
    """
    把串口里积压的历史数据丢掉，保证下一项测试从干净状态开始。

    **光清缓冲是不够的** —— 这里踩过一次：
    MCU 每秒都在主动上报，清掉的字节很可能正好是某帧的后半截，
    解析状态机就此停在半路上（比如还在等 payload 的第 3 个字节），
    接下来喂进去的数据全被当成 payload 吃掉，永远解不出完整帧。
    缓冲区和状态机必须一起复位。
    """
    time.sleep(0.15)
    ser.reset_input_buffer()
    parser.reset()


def wait_acks(ser, parser, seconds, count_all=False):
    """收 seconds 秒并统计帧数

    count_all=False（默认）：只数 MCU 回的 ACK
    count_all=True        ：数所有帧，用来确认串口到底通不通
    """
    t0 = time.time()
    n = 0

    while time.time() - t0 < seconds:
        waiting = ser.in_waiting
        if waiting == 0:
            time.sleep(0.005)
            continue

        for b in ser.read(waiting):
            st, frame = parser.feed(b)
            if st != FrameParser.OK:
                continue

            if frame["cmd"] == CMD_ACK:
                n += 1
                print("        <- %s" % parse_ack(frame["payload"]))
            elif count_all:
                n += 1

    return n


def run_boundary(ser, parser):
    """
    D7 的交付物就是这个：验证「半包 / 粘包 / 错帧」分别被怎么处理。

    串口的本质是**字节流**，没有"帧"的概念。
    一次 write 多少字节、底层怎么分片，上位机和下位机都管不着。
    所以这三种情况一定会发生，必须靠协议设计兜住：
      粘包 —— 两帧连着到达，不能被当成一坨，要解出两帧
      半包 —— 一帧被切成两半，要能等到下一半拼起来，中途不能误判
      错帧 —— CRC 错的帧必须被拒掉，而且要从下一个帧头自动重新同步
    """
    print("边界测试：每项跑之前都会清空串口缓冲并复位状态机")
    print("-" * 74)

    info = pack(CMD_GET_INFO, b"")
    results = []

    # ---- 第 0 项：先确认链路本身是通的 ----
    # 就算什么都不发，MCU 每秒也会主动上报 ADC/MPU 帧。
    # 一帧都收不到说明串口根本不通，后面几项测了也没意义，直接早退。
    drain(ser, parser)
    n = wait_acks(ser, parser, 1.5, count_all=True)
    if n == 0:
        print("[预检] 1.5 秒内一帧都没收到 —— 链路不通，后面的用例不跑了")
        print("       按顺序排查：")
        print("         1. 新程序下载进去了吗？串口助手里看有没有")
        print("            'boot ok, selftest fail=0'（按一下板子 reset 键重看）")
        print("         2. CH340 的 TXD 接的是 PA10 吗？TXD/RXD 是交叉的")
        print("         3. 串口助手关干净了吗？COM 口是独占的")
        return 1
    print("[预检] 1.5 秒内收到 %u 帧，链路正常" % n)
    print("-" * 74)

    # ---- 1. 粘包：两帧背靠背，中间一个字节间隔都不留 ----
    drain(ser, parser)
    ser.write(info + info)
    n = wait_acks(ser, parser, 1.0)
    ok = (n == 2)
    results.append(("粘包 2 帧连发", ok))
    print("[粘包] 期望 2 帧，实得 %u  ->  %s" % (n, "OK" if ok else "异常"))
    print("-" * 74)

    # ---- 2. 半包：把一帧切两半，中间隔 200 ms（远超 IDLE 判定时间） ----
    drain(ser, parser)
    cut = len(info) // 2
    ser.write(info[:cut])
    time.sleep(0.2)                 # 这期间 IDLE 一定触发过
    ser.write(info[cut:])
    n = wait_acks(ser, parser, 1.0)
    ok = (n == 1)
    results.append(("半包 分两次发", ok))
    print("[半包] 期望 1 帧，实得 %u  ->  %s" % (n, "OK" if ok else "异常"))
    print("-" * 74)

    # ---- 3. 半包到极致：一个字节一个字节地发 ----
    drain(ser, parser)
    for b in pack(CMD_GET_INFO, b""):
        ser.write(bytes([b]))
        time.sleep(0.002)
    n = wait_acks(ser, parser, 1.0)
    ok = (n == 1)
    results.append(("逐字节发送", ok))
    print("[逐字节] 期望 1 帧，实得 %u  ->  %s" % (n, "OK" if ok else "异常"))
    print("-" * 74)

    # ---- 4. 错帧：CRC 打坏 + 紧接一个好帧，必须拒坏的、收好的 ----
    drain(ser, parser)
    bad = bytearray(info)
    bad[-3] ^= 0xFF                 # 打坏 CRC 的一个字节
    ser.write(bytes(bad) + info)
    n = wait_acks(ser, parser, 1.0)
    ok = (n == 1)
    results.append(("坏帧后自动重同步", ok))
    print("[错帧] 坏帧应被拒、好帧应通过 -> 期望 1 帧，实得 %u  ->  %s"
          % (n, "OK" if ok else "异常"))
    print("-" * 74)

    failed = sum(1 for _, ok in results if not ok)
    for name, ok in results:
        print("  %-20s %s" % (name, "OK" if ok else "FAIL"))
    print("边界测试 %s（%u/%u 通过）"
          % ("通过" if failed == 0 else "失败", len(results) - failed, len(results)))
    return 0 if failed == 0 else 1


def main():
    args = sys.argv[1:]

    if "--list" in args:
        ports = list_ports_all()
        if not ports:
            print("没找到任何串口")
        for p in ports:
            print("%s  %s" % (p.device, p.description or ""))
        return 0

    if "--selftest" in args:
        return selftest()

    if args and not args[0].startswith("-"):
        port = args[0]
    else:
        ports = list_ports_all()
        if not ports:
            print("没找到串口。用 python monitor.py --list 看看，或手动指定：")
            print("  python monitor.py COM5")
            return 1
        port = ports[0].device
        print("未指定串口，自动选第一个：%s" % port)
        if len(ports) > 1:
            print("  其他可用：%s" % ", ".join(p.device for p in ports[1:]))

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.2)
    except Exception as e:
        print("打开 %s 失败：%s" % (port, e))
        return 1

    print("已打开 %s @ %d" % (port, BAUDRATE))
    print("-" * 74)

    parser = FrameParser()

    # 先把一次性动作跑掉：边界测试 / 下发一条命令。
    # 跑完仍然会落到下面的常规接收循环里，方便接着看 ACK。
    if "--boundary" in args:
        ret = run_boundary(ser, parser)
        ser.close()
        return ret

    if "--send" in args:
        try:
            spec = args[args.index("--send") + 1]
        except IndexError:
            print("--send 后面要跟命令名，例如：get_info 或 set_rate:500")
            ser.close()
            return 1
        try:
            frame = build_cmd(spec)
        except ValueError as e:
            print(e)
            ser.close()
            return 1
        ser.write(frame)
        print("已发送 %s（%u 字节）：%s"
              % (spec, len(frame), " ".join("%02X" % b for b in frame)))

    print("收数据中，Ctrl+C 结束。")
    print("-" * 74)

    checker = SeqChecker()
    stat = {"ok": 0, "crc": 0, "eof": 0, "len": 0, "bytes": 0}
    t_start = time.time()

    try:
        while True:
            waiting = ser.in_waiting
            if waiting == 0:
                time.sleep(0.005)
                continue

            data = ser.read(waiting)
            stat["bytes"] += len(data)

            for b in data:
                st, frame = parser.feed(b)

                if st == FrameParser.OK:
                    stat["ok"] += 1
                    if frame["cmd"] == CMD_ACK:
                        print("[ACK   ] %s" % parse_ack(frame["payload"]))
                    else:
                        name = CMD_NAME.get(frame["cmd"], "CMD_%02X" % frame["cmd"])
                        print("[%-6s] %s" % (name, parse_payload(frame["cmd"],
                                                                 frame["payload"])))

                    # seq 是「第几帧」，每帧 +1；round 是累计采样轮数，两者别混
                    if frame["cmd"] == CMD_ADC_DATA and len(frame["payload"]) >= 14:
                        seq = int.from_bytes(frame["payload"][0:4], "little")
                        rnd = int.from_bytes(frame["payload"][4:8], "little")
                        msg = checker.feed(seq, rnd)
                        if msg:
                            print(msg)

                elif st == FrameParser.ERR_CRC:
                    stat["crc"] += 1
                    print("[CRC 错] 已累计 %u" % stat["crc"])
                elif st == FrameParser.ERR_EOF:
                    stat["eof"] += 1
                elif st == FrameParser.ERR_LEN:
                    stat["len"] += 1

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    elapsed = time.time() - t_start
    print("-" * 74)
    print("运行 %.1f 秒，收到 %u 字节" % (elapsed, stat["bytes"]))
    print("完整帧 %u" % stat["ok"])
    print("CRC 错 %u，帧尾错 %u，长度非法 %u" % (stat["crc"], stat["eof"], stat["len"]))
    if stat["ok"] > 0:
        bad = stat["crc"] + stat["eof"] + stat["len"]
        print("误帧率 %.4f%%  （目标 < 0.01%%）" % (100.0 * bad / (stat["ok"] + bad)))
    print("丢帧 %u（MCU 复位 %u 次）" % (checker.lost, checker.restarts))
    if elapsed > 0:
        print("帧率 %.1f 帧/秒" % (stat["ok"] / elapsed))
    rate = checker.sample_rate()
    if rate is not None:
        print("采样率 %.2f Hz  （TIM3 目标 100Hz）" % rate)

    return 0


if __name__ == "__main__":
    sys.exit(main())
