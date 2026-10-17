#!/usr/bin/env python3
"""IMU 自检工具（维特 WT 协议，0x55 帧）—— 不依赖机器人，插上就能判断"通没通"。

为什么需要它：
  · C++ 侧 `ImuDevice::Initialize()` **只判 open() 成功**，不验证有没有有效数据帧，
    所以"IMU 有"不代表数据真的在流。
  · IMU 是 CH341 USB 转串口（本项目是 USB 口 1-4 → /dev/ttyUSB0）。它一旦枚举失败，
    程序只会打印一行 WARN 然后继续用"单位四元数（假设机身水平）"跑策略 —— 很危险。
  本工具在跑机器人之前先确认：设备在不在、波特率对不对、有没有有效帧、校验对不对。

用法:
    python3 tool/check_imu.py                 # 默认 /dev/ttyUSB0 @115200，测 3 秒
    python3 tool/check_imu.py -p /dev/ttyUSB1
    python3 tool/check_imu.py --secs 5 --raw  # --raw: 不套用安装方向变换
    python3 tool/check_imu.py --selftest      # 不开设备，用伪终端灌合成帧验证解析器

退出码: 0=正常  1=设备打不开  2=打开但收不到有效帧（线/波特率/供电）  3=自检失败
"""
import argparse
import math
import os
import select
import sys
import termios
import time

FRAME_HEAD = 0x55
FRAME_LEN = 11
TYPE_ACC = 0x51
TYPE_GYRO = 0x52
TYPE_QUAT = 0x59
DEG2RAD = math.pi / 180.0
TYPE_NAME = {TYPE_ACC: "acc", TYPE_GYRO: "gyro", TYPE_QUAT: "quat"}


def i16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


class Parser:
    """与 src/strategy/imu_device.cpp 的 ParseByte 等价（含 SUM 校验）"""

    def __init__(self):
        self.buf = bytearray()
        self.count = {TYPE_ACC: 0, TYPE_GYRO: 0, TYPE_QUAT: 0}
        self.bad_sum = 0
        self.resync = 0          # 丢弃的字节数（帧头对不齐的迹象）
        self.acc = self.gyro = self.quat = None

    def feed(self, b):
        if not self.buf:
            if b == FRAME_HEAD:
                self.buf.append(b)
            else:
                self.resync += 1
            return
        self.buf.append(b)
        if len(self.buf) < FRAME_LEN:
            return
        f = bytes(self.buf)
        self.buf.clear()
        if (sum(f[:FRAME_LEN - 1]) & 0xFF) != f[FRAME_LEN - 1]:
            self.bad_sum += 1
            return
        t, d = f[1], f[2:10]
        if t not in self.count:
            return
        self.count[t] += 1
        if t == TYPE_ACC:
            self.acc = [i16(d[0], d[1]) / 32768 * 16 * 9.80665,
                        i16(d[2], d[3]) / 32768 * 16 * 9.80665,
                        i16(d[4], d[5]) / 32768 * 16 * 9.80665]
        elif t == TYPE_GYRO:
            self.gyro = [i16(d[0], d[1]) / 32768 * 2000 * DEG2RAD,
                         i16(d[2], d[3]) / 32768 * 2000 * DEG2RAD,
                         i16(d[4], d[5]) / 32768 * 2000 * DEG2RAD]
        elif t == TYPE_QUAT:
            self.quat = [i16(d[0], d[1]) / 32768.0, i16(d[2], d[3]) / 32768.0,
                         i16(d[4], d[5]) / 32768.0, i16(d[6], d[7]) / 32768.0]


def apply_mount(gyro, quat, mount):
    """与 C++ ImuDevice 的补偿**逐位一致**：
       Z_DOWN_X（绕 X 翻 180°）：gyro 的 gy/gz 取反；quat (w,x,y,z) → (−x, w, −z, y)
       Z_UP / none（无变换）：原样返回
    ⚠ 之前这里只补了 gyro、漏了 quat ⇒ 打印的 roll/pitch/yaw 是未补偿的，会误导诊断。
    """
    if mount in ("none", "Z_UP", "up") or gyro is None:
        return gyro, quat
    g2 = [gyro[0], -gyro[1], -gyro[2]]
    q2 = quat
    if quat is not None and len(quat) == 4:
        w, x, y, z = quat
        q2 = [-x, w, -z, y]          # 绕 X 轴 180°：w'=−x, x'=w, y'=−z, z'=y
    return g2, q2


def configure_raw(fd, baud):
    """termios raw 模式 8N1 + 非阻塞读（等价 C++ open_serial 的设置部分）。
    ⚠ 必须 raw：canonical 模式会按换行符缓冲，而维特帧里没有换行。"""
    attrs = termios.tcgetattr(fd)
    _, _, _, _, _, _, cc = attrs
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
        raise ValueError(f"不支持的波特率 {baud}")
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                      [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL,
                       0, speed, speed, cc])


def open_serial(path, baud):
    """打开并置 raw 模式"""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_raw(fd, baud)
    return fd


def pump(fd, parser, seconds, quiet=False):
    """读 seconds 秒，喂给 parser。返回实际收到的字节数。"""
    n_total = 0
    t0 = time.time()
    last_report = t0
    while time.time() - t0 < seconds:
        r, _, _ = select.select([fd], [], [], 0.2)
        if not r:
            continue
        try:
            data = os.read(fd, 4096)
        except OSError:
            break
        if not data:
            continue
        n_total += len(data)
        for b in data:
            parser.feed(b)
        if not quiet and time.time() - last_report >= 1.0:
            last_report = time.time()
            print(f"    … {time.time()-t0:4.1f}s  已收 {n_total} 字节  "
                  f"quat {parser.count[TYPE_QUAT]} 帧  gyro {parser.count[TYPE_GYRO]} 帧")
    return n_total


def euler(quat):
    w, x, y, z = quat
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def list_candidates():
    out = []
    for d in sorted(os.listdir("/dev")):
        if d.startswith("ttyUSB") or d.startswith("ttyACM"):
            out.append("/dev/" + d)
    return out


def report(parser, nbytes, seconds, mount, raw):
    print()
    print(f"  收到字节 {nbytes}（{nbytes/seconds:.0f} B/s）；帧数: "
          + "  ".join(f"{TYPE_NAME[t]}={parser.count[t]}" for t in
                      (TYPE_ACC, TYPE_GYRO, TYPE_QUAT)))
    print(f"  校验失败 {parser.bad_sum} 帧；帧头外字节 {parser.resync}")
    rate = parser.count[TYPE_QUAT] / seconds
    if parser.quat:
        gyro, quat = apply_mount(parser.gyro, parser.quat, "none" if raw else mount)
        print(f"  最新 quat = {[round(v, 4) for v in quat]}"
              + ("" if raw else f"（已套用安装方向 {mount}）"))
        r, p, y = euler(quat)
        print(f"            ⇒ roll {r:+.1f}°  pitch {p:+.1f}°  yaw {y:+.1f}°")
        if gyro:
            print(f"  最新 gyro = {[round(v, 3) for v in gyro]} rad/s")
    return rate


def main():
    ap = argparse.ArgumentParser(description="IMU 自检（维特 WT 协议）")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="串口路径")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=3.0, help="采集时长（秒）")
    ap.add_argument("--mount", default="Z_DOWN_X",
                    choices=["Z_DOWN_X", "Z_UP", "none"],
                    help="安装方向补偿；Z_UP/none = 不补偿（对应 app 里 IMU_MOUNT=up）")
    ap.add_argument("--raw", action="store_true", help="不套用安装方向变换")
    ap.add_argument("--selftest", action="store_true", help="不开设备，用伪终端验证解析器")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    print(f"[check_imu] 打开 {args.port} @ {args.baud} …")
    if not os.path.exists(args.port):
        cands = list_candidates()
        print(f"🔴 设备不存在: {args.port}")
        print(f"   当前存在的串口: {cands if cands else '（一个都没有）'}")
        print("   排查（IMU 是 CH341 USB 转串口）:")
        print("     1) lsusb | grep -i 1a86        # 应看到 CH341；看不到 = 没枚举成功")
        print("     2) journalctl -k -f            # 拔插时观察：")
        print("          正常 → 'ch341-uart converter detected' + 'attached to ttyUSB0'")
        print("          失败 → 'device descriptor read/64, error -71'(EPROTO) = 线/供电/模块")
        print("     3) 换线、换 USB 口（直插机箱，别走 hub），必要时换一个 CH341 模块")
        print("   注: 达妙 CAN 适配器是 ttyACM*（cdc_acm），不是 IMU，别搞混。")
        return 1
    try:
        fd = open_serial(args.port, args.baud)
    except OSError as e:
        print(f"🔴 打开失败: {e}（权限？被别的进程占用？fuser -v {args.port}）")
        return 1
    try:
        parser = Parser()
        print(f"[check_imu] 采集 {args.secs:g}s …")
        n = pump(fd, parser, args.secs)
    finally:
        os.close(fd)

    rate = report(parser, n, args.secs, args.mount, args.raw)
    print()
    if parser.count[TYPE_QUAT] == 0 and parser.count[TYPE_GYRO] == 0:
        print("🔴 打开成功但**收不到任何有效帧** ⇒ 波特率不对 / 线序(TX-RX)不对 / 模块没供电")
        return 2
    if parser.bad_sum > parser.count[TYPE_QUAT] + parser.count[TYPE_GYRO]:
        print("🔴 校验失败帧多于有效帧 ⇒ 波特率不对或线太长/干扰大")
        return 2
    if rate < 20:
        print(f"⚠ 有效帧率偏低（{rate:.1f} Hz）⇒ 线材/波特率临界，或模块配置的 RRATE 很低")
        return 2
    print(f"✅ IMU 正常：{rate:.0f} Hz 有效帧，校验全过。可以跑机器人了。")
    return 0


def _synth_frame(type_byte, values):
    """打包成**定长 11 字节**的维特帧：0x55 + type + 8 数据字节 + SUM。
    ⚠ 帧长恒定：陀螺帧也是 8 个数据字节（6 个角速度 + 2 个温度/版本），
      只填 6 个会让帧变成 9 字节、后续全部错位（自检抓到的第一个 bug）。"""
    body = [FRAME_HEAD, type_byte]
    for v in list(values) + [0] * (4 - len(values)):   # 补齐到 4 个 int16
        i = int(max(-32768, min(32767, v)))
        body += [i & 0xFF, (i >> 8) & 0xFF]
    body.append(sum(body) & 0xFF)
    assert len(body) == FRAME_LEN, f"帧长 {len(body)} != {FRAME_LEN}"
    return bytes(body)


def selftest():
    """用伪终端灌入合成帧，验证解析器（含故意做坏的校验帧）"""
    master, slave = os.openpty()
    configure_raw(slave, 115200)   # ⚠ 不置 raw 的话 canonical 行规程会吃掉无换行的合成帧
    name = os.ttyname(slave)
    expect_q = [0.5, -0.5, 0.5, 0.5]
    expect_g = [100.0, -200.0, 300.0]          # 原始刻度
    frames = bytearray()
    frames += _synth_frame(TYPE_QUAT, [int(v * 32768) for v in expect_q])
    frames += _synth_frame(TYPE_GYRO, expect_g)
    bad = bytearray(_synth_frame(TYPE_QUAT, [1000, 1000, 1000, 1000]))
    bad[-1] ^= 0xFF                             # 破坏校验
    frames += bad
    frames += b"\x01\x02\x03"                   # 帧头外字节
    os.write(master, bytes(frames))

    parser = Parser()
    pump(slave, parser, 0.5, quiet=True)
    os.close(master)
    os.close(slave)

    q_ok = parser.quat is not None and all(
        abs(a - b) < 2e-4 for a, b in zip(parser.quat, expect_q))
    # gyro 原始刻度 ±2000°/s
    g_exp = [v / 32768 * 2000 * DEG2RAD for v in expect_g]
    g_ok = parser.gyro is not None and all(
        abs(a - b) < 1e-3 for a, b in zip(parser.gyro, g_exp))
    checks = [
        ("quat 解析", q_ok, f"{parser.quat}"),
        ("gyro 解析", g_ok, f"{parser.gyro}"),
        ("有效帧计数", parser.count[TYPE_QUAT] == 1 and parser.count[TYPE_GYRO] == 1,
         f"quat={parser.count[TYPE_QUAT]} gyro={parser.count[TYPE_GYRO]}"),
        ("坏校验被拒", parser.bad_sum == 1, f"bad_sum={parser.bad_sum}"),
        ("帧头外字节计数", parser.resync == 3, f"resync={parser.resync}"),
        ("ptyname", name.startswith("/dev/pts/"), name),
    ]
    print("=== check_imu 自检 ===")
    allok = True
    for name_, ok, info in checks:
        print(f"  {'✅' if ok else '❌'} {name_:16s} {info}")
        allok &= ok
    print("\n" + ("✅ 全部通过" if allok else "❌ 有失败项"))
    return 0 if allok else 3


if __name__ == "__main__":
    sys.exit(main())
