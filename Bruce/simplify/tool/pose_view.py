#!/usr/bin/env python3
"""关节姿态可视化（Python + MuJoCo）—— 调试用，2026-10-12

用**真实 MJCF 做正运动学**（只调 `mj_forward`，**不跑物理**），把 12 个腿关节角
画成连杆骨架；机身位姿可用 IMU 四元数（数据里有就自动用）。

设计要点（见 §坐标链路）：
  · 内部统一用 **URDF 坐标 + POLICY 关节序**，输入侧按日志类型自动换算：
      log/dataset_*.csv  m_pos_*   → status(CAN 序) → URDF(POLICY 序)
      log/recv_*.csv     cal_pos   → status(CAN 序) → URDF(POLICY 序)
      log/pace/*.csv     meas_*    → 已是 URDF(PACE 序) → POLICY 序
  · 换算常数**不手抄**，启动时从 C++ 源文件解析（项目规范：单一真值来源）
      src/strategy/sim2real_conv.cpp   → DEFAULT_POSE / CONV_A / CONV_B
      dogurdf_sim2sim_deploy/src/dogurdf.py → NOMINAL_HIP/THIGH/CALF/TORSO_HEIGHT
  · 骨架 = 只显示 MJCF 里的**碰撞体**（每条连杆都有 group=3 的圆柱/方盒），
    关掉 group=1 的 mesh ⇒ 不用另写模型。

用法（**必须**用装了 mujoco 的环境）：
    PY=/home/bruce/miniforge3/envs/MJX/bin/python
    $PY tool/pose_view.py                      # 内建预设：部署侧 DEFAULT_POSE
    $PY tool/pose_view.py --preset nominal     # 训练/sim2sim 的 NOMINAL_*（对比用）
    $PY tool/pose_view.py --preset zero        # 物理零位姿态（status = 0/0/π2）
    $PY tool/pose_view.py --preset zero-kin    # 运动学理论零位姿态（见下）
    $PY tool/pose_view.py --csv log/dataset_20261006_130814.csv
    $PY tool/pose_view.py --csv log/recv_20261008_211830.csv
    $PY tool/pose_view.py --selftest           # 不开窗口，只验证坐标链路

    # 实时（机身姿态走 IMU）：C++ 侧先开姿态流，再起 viewer
    POSE_STREAM=1 ./run.sh                     # 机甲进程（UDP 发到 127.0.0.1:45555）
    $PY tool/pose_view.py --udp                # viewer 监听 45555（端口不同就 --udp <port>）
    $PY tool/pose_view.py --udp --shot live.png   # 实时抓一帧存档

实时协议：见 include/common/pose_stream.h（164 字节 UDP：seq/flags/t_us/quat[4]/gyro[3]/
q[16]/q_target[12]，坐标 = URDF + POLICY 序）。**viewer 没开也不影响机器人**（非阻塞、丢帧无妨）。
环境变量：POSE_STREAM=1 启用；POSE_HOST/POSE_PORT/POSE_HZ 可调。
窗口内按键（GLFW）：
    空格 暂停/继续   N 单步   R 回起点   1..4 切预设   0 隐藏机身   Q 退出
"""
from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import re
import sys
import socket
import struct
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # …/Bruce/simplify
XML_PATH = os.path.join(
    ROOT, "dogurdf_sim2sim_deploy", "assets", "bot_model", "dogurdf", "dogurdf.xml")
CONV_CPP = os.path.join(ROOT, "src", "strategy", "sim2real_conv.cpp")
DOGURDF_PY = os.path.join(ROOT, "dogurdf_sim2sim_deploy", "src", "dogurdf.py")

LEGS = ("FL", "FR", "RL", "RR")
# POLICY 序：12 腿关节（按腿分组）+ 4 轮。与 sim2real_conv.cpp 的 CONV_A/B 索引一致。
POLICY_NAMES = ([f"{leg}_{j}_joint" for leg in LEGS for j in ("hip", "thigh", "calf")]
                + [f"{leg}_wheel_joint" for leg in LEGS])
# CAN 序 → POLICY 序（i = can*4 + motor_id-1；policy[CAN_TO_POLICY[i]] = status[i]）
CAN_TO_POLICY = [0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15]
# PACE joint_order 名 → POLICY 索引（PACE 按腿分组 HAA/HFE/KFE）
_PACE_LEG = {"LF": "FL", "RF": "FR", "LH": "RL", "RH": "RR"}
_PACE_JNT = {"HAA": "hip", "HFE": "thigh", "KFE": "calf"}
PACE_TO_POLICY = {
    f"{pl}_{pj}": POLICY_NAMES.index(f"{_PACE_LEG[pl]}_{_PACE_JNT[pj]}_joint")
    for pl in _PACE_LEG for pj in _PACE_JNT
}
PACE_ORDER = [f"{pl}_{pj}" for pl in ("LF", "RF", "LH", "RH") for pj in ("HAA", "HFE", "KFE")]

# Ex57 从 MuJoCo 反推的"物理零位姿态"运动学理论 URDF 角（hip/thigh/calf，单腿）
ZERO_KIN_URDF = (0.000, -1.020, +0.168)


# =====================================================================
#  坐标链路：从 C++ / Python 源文件解析常数（禁止手抄）
# =====================================================================
def _strip_comments(src: str) -> str:
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def _cpp_array(src: str, name: str) -> list[float]:
    m = re.search(rf"\b{name}\s*\[[^\]]*\]\s*=\s*\{{(.*?)\}}", src, re.S)
    if not m:
        raise RuntimeError(f"在源码里找不到数组 {name}")
    return [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", m.group(1))]


def load_calib() -> dict:
    """从 sim2real_conv.cpp / dogurdf.py 读出全部坐标常数。"""
    cpp = _strip_comments(open(CONV_CPP, encoding="utf-8").read())
    conv_a = _cpp_array(cpp, "CONV_A")
    conv_b = _cpp_array(cpp, "CONV_B")
    default_pose = _cpp_array(cpp, "DEFAULT_POSE")
    if not (len(conv_a) == len(conv_b) == len(default_pose) == 16):
        raise RuntimeError("CONV_A / CONV_B / DEFAULT_POSE 长度必须都是 16")

    py = open(DOGURDF_PY, encoding="utf-8").read()
    def _num(name):
        m = re.search(rf"^{name}\s*=\s*([-+]?[0-9.]+)", py, re.M)
        if not m:
            raise RuntimeError(f"dogurdf.py 里找不到 {name}")
        return float(m.group(1))
    nominal = {
        "hip": _num("NOMINAL_HIP"),
        "thigh": _num("NOMINAL_THIGH"),
        "calf": _num("NOMINAL_CALF"),
        "torso_z": _num("NOMINAL_TORSO_HEIGHT"),
    }
    return {"A": conv_a, "B": conv_b, "default": default_pose, "nominal": nominal}


def status_to_urdf(status_policy: np.ndarray, cal: dict) -> np.ndarray:
    """status(POLICY 序) → URDF(POLICY 序)：URDF = A·status + B"""
    return np.array(cal["A"]) * np.asarray(status_policy) + np.array(cal["B"])


def urdf_to_status(urdf_policy: np.ndarray, cal: dict) -> np.ndarray:
    return (np.asarray(urdf_policy) - np.array(cal["B"])) / np.array(cal["A"])


def can_to_policy_status(can16: np.ndarray) -> np.ndarray:
    """CAN 序 → POLICY 序"""
    out = np.zeros(16)
    for i, p in enumerate(CAN_TO_POLICY):
        out[p] = can16[i]
    return out


# =====================================================================
#  内建姿态预设
# =====================================================================
def build_presets(cal: dict) -> dict:
    """返回 {名字: (URDF16, 说明)}"""
    p = {}
    p["default"] = (np.array(cal["default"]),
                    "部署侧 DEFAULT_POSE（sim2real_conv.cpp）")
    nom = cal["nominal"]
    nom16 = np.array([nom["hip"], nom["thigh"], nom["calf"]] * 4 + [0.0] * 4)
    p["nominal"] = (nom16, "训练/sim2sim 侧 NOMINAL_*（dogurdf.py）")
    # 物理零位姿态：status = hip 0 / thigh 0 / calf π/2（Ex20/Ex45 的定义）
    st = np.zeros(16)
    for leg in range(4):
        st[leg * 3 + 0] = 0.0
        st[leg * 3 + 1] = 0.0
        st[leg * 3 + 2] = math.pi / 2
    p["zero"] = (status_to_urdf(st, cal), "物理零位姿态（status 0 / 0 / π2）")
    # 运动学理论零位（Ex57 从 MuJoCo 反推）——与上面那条的差就是 CONV_B 残差
    kin = np.array(list(ZERO_KIN_URDF) * 4 + [0.0] * 4)
    p["zero-kin"] = (kin, "运动学理论零位（Ex57）——与 zero 之差 = CONV_B 残差")
    return p


# =====================================================================
#  数据源：统一吐出 (t_ms, URDF16, quat4|None)
# =====================================================================
class Frames:
    def __init__(self, t, urdf, quat=None, label="?", layer="?"):
        self.t = np.asarray(t, dtype=float)                 # ms
        self.urdf = np.asarray(urdf, dtype=float)           # (N,16) POLICY 序
        self.quat = quat                                    # (N,4) 或 None
        self.label = label
        self.layer = layer

    def __len__(self):
        return len(self.t)

    def at(self, t_ms: float):
        i = max(0, min(len(self) - 1, bisect.bisect_left(self.t, t_ms)))
        q = None if self.quat is None else self.quat[i]
        return self.urdf[i], q


def load_dataset(path: str, cal: dict) -> Frames:
    """统一 500 Hz 数据集（188 列）：m_pos_00..15 是 status(CAN 序)，另有 quat。"""
    with open(path, newline="") as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        idx = {n: hdr.index(n) for n in hdr}
        need = [f"m_pos_{i:02d}" for i in range(16)]
        if not all(n in idx for n in need):
            raise RuntimeError("不是统一数据集（缺 m_pos_00..15）")
        mpos = [idx[n] for n in need]
        t_i = idx["t_ms"]
        q_i = [idx[n] for n in ("quat_w", "quat_x", "quat_y", "quat_z")]
        t, urdf, quat = [], [], []
        for row in rd:
            if len(row) != len(hdr):
                continue
            st = can_to_policy_status(np.array([float(row[i]) for i in mpos]))
            t.append(float(row[t_i]))
            urdf.append(status_to_urdf(st, cal))
            quat.append(np.array([float(row[i]) for i in q_i]))
    return Frames(t, urdf, np.array(quat), os.path.basename(path), "status(CAN) → URDF")


def load_recv(path: str, cal: dict, frame_ms: int = 2) -> Frames:
    """log/recv_*.csv：一行 = 一个电机一帧；按时间窗组装成整机帧。cal_pos 是 status。"""
    with open(path, newline="") as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        c = {n: hdr.index(n) for n in ("elapsed_ms", "can_port", "motor_id", "cal_pos")}
        latest = np.zeros(16)
        t, urdf = [], []
        bucket = None
        for row in rd:
            if len(row) != len(hdr):
                continue
            b = int(row[c["elapsed_ms"]]) // frame_ms
            if bucket is None:
                bucket = b
            elif b != bucket:
                t.append(bucket * frame_ms)
                urdf.append(status_to_urdf(can_to_policy_status(latest.copy()), cal))
                bucket = b
            can, mid = int(row[c["can_port"]]), int(row[c["motor_id"]])
            if 0 <= can < 4 and 1 <= mid <= 4:
                latest[can * 4 + mid - 1] = float(row[c["cal_pos"]])
        t.append(bucket * frame_ms)
        urdf.append(status_to_urdf(can_to_policy_status(latest.copy()), cal))
    return Frames(t, urdf, None, os.path.basename(path), "status(CAN) → URDF（无 IMU）")


def load_pace(path: str, cal: dict) -> Frames:
    """log/pace/chirp_*.csv：meas_<PACE 名> 已经是 URDF（PACE 序）。"""
    with open(path, newline="") as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        cols = [hdr.index(f"meas_{n}") for n in PACE_ORDER]
        t_i = hdr.index("t_actual_s")
        t, urdf = [], []
        for row in rd:
            if len(row) != len(hdr):
                continue
            u = np.zeros(16)
            for col, name in zip(cols, PACE_ORDER):
                u[PACE_TO_POLICY[name]] = float(row[col])
            t.append(float(row[t_i]) * 1000.0)
            urdf.append(u)
    return Frames(t, urdf, None, os.path.basename(path), "URDF(PACE) → POLICY（无 IMU）")


def load_csv(path: str, cal: dict) -> Frames:
    with open(path, newline="") as fh:
        hdr = next(csv.reader(fh))
    if "m_pos_00" in hdr:
        return load_dataset(path, cal)
    if "cal_pos" in hdr:
        return load_recv(path, cal)
    if "meas_LF_HAA" in hdr:
        return load_pace(path, cal)
    raise RuntimeError(f"认不出这个 CSV 的格式：{path}")


# =====================================================================
#  实时源：UDP 姿态流（C++ 侧 POSE_STREAM=1 时发送，格式见 include/common/pose_stream.h）
# =====================================================================
POSE_MAGIC = 0x444F4750          # 'DOGP'
POSE_FMT = "<IIIIQ4f3f16f12f"    # magic,ver,seq,flags,t_us,quat[4],gyro[3],q[16],q_tgt[12]
POSE_SIZE = struct.calcsize(POSE_FMT)


class UdpPoseSource:
    """后台线程收姿态流，**永远只保留最新一帧**（丢帧无所谓）。

    C++ 侧发的是 **URDF 坐标 + POLICY 序**（12 腿 + 4 轮），可直接写进 MuJoCo 的 qpos。
    """

    def __init__(self, port: int = 45555, bind: str = "0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.urdf = None          # (16,) URDF/POLICY 序
        self.q_target = None      # (12,) 或 None
        self.quat = None          # (4,) 或 None
        self.gyro = None
        self.seq = -1
        self.lost = 0
        self.n = 0
        self.t_last = 0.0
        self.rate = 0.0
        self._stop = False
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        t_prev, n_prev = time.time(), 0
        while not self._stop:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) != POSE_SIZE:
                continue
            f = struct.unpack(POSE_FMT, data)
            if f[0] != POSE_MAGIC:
                continue
            # ⚠ struct.unpack 返回**扁平**元组（40 个值），不能按字段名解包，按切片取
            seq, flags = f[2], f[3]
            quat, gyro, q, q_tgt = f[5:9], f[9:12], f[12:28], f[28:40]
            now = time.time()
            with self.lock:
                if self.seq >= 0 and seq > self.seq + 1:
                    self.lost += seq - self.seq - 1
                self.seq = seq
                self.n += 1
                self.t_last = now
                self.urdf = np.array(q, dtype=float)
                self.q_target = np.array(q_tgt, dtype=float) if (flags & 4) else None
                self.quat = np.array(quat, dtype=float) if (flags & 1) else None
                self.gyro = np.array(gyro, dtype=float) if (flags & 2) else None
                if now - t_prev >= 1.0:
                    self.rate = (self.n - n_prev) / (now - t_prev)
                    t_prev, n_prev = now, self.n

    def latest(self):
        with self.lock:
            return (self.urdf, self.q_target, self.quat, self.gyro,
                    self.seq, self.lost, self.rate, self.t_last)

    def close(self):
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass


# =====================================================================
#  MuJoCo：模型 + qpos 组装
# =====================================================================
def load_model():
    import mujoco
    if not os.path.exists(XML_PATH):
        raise RuntimeError(f"找不到 MJCF: {XML_PATH}")
    m = mujoco.MjModel.from_xml_path(XML_PATH)
    adr = np.empty(16, dtype=int)
    for k, name in enumerate(POLICY_NAMES):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"MJCF 里找不到关节 {name}")
        adr[k] = m.jnt_qposadr[jid]
    return m, adr


def make_skeleton(m) -> None:
    """把模型变成"连杆骨架"外观。

    ⚠ MJCF 里**所有碰撞体（group=3）的 rgba alpha 都是 0**（仿真里本来就不可见，
      模型外观全靠 group=1 的 mesh），所以"关掉 mesh 只留碰撞体"会得到一副**隐形的骨架**。
      这里直接把 group=3 的 alpha 抬到 1（保留原色）、把 group=1 的压到 0，
      于是无论 viewer 的 geomgroup 怎么设，看到的都是连杆。
    """
    n3 = 0
    for i in range(m.ngeom):
        g = int(m.geom_group[i])
        if g == 3:
            m.geom_rgba[i, 3] = 1.0
            n3 += 1
        elif g == 1:
            m.geom_rgba[i, 3] = 0.0
    # site 会在渲染里显示成小白球，骨架模式下关掉更干净
    for i in range(m.nsite):
        m.site_rgba[i, 3] = 0.0
    print(f"[pose_view] 骨架模式：{n3} 个碰撞体置为不透明，mesh/site 置透明")


def set_pose(mujoco, m, d, adr, urdf16, quat=None, z=0.40, show_body=True):
    if m.nq >= 7:                       # 有 freejoint：可以摆机身
        d.qpos[0:3] = [0.0, 0.0, z if show_body else -5.0]
        if quat is not None:
            q = np.asarray(quat, dtype=float)
            n = np.linalg.norm(q)
            d.qpos[3:7] = q / n if n > 1e-9 else np.array([1.0, 0.0, 0.0, 0.0])
        else:
            d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    d.qpos[adr] = np.asarray(urdf16, dtype=float)
    mujoco.mj_forward(m, d)


def write_png(path: str, rgb: np.ndarray) -> None:
    """极简 PNG 写出（只用 zlib，不依赖 PIL/imageio）。rgb: (H,W,3) uint8。"""
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    blob = b"\x89PNG\r\n\x1a\n"
    blob += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    blob += chunk(b"IDAT", zlib.compress(raw, 6))
    blob += chunk(b"IEND", b"")
    with open(path, "wb") as fh:
        fh.write(blob)


def render_shot(mujoco, m, d, adr, urdf16, quat, z, path, size, show_mesh=False,
                azimuth=128.0, elevation=-14.0, distance=1.85):
    """离屏渲染一张图（用于文档/复核，不需要开窗口）。"""
    w, h = (int(x) for x in size.lower().split("x"))
    set_pose(mujoco, m, d, adr, urdf16, quat, z=z)
    opt = mujoco.MjvOption()
    opt.geomgroup[1] = 1 if show_mesh else 0     # 1 = mesh
    opt.geomgroup[3] = 1                          # 3 = 碰撞体（连杆骨架）
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0.0, 0.0, z * 0.55]
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    r = mujoco.Renderer(m, h, w)
    r.update_scene(d, camera=cam, scene_option=opt)
    img = r.render()
    r.close()
    write_png(path, img)
    print(f"[pose_view] 已出图 {path}（{w}x{h}，{'mesh' if show_mesh else '连杆骨架'}）")

# =====================================================================
#  selftest：不开窗口，只验证坐标链路
# =====================================================================
def selftest(pre: dict, cal: dict) -> int:
    import mujoco
    print("=== 坐标链路自检（不开窗口）===")
    print(f"从 C++ 解析：DEFAULT_POSE={np.round(cal['default'], 4).tolist()}")
    print(f"            CONV_A    ={cal['A']}")
    print(f"            CONV_B    ={np.round(cal['B'], 4).tolist()}")
    print(f"从 dogurdf.py 解析：NOMINAL_*={cal['nominal']}")
    print()
    ok = True
    # 1) 关节名都能在 MJCF 里找到
    try:
        m, adr = load_model()
        print(f"[1] MJCF 加载成功：nq={m.nq} nv={m.nv} timestep={m.opt.timestep}")
        print(f"    16 个关节的 qpos 地址 = {adr.tolist()}")
        d = mujoco.MjData(m)
    except Exception as e:                                   # noqa: BLE001
        print(f"[1] 🔴 {e}")
        return 1
    # 2) 往返自检：status → URDF → status
    rng = np.random.default_rng(0)
    st = rng.uniform(-1.0, 1.0, 16)
    back = urdf_to_status(status_to_urdf(st, cal), cal)
    err = float(np.max(np.abs(back - st)))
    print(f"[2] status→URDF→status 往返最大误差 = {err:.3e}  {'✅' if err < 1e-12 else '🔴'}")
    ok &= err < 1e-12
    # 3) CAN→POLICY 置换：双射且腿/轮范围正确
    p = can_to_policy_status(np.arange(16.0))
    perm_ok = sorted(p.tolist()) == list(range(16))
    print(f"[3] CAN→POLICY 是双射 = {perm_ok}  轮子在 policy[12:16] = "
          f"{p[12:16].astype(int).tolist()}  {'✅' if perm_ok else '🔴'}")
    ok &= perm_ok
    # 4) 预设与两者之差
    print("[4] 预设（URDF，单腿前三值 hip/thigh/calf）：")
    for name, (urdf, desc) in pre.items():
        print(f"    {name:9s} {np.round(urdf[:3], 4).tolist()}  ← {desc}")
    dv = pre["nominal"][0] - pre["default"][0]
    print(f"    ⚠ nominal − default = {np.round(dv[:3], 4).tolist()} rad "
          f"= {np.round(np.degrees(dv[:3]), 2).tolist()}°（calf 的差最大）")
    dz = pre["zero"][0] - pre["zero-kin"][0]
    print(f"    ⚠ zero − zero-kin  = {np.round(np.rad2deg(dz[:3]), 2).tolist()}°（= CONV_B 残差）")
    # 5) 校验零位姿态的 status 是否如定义
    st_zero = urdf_to_status(pre["zero"][0], cal)
    print(f"[5] zero 预设反解回 status（单腿）= {np.round(st_zero[:3], 4).tolist()} "
          f"（应 ≈ [0, 0, {math.pi/2:.4f}]）")
    ret = float(np.max(np.abs(st_zero[:3] - [0, 0, math.pi / 2])))
    print(f"    最大偏差 = {ret:.3e}  {'✅' if ret < 1e-9 else '🔴'}")
    ok &= ret < 1e-9
    print()
    # 6) 前向运动学：轮心是否在髋正下方 / 是否落到地面（用 MuJoCo 真算，不靠肉眼）
    z0 = cal["nominal"]["torso_z"]
    print(f"[6] FK 对比（机身 z = {z0} m；判据：轮心 Δx→0、轮心 z→轮半径）")
    sites = [f"{l.lower()}_wheel_site" for l in LEGS]
    bodies = [f"{l}_hip_link" for l in LEGS]
    print(f"    {'预设':10s} {'轮心−髋 Δx (mm)':>18s} {'轮心 z (mm)':>14s}")
    for name in ("default", "nominal"):
        set_pose(mujoco, m, d, adr, pre[name][0], None, z=z0)
        dxs, zs = [], []
        for sn, bn in zip(sites, bodies):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, sn)
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn)
            dxs.append((d.site_xpos[sid][0] - d.xpos[bid][0]) * 1000.0)
            zs.append(d.site_xpos[sid][2] * 1000.0)
        print(f"    {name:10s} {', '.join(f'{v:+7.1f}' for v in dxs)}   "
              f"{', '.join(f'{v:6.1f}' for v in zs)}")
    print("    参考：dogurdf.py 称 nominal 姿态使轮心在髋正下方（x 偏 1e-4 m）、机身 0.40 m")
    print()
    print("✅ 全部通过" if ok else "🔴 有失败项")
    return 0 if ok else 1


# =====================================================================
#  主循环
# =====================================================================
KEY_SPACE, KEY_Q, KEY_R, KEY_N = 32, 81, 82, 78
KEY_1, KEY_4 = 49, 52
KEY_0 = 48


# ---- IMU 四元数处理 ----
# ⚠ IMU（维特 HWT606）**无磁力计** ⇒ 偏航(yaw)基准是任意的：2026-10-06 数据集首帧
#   quat = [-0.9022, 0.0514, -0.0340, -0.4268]，换算下来是"绕 Z 约 51° 偏航 + 很小横滚/俯仰"。
#   直接用会让狗在窗口里朝随机方向站着、yaw 漂移时慢慢转 ⇒ 默认按"首帧为基准"去掉初始偏航。
def _qmul(a, b):
    """四元数乘法 a⊗b（w,x,y,z）。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _qinv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _drop_yaw(q):
    """去掉偏航，只留横滚/俯仰（把机体系 x 轴投到世界 XY 平面）。"""
    w, x, y, z = q
    yaw = math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))
    return _qmul([math.cos(-yaw / 2.0), 0.0, 0.0, math.sin(-yaw / 2.0)], q)


class ImuTransform:
    """把日志里的原始四元数按指定模式变成"给 MuJoCo 摆机身"用的四元数。

    mode: yaw0 = 以首帧为基准去掉初始偏航（默认，保留真实转向）
          full = 原样使用（会看到任意朝向 + 漂移）
          tilt = 彻底去掉偏航，只看横滚/俯仰
          none = 恒等（机身始终水平）
    """

    def __init__(self, mode: str, q0=None):
        self.mode = mode
        self.q0 = None if q0 is None else np.asarray(q0, dtype=float)

    def __call__(self, q):
        if self.mode == "none" or q is None:
            return None
        q = np.asarray(q, dtype=float)
        n = np.linalg.norm(q)
        if n < 1e-9:
            return None
        q = q / n
        if self.mode == "full":
            return q
        if self.mode == "tilt":
            return _drop_yaw(q)
        # yaw0
        if self.q0 is None:
            return q
        return _qmul(_qinv(self.q0), q)


def main() -> int:
    ap = argparse.ArgumentParser(description="关节姿态可视化（Python + MuJoCo）")
    ap.add_argument("--csv", help="回放日志：dataset_*.csv / recv_*.csv / pace chirp_*.csv")
    ap.add_argument("--udp", nargs="?", const=45555, type=int, default=None,
                    help="实时：监听 C++ 侧的姿态流（默认端口 45555；对方要 POSE_STREAM=1 ./run.sh）")
    ap.add_argument("--preset", default="default",
                    choices=["default", "nominal", "zero", "zero-kin"],
                    help="无 --csv 时显示的静态预设姿态")
    ap.add_argument("--z", type=float, default=None, help="机身高（默认取 NOMINAL_TORSO_HEIGHT）")
    ap.add_argument("--imu-mode", default="yaw0", choices=["yaw0", "full", "tilt", "none"],
                    help="IMU 四元数怎么用：yaw0=去掉首帧初始偏航(默认) / full=原样 / tilt=只留横滚俯仰 / none=忽略")
    ap.add_argument("--no-imu", action="store_true", help="已弃用别名，等价于 --imu-mode none")
    ap.add_argument("--speed", type=float, default=1.0, help="回放倍速")
    ap.add_argument("--show-mesh", action="store_true", help="显示 mesh（默认只画连杆骨架）")
    ap.add_argument("--selftest", action="store_true", help="只做坐标链路自检，不开窗口")
    ap.add_argument("--shot", help="离屏渲染一张 PNG 后退出（不开窗口；预设或 --shot-t 时刻的那一帧）")
    ap.add_argument("--shot-t", type=float, default=0.0, help="--shot 用 --csv 时取哪个时刻（秒）")
    ap.add_argument("--shot-size", default="800x600", help="--shot 出图尺寸 WxH，默认 800x600")
    ap.add_argument("--shot-wait", type=float, default=5.0, help="--shot --udp 时等第一帧的最长秒数")
    ap.add_argument("--cam-az", type=float, default=128.0, help="--shot 相机方位角（90/270=正侧视，0/180=正视）")
    ap.add_argument("--cam-el", type=float, default=-14.0, help="--shot 相机俯仰角")
    ap.add_argument("--cam-dist", type=float, default=1.85, help="--shot 相机距离 (m)")
    a = ap.parse_args()
    if a.no_imu:                      # 旧参数：等价于 --imu-mode none
        a.imu_mode = "none"

    cal = load_calib()
    pre = build_presets(cal)
    if a.z is None:
        a.z = cal["nominal"]["torso_z"]

    if a.selftest:
        return selftest(pre, cal)

    import mujoco
    import mujoco.viewer

    m, adr = load_model()
    if not a.show_mesh:
        make_skeleton(m)
    d = mujoco.MjData(m)

    src = load_csv(a.csv, cal) if a.csv else None
    udp = UdpPoseSource(a.udp) if a.udp else None
    if udp is not None:
        print(f"[pose_view] 实时模式：监听 udp://0.0.0.0:{a.udp}（等 C++ 侧的姿态流…）")
        print("            C++ 侧启动方式：POSE_STREAM=1 ./run.sh")
        print("            若一直无包：确认对方设了 POSE_STREAM=1、端口一致、防火墙未拦本机 UDP。")
    q0 = None
    if src is not None and src.quat is not None:
        for q in src.quat:                      # 首帧有效四元数作为偏航基准
            if np.linalg.norm(q) > 0.5:
                q0 = q
                break
    imu = ImuTransform(a.imu_mode, q0)
    if src is not None:
        extra = "" if src.quat is None else f"，IMU 基准 quat={np.round(q0, 4).tolist() if q0 is not None else '无'}"
        print(f"[pose_view] 载入 {src.label}：{len(src)} 帧，"
              f"{src.t[0]/1000:.2f}s ~ {src.t[-1]/1000:.2f}s，坐标层 = {src.layer}{extra}")
        print(f"[pose_view] IMU 模式 = {a.imu_mode}")
    else:
        print(f"[pose_view] 静态预设 = {a.preset}（{pre[a.preset][1]}）")

    # ---- 只出图：离屏渲染一张后退出（不开窗口）----
    if a.shot:
        if udp is not None:                 # 实时出图：等第一帧到了再渲染
            print(f"[pose_view] 等待 udp://0.0.0.0:{a.udp} 的帧（最多 {a.shot_wait:g}s）…")
            urdf = quat = None
            deadline = time.time() + a.shot_wait
            while time.time() < deadline:
                u, _t, q, _g, _seq, _l, _r, _tl = udp.latest()
                if u is not None:
                    urdf, quat = u, imu(q)
                    break
                time.sleep(0.05)
            if urdf is None:
                print("🔴 没等到姿态流（确认对方设了 POSE_STREAM=1、端口一致）⇒ 改用预设")
                urdf, quat = pre[a.preset][0], None
        elif src is not None:
            urdf, quat = src.at(a.shot_t * 1000.0)
            quat = imu(quat)
            urdf, quat = src.at(a.shot_t * 1000.0)
            quat = imu(quat)
        else:
            urdf, quat = pre[a.preset][0], None
        render_shot(mujoco, m, d, adr, urdf, quat, a.z, a.shot,
                    a.shot_size, show_mesh=a.show_mesh,
                    azimuth=a.cam_az, elevation=a.cam_el, distance=a.cam_dist)
        return 0

    state = {
        "playing": True,
        "t": 0.0,                      # 当前回放时间 (ms)
        "preset": a.preset,
        "show_body": True,
    }
    pset_names = ["default", "nominal", "zero", "zero-kin"]

    def key_cb(keycode):
        if keycode == KEY_SPACE:
            state["playing"] = not state["playing"]
        elif keycode == KEY_N:
            state["playing"] = False
            if src is not None:
                state["t"] = min(src.t[-1], state["t"] + 50.0)
        elif keycode == KEY_R:
            state["t"] = 0.0
        elif keycode == KEY_0:
            state["show_body"] = not state["show_body"]
        elif KEY_1 <= keycode <= KEY_4:
            state["preset"] = pset_names[keycode - KEY_1]
        elif keycode == KEY_Q:
            raise KeyboardInterrupt

    print("按键：空格=暂停  N=单步  R=回起点  1..4=预设  0=隐藏机身  Q=退出")

    # 骨架：关掉 mesh（group 1），打开碰撞体（group 3）
    last_print = 0.0
    with mujoco.viewer.launch_passive(m, d, key_callback=key_cb) as viewer:
        try:
            viewer.opt.geomgroup[1] = 0 if not a.show_mesh else 1
            viewer.opt.geomgroup[3] = 1
            viewer.opt.geomgroup[0] = 1
        except Exception as e:                                # noqa: BLE001
            print(f"[WARN] 无法设置 geomgroup（{e}），将显示完整 mesh")

        t_prev = time.time()
        while viewer.is_running():
            now = time.time()
            wall_ms = (now - t_prev) * 1000.0
            t_prev = now

            if udp is not None:
                u_urdf, u_tgt, u_quat, u_gyro, seq, lost, rate, t_last = udp.latest()
                fresh = (t_last > 0) and (time.time() - t_last < 0.5)
                if fresh and u_urdf is not None:
                    urdf = u_urdf
                    quat = imu(u_quat)
                    label = (f"实时 seq={seq}  {rate:5.1f} 包/s  丢 {lost}"
                             + ("" if u_quat is not None else "  (无 IMU)"))
                else:
                    urdf, quat = pre[state["preset"]][0], None
                    label = (f"⏳ 等待姿态流…（已收 {seq + 1} 包）—— 暂显预设 {state['preset']}"
                             if seq >= 0 else "⏳ 等待姿态流…（一包未收）—— 暂显预设")
            elif src is None:
                urdf, quat = pre[state["preset"]][0], None
                label = f"预设 {state['preset']}"
            else:
                if state["playing"]:
                    state["t"] += wall_ms * a.speed
                    if state["t"] > src.t[-1]:
                        state["t"] = src.t[-1]
                urdf, quat = src.at(state["t"])
                quat = imu(quat)
                label = (f"{src.label}  t={state['t']/1000:7.2f}s  "
                         f"{'▶' if state['playing'] else '⏸'} x{a.speed:g}")

            set_pose(mujoco, m, d, adr, urdf, quat, z=a.z,
                     show_body=state["show_body"])
            viewer.sync()

            if now - last_print > 1.0:
                last_print = now
                tail = "  ".join(f"{POLICY_NAMES[k][:4]}{k}={math.degrees(urdf[k]):6.1f}°"
                                 for k in (1, 2, 4, 5))     # FL/FR 的 thigh+calf
                print(f"  {label}  {tail}")

            time.sleep(1.0 / 60.0)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[pose_view] 退出")
        sys.exit(0)
