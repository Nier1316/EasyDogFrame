#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
delay_fit.py —— 从阶跃/脉冲数据估计「端到端纯延迟 T_d」与「一阶时间常数 τ」

用法:
    python3 tool/delay_fit.py <dataset.csv> [--motor 1] [--auto]
                              [--signal auto|pos|vel|tau] [--policy-order]
                              [--csv out.csv] [--quiet]
    python3 tool/delay_fit.py --selftest [--tmpdir /tmp]

参数:
    <dataset.csv>   统一数据集 CSV（见下方 schema，188 列，500Hz）
    --motor N       CAN 关节索引 i = can_port*4 + (motor_id-1)，0..15，默认 1 (=FL-thigh)。
                    给出 --policy-order 时 N 按 POLICY 序解释（12 腿 + 4 轮，见 rl_controller.h）。
    --auto          自动扫描全部 16 个关节，挑「阶跃沿最多」的通道做详细辨识，其余给汇总表。
    --signal S      指定激励/反馈通道：auto(按行内 c_mode 自动选) / pos(c_pos→m_pos) /
                    vel(c_vel→m_vel) / tau(c_tau→m_tau)。
    --csv PATH      把每段明细表落盘为 CSV。
    --quiet         只输出汇总行。
    --selftest      生成合成数据集（真实 T_d=18ms, τ=40ms）到 --tmpdir，跑一遍并校验 ±5ms。

估计方法（对每一段阶跃沿分别计算，最后取中位数/标准差）:
    1) 互相关法：段内 c 与 m 各自去均值后按 lag 归一化互相关，取峰值 lag；峰值做抛物线
       亚采样插值。lag>0 表示 m 滞后 c，即 T_d。
    2) 首次越阈法：以阶跃前 m 的 median ± 3·(1.4826·MAD) 为阈值（符号随阶跃方向），
       找 m 首次越阈时刻 − c 阶跃时刻。
    3) 一阶+延迟拟合：m(t) = m0 + K·Δc·(1 − exp(−(t−T_d)/τ))·u(t−T_d)。
       对 (T_d, τ) 网格搜索：固定 τ 时模型对 (a,b) 线性（y = a − b·exp(−t/τ)），
       用最小二乘解析求解，取残差最小者；有 scipy 时再用 curve_fit 精修。
    分辨率说明：采样 2ms → 单点延迟分辨率 2ms，所有 T_d 都受 ±dt 离散化限制
    （互相关峰值已做抛物线插值给出亚采样值，但不代表真实精度优于 ±2ms）。

⚠ 坐标约定：c_* / m_* 都是「标定后统一坐标」（GetStatus 返回、SendImpedance/SendSpeed
   接受的坐标），离线拟合不要再乘/除任何标定系数。

================= 冻结数据集 schema（188 列，UTF-8，逗号分隔，\\n 行尾）=================
 1. wall_ms                     系统墙钟 epoch 毫秒(int)，与 sim2sim --record / log/rl_*.csv 可直接对齐
 2. t_ms                        相对录制开始毫秒(float，3 位小数，单调递增)
 3. c_mode_00 .. c_mode_15      int：0=IMPEDANCE 1=SPEED 2=POSITION
 4. c_pos_00 .. c_pos_15        指令(按该行 c_mode_i 解释)
 5. c_vel_00 .. c_vel_15
 6. c_kp_00  .. c_kp_15
 7. c_kd_00  .. c_kd_15
 8. c_tau_00 .. c_tau_15
 9. m_pos_00 .. m_pos_15        反馈位置
10. m_vel_00 .. m_vel_15        反馈速度
11. m_tau_00 .. m_tau_15        反馈力矩
12. m_temp_00 .. m_temp_15      电机温度 °C（1Hz 轮询全部 16 个；未轮询到 = 0）
13. m_vbus_00 .. m_vbus_15      母线电压 V（1Hz 只轮询每路 CAN 的 1 号电机 i%4==0；其余 0/保持）
14. gyro_0, gyro_1, gyro_2      机体系角速度 rad/s
15. quat_w, quat_x, quat_y, quat_z   body←world，w 在前
16. cmd_vx, cmd_vy, cmd_wz      上层速度命令（站立即 0）

c_* 语义（按行内 c_mode_i 解释，关键）:
    mode=0 IMPEDANCE：c_pos=目标位置, c_vel=目标速度, c_kp=kp, c_kd=kd, c_tau=τ_ff
    mode=1 SPEED    ：c_pos=0,        c_vel=目标速度, c_kp=kvp, c_kd=0,  c_tau=ki
    mode=2 POSITION ：c_pos=目标位置, c_vel=0,        c_kp=kvp, c_kd=kp, c_tau=kvi

电机索引约定（CAN 顺序，非 POLICY 顺序）:
    i = can_port*4 + (motor_id-1)
    00=FL-hip 01=FL-thigh 02=FL-calf 03=FL-wheel
    04=FR-hip 05=FR-thigh 06=FR-calf 07=FR-wheel
    08=RL-hip 09=RL-thigh 10=RL-calf 11=RL-wheel
    12=RR-hip 13=RR-thigh 14=RR-calf 15=RR-wheel
    CAN→POLICY: POLICY_TO_MJX = [0,1,2,12, 3,4,5,13, 6,7,8,14, 9,10,11,15]

同目录同名 .meta.txt 为 key=value 文本（note / start_ts / freq=500Hz / motor_order=can /
imu=Z_DOWN_X / dropped=<丢帧计数> / weight=<权重文件名>）。

本模块同时是 wheel_servo_fit.py 的底层库（load_dataset / detect_steps / 各延迟估计函数）。
"""
import argparse
import csv
import math
import os
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.stderr.write("[ERROR] 需要 numpy：pip install numpy（本工具只用 stdlib + numpy）\n")
    sys.exit(2)

# ---------------------------------------------------------------------------
# schema
# ---------------------------------------------------------------------------
_BLOCK16 = ("c_mode", "c_pos", "c_vel", "c_kp", "c_kd", "c_tau",
            "m_pos", "m_vel", "m_tau", "m_temp", "m_vbus")
_SCALARS = ("wall_ms", "t_ms", "gyro_0", "gyro_1", "gyro_2",
            "quat_w", "quat_x", "quat_y", "quat_z", "cmd_vx", "cmd_vy", "cmd_wz")

# CAN <-> POLICY 关节索引置换（见 include/strategy/rl_controller.h / rl_controller.cpp）:
#   pos_policy[i] = pos_can[MJX_TO_POLICY[i]]  → MJX_TO_POLICY 是按 POLICY 索引、给出 CAN
#   can[mjx] ↔ policy[POLICY_TO_MJX[mjx]]      → POLICY_TO_MJX 是按 CAN 索引、给出 POLICY
# 两者互为逆置换，命名与直觉相反，这里按「语义」重新命名避免踩坑。
POLICY_TO_CAN = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15]   # = MJX_TO_POLICY
CAN_TO_POLICY = [0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15]   # = POLICY_TO_MJX
_LEG = ("FL", "FR", "RL", "RR")
_PART = ("hip", "thigh", "calf", "wheel")


def joint_name(i):
    """CAN 关节索引 -> 'FL-hip(can00)' 之类的中文可读名。"""
    return "%s-%s(can%02d)" % (_LEG[i // 4], _PART[i % 4], i)


def policy_name(i):
    """POLICY 序 0..11 = 12 腿关节(FL/FR/RL/RR × hip/thigh/calf)，12..15 = 4 轮。"""
    if i < 12:
        return "%s-%s(policy%02d)" % (_LEG[i // 3], _PART[i % 3], i)
    return "%s-wheel(policy%02d)" % (_LEG[i - 12], i)


def expected_header():
    """按冻结 schema 生成 188 列表头。"""
    h = ["wall_ms", "t_ms"]
    for base in _BLOCK16:
        h += ["%s_%02d" % (base, i) for i in range(16)]
    h += ["gyro_0", "gyro_1", "gyro_2",
          "quat_w", "quat_x", "quat_y", "quat_z",
          "cmd_vx", "cmd_vy", "cmd_wz"]
    return h


EXPECTED_HEADER = expected_header()
EXPECTED_NCOL = len(EXPECTED_HEADER)          # 188
REQUIRED_COLS = (["wall_ms", "t_ms"] +
                 ["c_mode_%02d" % i for i in range(16)] +
                 ["c_pos_%02d" % i for i in range(16)] +
                 ["c_vel_%02d" % i for i in range(16)] +
                 ["m_pos_%02d" % i for i in range(16)] +
                 ["m_vel_%02d" % i for i in range(16)] +
                 ["m_tau_%02d" % i for i in range(16)])


class DatasetError(Exception):
    """数据集体检类错误（中文消息，绝不抛 traceback）。"""


class Dataset(object):
    """列式数据集：ds.cols[name] -> np.ndarray(float)，长度 n。"""

    def __init__(self, path, header, cols, n_rows, n_bad_rows, n_mismatch, warnings):
        self.path = path
        self.header = header
        self.cols = cols
        self.n = n_rows
        self.n_bad_rows = n_bad_rows        # 单元格非数值(->NaN) 的行数
        self.n_mismatch = n_mismatch        # 列数与表头不符被跳过的行数
        self.warnings = warnings

    def has(self, name):
        return name in self.cols

    def get(self, name):
        return self.cols.get(name)

    @property
    def t(self):
        return self.cols["t_ms"]

    @property
    def wall(self):
        return self.cols.get("wall_ms")

    def dt_ms(self):
        """中位采样间隔(ms)；不足两点返回 2.0。"""
        if self.n < 2:
            return 2.0
        d = np.diff(self.t)
        d = d[np.isfinite(d)]
        if d.size == 0:
            return 2.0
        med = float(np.median(d))
        return med if med > 0 else 2.0


def load_dataset(path, required=None, check_schema=True):
    """读统一数据集 CSV。任何异常都抛 DatasetError（中文消息）。"""
    if not os.path.exists(path):
        raise DatasetError("文件不存在: %s" % path)
    if os.path.isdir(path):
        raise DatasetError("这是目录不是文件: %s" % path)
    if os.path.getsize(path) == 0:
        raise DatasetError("文件为空(0 字节): %s" % path)

    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        rd = csv.reader(f)
        try:
            header = next(rd)
        except StopIteration:
            raise DatasetError("文件没有表头: %s" % path)
        header = [h.strip() for h in header]
        ncol = len(header)
        warnings = []

        if check_schema and ncol != EXPECTED_NCOL:
            raise DatasetError(
                "列数不符: 期望 %d 列(冻结 schema)，实际 %d 列。\n"
                "  前 8 列表头: %s\n"
                "  请确认录制器按冻结 schema 输出（见本文件 docstring）。"
                % (EXPECTED_NCOL, ncol, ",".join(header[:8])))

        raw = []
        n_mismatch = 0
        for line_no, row in enumerate(rd, start=2):
            if not row:
                continue
            if len(row) != ncol:
                n_mismatch += 1
                if n_mismatch <= 3:
                    warnings.append("第 %d 行字段数 %d != 表头 %d，已跳过" % (line_no, len(row), ncol))
                continue
            raw.append(row)

    if not raw:
        raise DatasetError("没有任何可用数据行: %s" % path)

    # 整体转 float；失败再逐元素（容忍个别脏格 -> NaN）
    n_rows = len(raw)
    n_bad = 0
    try:
        arr = np.asarray(raw, dtype=float)
    except (ValueError, TypeError):
        arr = np.full((n_rows, ncol), np.nan, dtype=float)
        bad_rows = set()
        for j in range(ncol):
            for k in range(n_rows):
                v = raw[k][j]
                try:
                    arr[k, j] = float(v)
                except (TypeError, ValueError):
                    bad_rows.add(k)
        n_bad = len(bad_rows)
        warnings.append("有 %d 行存在非数值单元格（已置 NaN）" % n_bad)

    cols = {header[j]: arr[:, j] for j in range(ncol)}

    if check_schema:
        missing = [n for n in EXPECTED_HEADER if n not in cols]
        extra = [n for n in header if n not in EXPECTED_HEADER]
        if missing:
            warnings.append("缺少 %d 列: %s" % (len(missing), ",".join(missing[:8])))
            for n in missing:                      # 全 NaN 占位，保证下游不 KeyError
                cols[n] = np.full(n_rows, np.nan)
        if extra:
            warnings.append("存在 %d 个未知列: %s" % (len(extra), ",".join(extra[:8])))
        req = required if required is not None else REQUIRED_COLS
        lack = [n for n in req if n not in header]
        if lack:
            raise DatasetError("缺少必需列 %d 个: %s\n  该文件可能不是冻结 schema 的统一数据集。"
                               % (len(lack), ",".join(lack[:10])))
    else:
        for n in ("wall_ms", "t_ms"):
            if n not in cols:
                raise DatasetError("缺少必需列: %s" % n)

    return Dataset(path, header, cols, n_rows, n_bad, n_mismatch, warnings)


# ---------------------------------------------------------------------------
# 通道选择 / 分段
# ---------------------------------------------------------------------------
def mode_of(ds, i, row=0):
    """取第 i 个关节在 row 行的 c_mode（0/1/2）。"""
    col = ds.get("c_mode_%02d" % i)
    if col is None or row >= ds.n or not np.isfinite(col[row]):
        return 0
    return int(round(float(col[row])))


def pick_signal(ds, i, signal="auto"):
    """返回 (cmd_col, fb_col, 说明)。signal: auto/pos/vel/tau。"""
    if signal == "pos":
        return "c_pos_%02d" % i, "m_pos_%02d" % i, "位置 c_pos→m_pos"
    if signal == "vel":
        return "c_vel_%02d" % i, "m_vel_%02d" % i, "速度 c_vel→m_vel"
    if signal == "tau":
        return "c_tau_%02d" % i, "m_tau_%02d" % i, "力矩 c_tau→m_tau"
    modes = ds.get("c_mode_%02d" % i)
    m = int(round(float(np.nanmedian(modes)))) if modes is not None else 0
    if m == 1:
        return "c_vel_%02d" % i, "m_vel_%02d" % i, "SPEED(mode=1) c_vel→m_vel"
    if m == 2:
        return "c_pos_%02d" % i, "m_pos_%02d" % i, "POSITION(mode=2) c_pos→m_pos"
    return "c_pos_%02d" % i, "m_pos_%02d" % i, "IMPEDANCE(mode=0) c_pos→m_pos"


def post_limit_ms(ds, i0, post_ms, dt_ms):
    """从 i0 起，遇到采样缺口(>2.5dt)就截断 post 窗口，返回可用毫秒数。"""
    t = ds.t
    n = ds.n
    j = i0 + 1
    limit = post_ms
    while j < n and t[j] - t[i0] <= post_ms:
        if t[j] - t[j - 1] > 2.5 * dt_ms:
            limit = float(t[j - 1] - t[i0])
            break
        j += 1
    return limit


def detect_steps(t, c, dt_ms, min_sigma=3.0, pre_ms=100.0, post_ms=400.0,
                 max_events=64, min_dwell_ms=60.0):
    """在命令通道 c 上找阶跃沿。

    判据：|diff(c)| > max(3·(1.4826·MAD(diff)), 2%·动态范围) 且新电平保持。
    返回 [{'idx','t_step','before','after','dc','post_ms'}, ...]
    post_ms 会在「下一个阶跃太近」时截断，可用后段 < 0.5·post_ms 的段直接丢弃。
    """
    t = np.asarray(t, dtype=float)
    c = np.asarray(c, dtype=float)
    n = len(c)
    if n < 8 or dt_ms <= 0:
        return []
    pre_n = int(round(pre_ms / dt_ms))
    post_n = int(round(post_ms / dt_ms))
    if n < pre_n + post_n // 2:
        return []

    finite = np.isfinite(c)
    if finite.sum() < max(8, n // 2):
        return []
    if not finite.all():
        c = np.where(finite, c, np.nanmedian(c))

    d = np.diff(c)
    mad = float(np.median(np.abs(d - np.median(d))))
    sigma = 1.4826 * mad
    rng = float(np.percentile(c, 99) - np.percentile(c, 1))
    thr = max(min_sigma * sigma, 0.02 * rng, 1e-12)
    cand = np.where(np.abs(d) > thr)[0]
    if cand.size == 0:
        return []

    gap = max(2, int(round(min_dwell_ms / dt_ms * 0.3)))
    groups = []
    cur = [int(cand[0])]
    for k in cand[1:]:
        if k - cur[-1] <= gap:
            cur.append(int(k))
        else:
            groups.append(cur)
            cur = [int(k)]
    groups.append(cur)

    events = []
    for g in groups:
        s, e = g[0], g[-1]
        if s < pre_n or e + 2 >= n:
            continue
        before = float(np.median(c[s - pre_n:s]))
        after = float(np.median(c[e + 1:min(n, e + 1 + max(3, pre_n // 2))]))
        dc = after - before
        if abs(dc) < thr:
            continue
        events.append({"idx": s, "t_step": float(t[s]), "before": before,
                       "after": after, "dc": dc, "post_ms": post_ms})

    out = []
    for k, ev in enumerate(events):
        nxt = events[k + 1]["idx"] if k + 1 < len(events) else n
        avail = float(t[min(nxt, n - 1)] - t[ev["idx"]])
        if avail < 0.5 * post_ms:
            continue                     # 后段被下一个阶跃污染
        out.append(ev)
        if len(out) >= max_events:
            break
    return out


# ---------------------------------------------------------------------------
# 三种延迟估计
# ---------------------------------------------------------------------------
def edge_xcorr_delay(t, c, m, i0, dt_ms, pre_ms=100.0, post_ms=400.0, max_lag_ms=200.0):
    """边沿(微分)互相关：对 Δc 与 Δm 做互相关取峰值。阶跃激励下无偏，是互相关主值。

    说明：对「阶跃 + 一阶滞后」响应，直接用原始信号去均值互相关（见 xcorr_delay）
    峰值会被指数尾巴拉偏约 +0.7τ；改用微分（边沿）后，延迟环节表现为孤立脉冲，
    峰值即纯延迟。
    """
    t = np.asarray(t, float)
    c = np.asarray(c, float)
    m = np.asarray(m, float)
    pre_n = int(round(pre_ms / dt_ms))
    post_n = int(round(post_ms / dt_ms))
    lo = max(0, i0 - pre_n)
    hi = min(len(t), i0 + post_n)
    if hi - lo < 12:
        return None
    a = np.diff(c[lo:hi])
    b = np.diff(m[lo:hi])
    good = np.isfinite(a) & np.isfinite(b)
    if good.sum() < 10:
        return None
    a = a[good]
    b = b[good]
    a = a - a.mean()
    b = b - b.mean()
    if np.allclose(a, 0.0) or np.allclose(b, 0.0):
        return None
    n = len(a)
    max_lag = max(1, int(round(max_lag_ms / dt_ms)))
    lags = np.arange(-max_lag, max_lag + 1)
    rs = np.full(lags.size, np.nan, dtype=float)
    for k, lag in enumerate(lags):
        if lag >= 0:
            x, y = a[:n - lag], b[lag:]
        else:
            x, y = a[-lag:], b[:n + lag]
        if x.size < max(8, n // 4):
            continue
        rs[k] = float((x * y).sum()) / x.size
    if not np.isfinite(rs).any():
        return None
    j = int(np.nanargmax(rs))
    lag = int(lags[j])
    delta = 0.0
    if 0 < j < rs.size - 1 and np.isfinite(rs[j - 1]) and np.isfinite(rs[j + 1]):
        y0, y1, y2 = float(rs[j - 1]), float(rs[j]), float(rs[j + 1])
        den = y0 - 2.0 * y1 + y2
        if abs(den) > 1e-12:
            delta = float(np.clip(0.5 * (y0 - y2) / den, -1.0, 1.0))
    return {"td_ms": (lag + delta) * dt_ms, "lag_samples": lag,
            "peak_r": float(rs[j]), "interp": delta, "resolution_ms": dt_ms}


def xcorr_delay(t, c, m, i0, dt_ms, pre_ms=100.0, post_ms=400.0, max_lag_ms=200.0):
    """段内去均值归一化互相关，峰值为 T_d（lag>0 = m 滞后 c）。带抛物线亚采样插值。

    ⚠ 对「阶跃 + 一阶滞后」数据，该峰值含 +≈0.7τ 的系统偏置（指数尾巴所致），
    仅作参考；阶跃数据的互相关主值请用 edge_xcorr_delay。
    """
    t = np.asarray(t, float)
    c = np.asarray(c, float)
    m = np.asarray(m, float)
    pre_n = int(round(pre_ms / dt_ms))
    post_n = int(round(post_ms / dt_ms))
    lo = max(0, i0 - pre_n)
    hi = min(len(t), i0 + post_n)
    a = c[lo:hi].copy()
    b = m[lo:hi].copy()
    good = np.isfinite(a) & np.isfinite(b)
    if good.sum() < 10:
        return None
    a = a[good]
    b = b[good]
    a -= a.mean()
    b -= b.mean()
    if np.allclose(a, 0.0) or np.allclose(b, 0.0):
        return None

    n = len(a)
    max_lag = max(1, int(round(max_lag_ms / dt_ms)))
    lags = np.arange(-max_lag, max_lag + 1)
    rs = np.full(lags.size, np.nan, dtype=float)
    for k, lag in enumerate(lags):
        if lag >= 0:
            x = a[:n - lag]
            y = b[lag:]
        else:
            x = a[-lag:]
            y = b[:n + lag]
        if x.size < max(8, n // 4):
            continue
        x = x - x.mean()
        y = y - y.mean()
        den = math.sqrt(float((x * x).sum()) * float((y * y).sum()))
        if den <= 0:
            continue
        rs[k] = float((x * y).sum()) / den
    if not np.isfinite(rs).any():
        return None
    j = int(np.nanargmax(rs))
    lag = int(lags[j])
    delta = 0.0
    if 0 < j < rs.size - 1 and np.isfinite(rs[j - 1]) and np.isfinite(rs[j + 1]):
        y0, y1, y2 = float(rs[j - 1]), float(rs[j]), float(rs[j + 1])
        den = y0 - 2.0 * y1 + y2
        if abs(den) > 1e-12:
            delta = float(np.clip(0.5 * (y0 - y2) / den, -1.0, 1.0))
    return {"td_ms": (lag + delta) * dt_ms, "lag_samples": lag,
            "peak_r": float(rs[j]), "interp": delta,
            "resolution_ms": dt_ms}


def threshold_delay(t, c, m, i0, dt_ms, pre_ms=100.0, post_ms=400.0, k=3.0):
    """首次越阈法：阶跃前 m 的 median±k·σ 作阈值，m 首次越阈 - c 阶跃时刻。"""
    t = np.asarray(t, float)
    c = np.asarray(c, float)
    m = np.asarray(m, float)
    pre_n = int(round(pre_ms / dt_ms))
    post_n = int(round(post_ms / dt_ms))
    lo = max(0, i0 - pre_n)
    pre = m[lo:i0]
    pre = pre[np.isfinite(pre)]
    if pre.size < 5:
        return None
    mu = float(np.median(pre))
    sd = 1.4826 * float(np.median(np.abs(pre - mu)))
    if not (sd > 0):
        sd = float(np.std(pre))
    if not (sd > 0):
        sd = 1e-9

    cb = c[lo:i0]
    ca = c[i0 + 1:min(len(c), i0 + 1 + max(3, post_n // 4))]
    if cb.size == 0 or ca.size == 0:
        return None
    dc = float(np.median(ca) - np.median(cb))
    if abs(dc) < 1e-12:
        return None
    sgn = 1.0 if dc > 0 else -1.0
    # 阶跃在 i0 与 i0+1 之间发生：c[i0]=旧电平, c[i0+1]=新电平（新电平从 t[i0+1] 起保持）
    t0 = float(t[i0]) + dt_ms
    hi = min(len(t), i0 + post_n)
    seg = (m[i0:hi] - mu) * sgn
    idx = np.where(np.isfinite(seg) & (seg > k * sd))[0]
    if idx.size == 0:
        return None
    j = i0 + int(idx[0])
    return {"td_ms": float(t[j] - t0), "idx": j, "mu": mu, "sd": sd, "dc": dc}


def _linfit_exp(tt, yy, tau):
    """y = a - b·exp(-tt/tau) 的最小二乘（闭式 2x2）。返回 (a,b,rms)。"""
    e = np.exp(-tt / tau)
    X = np.column_stack([np.ones_like(tt), -e])
    try:
        beta, *_ = np.linalg.lstsq(X, yy, rcond=None)
    except np.linalg.LinAlgError:
        return None
    r = yy - X @ beta
    return float(beta[0]), float(beta[1]), float(np.sqrt(np.mean(r * r)))


def _fit_td_tau(tt, yy, m0, dc, tds, taus):
    """在 (Td, τ) 网格上最小化「全段(tt>=0)残差」。
    模型: t<Td 时 m=m0；t>=Td 时 m = a - b·exp(-(t-Td)/τ)，固定 τ 时对 (a,b) 线性。
    必须把 (0, Td) 的平段也计入残差，否则 Td 会被指数尾巴吸走（曾偏到 +66ms）。"""
    best = None
    for Td in tds:
        sel = tt >= Td
        if sel.sum() < 6:
            continue
        xs, ys = tt[sel], yy[sel]
        for tau in taus:
            r = _linfit_exp(xs - Td, ys, tau)
            if r is None:
                continue
            a, b, _ = r
            pred = np.where(tt >= Td, a - b * np.exp(-np.maximum(tt - Td, 0.0) / tau), m0)
            rms = float(np.sqrt(np.mean((yy - pred) ** 2)))
            if best is None or rms < best[0]:
                best = (rms, float(Td), float(tau), a, b)
    return best


def fit_first_order_delay(t, c, m, i0, dt_ms, pre_ms=100.0, post_ms=400.0,
                          td_center=None, td_half_ms=40.0, max_tau_ms=600.0,
                          use_scipy=True):
    """一阶+纯延迟拟合。返回 dict(td_ms, tau_ms, K, m0, rms, method)。"""
    t = np.asarray(t, float)
    c = np.asarray(c, float)
    m = np.asarray(m, float)
    pre_n = int(round(pre_ms / dt_ms))
    post_n = int(round(post_ms / dt_ms))
    lo = max(0, i0 - pre_n)
    hi = min(len(t), i0 + post_n)
    if hi - i0 < 8:
        return None
    t_step = float(t[i0]) + dt_ms      # 新电平自 t[i0+1] 起保持
    tt_all = t[lo:hi] - t_step
    yy_all = m[lo:hi]
    ok = np.isfinite(tt_all) & np.isfinite(yy_all)
    if ok.sum() < 8:
        return None
    tt_all, yy_all = tt_all[ok], yy_all[ok]
    tt = tt_all[tt_all >= 0.0]          # 残差只在阶跃之后评（前面是常数 m0）
    yy = yy_all[tt_all >= 0.0]
    if tt.size < 8:
        return None

    pre = m[lo:i0]
    pre = pre[np.isfinite(pre)]
    m0 = float(np.median(pre)) if pre.size else float(yy[0])
    cb = c[lo:i0]
    ca = c[i0 + 1:min(len(c), i0 + 1 + max(3, post_n // 4))]
    if cb.size == 0 or ca.size == 0:
        return None
    dc = float(np.median(ca) - np.median(cb))
    if abs(dc) < 1e-12:
        return None

    if td_center is None:
        td_center = 0.0
    td_lo = max(0.0, td_center - td_half_ms)
    td_hi = td_center + td_half_ms
    taus = np.geomspace(max(1.0, dt_ms * 0.5), max_tau_ms, 40)

    # 阶段 1：粗网格
    best = _fit_td_tau(tt, yy, m0, dc,
                       np.arange(td_lo, td_hi + 1e-9, max(2.0, dt_ms)),
                       taus)
    if best is None:
        return None
    # 阶段 2：在粗解附近细化 Td/τ
    _, td0, tau0, _, _ = best
    fine_tds = np.arange(max(0.0, td0 - 2.0), td0 + 2.0 + 1e-9, 0.25)
    fine_taus = np.geomspace(max(0.5, tau0 / 1.4), tau0 * 1.4, 25)
    best2 = _fit_td_tau(tt, yy, m0, dc, fine_tds, fine_taus)
    if best2 is not None and best2[0] <= best[0]:
        best = best2
    rms, td, tau, a, b = best
    method = "grid"

    if use_scipy:
        try:
            from scipy.optimize import curve_fit

            def model(x, Td, tau_, a_, b_):
                x = np.asarray(x, float)
                return np.where(x >= Td,
                                a_ - b_ * np.exp(-np.maximum(x - Td, 0.0) / max(tau_, 1e-9)),
                                m0)

            p0 = [td, tau, a, b]
            bounds = ([0.0, max(0.2, dt_ms * 0.1), -1e12, -1e12],
                      [max(td * 4.0 + 50.0, 300.0), max_tau_ms * 2.0, 1e12, 1e12])
            popt, _ = curve_fit(model, tt, yy, p0=p0, bounds=bounds, maxfev=6000)
            rms2 = float(np.sqrt(np.mean((yy - model(tt, *popt)) ** 2)))
            if rms2 <= rms * 1.02:
                td, tau, a, b = (float(popt[0]), float(popt[1]),
                                 float(popt[2]), float(popt[3]))
                rms = rms2
                method = "scipy"
        except Exception:
            pass

    K = (a - m0) / dc if abs(dc) > 0 else float("nan")
    return {"td_ms": td, "tau_ms": tau, "K": K, "m0": m0, "dc": dc,
            "rms": rms, "method": method, "t_step": t_step,
            "tau_at_ceiling": bool(tau >= 0.9 * max_tau_ms)}


def analyze_events(ds, i, cmd_col, fb_col, events, dt_ms,
                   pre_ms=100.0, post_ms=400.0, use_scipy=True):
    """对每个阶跃段跑三法，返回明细 list[dict]。"""
    c = ds.get(cmd_col)
    m = ds.get(fb_col)
    rows = []
    for ev in events:
        i0 = ev["idx"]
        post = ev.get("post_ms") or post_ms
        post = min(post, post_limit_ms(ds, i0, post, dt_ms))
        if post < 0.5 * post_ms:
            continue
        xc = xcorr_delay(ds.t, c, m, i0, dt_ms, pre_ms, post)
        ed = edge_xcorr_delay(ds.t, c, m, i0, dt_ms, pre_ms, post)
        th = threshold_delay(ds.t, c, m, i0, dt_ms, pre_ms, post)
        td_guess = ed["td_ms"] if ed else (th["td_ms"] if th else 0.0)
        fo = fit_first_order_delay(ds.t, c, m, i0, dt_ms, pre_ms, post,
                                   td_center=max(0.0, td_guess), td_half_ms=40.0,
                                   use_scipy=use_scipy)
        # 段内背景运动：阶跃前 m 去线性趋势后的 std / |K·Δc|，用于判断 τ 是否可信
        pre = m[max(0, i0 - int(round(pre_ms / dt_ms))):i0]
        pre = pre[np.isfinite(pre)]
        pre_mu = float(np.median(pre)) if pre.size else float("nan")
        pre_sd = (1.4826 * float(np.median(np.abs(pre - pre_mu)))
                  if pre.size >= 8 else float("nan"))
        bg = float("nan")
        if pre.size >= 8 and fo is not None and np.isfinite(fo["K"]) and abs(fo["K"] * ev["dc"]) > 0:
            tp = np.arange(pre.size, dtype=float) * dt_ms
            coef = np.polyfit(tp, pre, 1)
            resid = pre - np.polyval(coef, tp)
            bg = float(np.std(resid) / abs(fo["K"] * ev["dc"]))
        tau_ok = bool(fo is not None and not fo["tau_at_ceiling"])
        # 稳态反馈值（供静态增益 / 死区 / 饱和统计）：τ 可信时从 t0+Td+3τ 起，否则取后 50%
        if fo is not None and tau_ok:
            ts = float(ds.t[i0]) + dt_ms + fo["td_ms"] + 3.0 * fo["tau_ms"]
        else:
            ts = float(ds.t[i0]) + 0.5 * post
        hi_w = min(ds.n, i0 + int(round(post / dt_ms)))
        w_ss = float("nan")
        if hi_w > i0:
            seg_w = np.arange(i0, hi_w)
            mask = ds.t[seg_w] >= ts
            if int(mask.sum()) >= 3:
                w_ss = float(np.median(m[seg_w[mask]]))
        # 响应显著性：稳态变化量必须超过阶跃前噪声的 3σ，否则该段对 τ/T_d 无意义
        # （典型：SPEED 死区内的小指令、IMPEDANCE 被限位卡住的段）
        resp_ok = bool(np.isfinite(w_ss) and np.isfinite(pre_mu) and
                       (not (pre_sd > 0) or abs(w_ss - pre_mu) > 3.0 * pre_sd))
        resp_ratio = (abs(w_ss - pre_mu) / pre_sd
                      if (np.isfinite(w_ss) and np.isfinite(pre_sd) and pre_sd > 0)
                      else float("nan"))
        cand = [v for v in ((ed["td_ms"] if (ed and resp_ok) else None),
                            (th["td_ms"] if (th and resp_ok) else None),
                            (fo["td_ms"] if (fo and tau_ok and resp_ok) else None))
                if v is not None]
        rows.append({
            "idx": i0, "t_step": ev["t_step"], "dc": ev["dc"],
            "before": ev["before"], "after": ev["after"], "post_ms": post,
            "w_ss": w_ss, "resp_ok": resp_ok, "resp_ratio": resp_ratio,
            "pre_mu": pre_mu, "pre_sd": pre_sd,
            "td_edge": ed["td_ms"] if ed else float("nan"),
            "edge_r": ed["peak_r"] if ed else float("nan"),
            "td_xcorr": xc["td_ms"] if xc else float("nan"),
            "xc_r": xc["peak_r"] if xc else float("nan"),
            "td_thr": th["td_ms"] if th else float("nan"),
            "td_fit": fo["td_ms"] if fo else float("nan"),
            "tau_ms": fo["tau_ms"] if fo else float("nan"),
            "tau_ok": tau_ok,
            "bg": bg,
            "K": fo["K"] if fo else float("nan"),
            "rms": fo["rms"] if fo else float("nan"),
            "method": fo["method"] if fo else "-",
            "td_med": float(np.median(cand)) if cand else float("nan"),
        })
    return rows


def _ms(vals):
    """返回 (中位数, 标准差(样本, n>1), n)；全 NaN 时 (nan, nan, 0)。"""
    v = np.asarray([x for x in vals], dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return float("nan"), float("nan"), 0
    med = float(np.median(v))
    sd = float(np.std(v, ddof=1)) if v.size > 1 else 0.0
    return med, sd, int(v.size)


def summarize(rows):
    """把明细行汇总成字典。

    只统计「响应显著(resp_ok)」的段；τ/一阶拟合另外要求「未撞 τ 网格上限(tau_ok)」。
    若全部段都被滤掉，退回全部段以免汇总为空。
    """
    if not rows:
        return None
    good = [r for r in rows if r.get("resp_ok")]
    fit_rows = [r for r in good if r.get("tau_ok")] or \
               [r for r in rows if r.get("tau_ok")]
    base = good or rows
    s = {}
    for key, name, src in (("td_edge", "edge", base), ("td_xcorr", "xcorr", base),
                           ("td_thr", "thr", base), ("td_fit", "fit", fit_rows),
                           ("td_med", "all", base)):
        med, sd, n = _ms([r[key] for r in src])
        s[name] = (med, sd, n)
    med, sd, n = _ms([r["tau_ms"] for r in fit_rows])
    s["tau"] = (med, sd, n)
    s["n_tau_ok"] = len(fit_rows)
    s["n_resp_ok"] = len(good)
    s["n"] = len(rows)
    bgs = [r["bg"] for r in rows if np.isfinite(r.get("bg", float("nan")))]
    s["bg_max"] = float(np.max(bgs)) if bgs else float("nan")
    # 推荐值：一阶+纯延迟拟合（唯一亚采样估计，且与 τ 联合辨识）；
    # τ 不可辨识(撞网格上限)或拟合缺失时退回「综合中位数」。
    if s["n_tau_ok"] > 0 and s["fit"][2] > 0:
        s["rec"] = s["fit"]
        s["rec_why"] = "一阶+纯延迟拟合（亚采样，与 τ 联合）"
    else:
        s["rec"] = s["all"]
        s["rec_why"] = "综合中位数（各段取中位），拟合不可用"
    return s


def fmt_ms(t):
    med, sd, n = t
    if n == 0 or not np.isfinite(med):
        return "n/a"
    return "%.1f ± %.1f ms (n=%d)" % (med, sd, n)


# ---------------------------------------------------------------------------
# 合成数据集（自测用；数据写到 /tmp，不留在工程里）
# ---------------------------------------------------------------------------
def build_synthetic_dataset(duration_s=9.0, fs=500.0, td_ms=18.0, tau_ms=40.0,
                            vbus=48.0, temp=36.0, seed=7, drops=3,
                            deadzone=0.4, wmax=25.0):
    """构造与冻结 schema 完全一致的 188 列假数据。

    含: 已知纯延迟 td_ms、一阶时间常数 tau_ms；轮子 SPEED 阶跃序列（含死区/饱和）、
    腿 IMPEDANCE 正弦+阶跃、关节 1 用 POSITION(mode=2)、若干丢帧、每路 CAN 只有 1 号
    电机有 temp/vbus。返回 (header, columns_dict, meta_lines)。
    """
    dt = 1000.0 / fs
    n = int(round(duration_s * fs))
    t = np.arange(n, dtype=float) * dt
    rng = np.random.default_rng(seed)
    header = expected_header()
    cols = {h: np.zeros(n, dtype=float) for h in header}

    def lagged(des):
        """对命令做纯延迟(np.interp) + 一阶离散仿真(精确离散化)。"""
        delayed = np.interp(t - td_ms, t, des, left=des[0], right=des[-1])
        a = math.exp(-dt / tau_ms)
        y = np.empty(n)
        y[0] = 0.0
        for k in range(n - 1):
            y[k + 1] = y[k] * a + delayed[k] * (1.0 - a)
        return y, delayed

    # ---- 轮子 SPEED 阶梯（死区 + 饱和）----
    levels = [0.0, 0.4, 1.2, 5.0, -5.0, 12.0, -12.0, 25.0, -25.0,
              35.0, -35.0, 8.0, -8.0, 0.0]
    dwell = max(0.7, duration_s / len(levels))
    des_all = np.zeros(n)
    for k in range(int(math.ceil(duration_s / dwell))):
        lvl = levels[k % len(levels)]
        s = int(round(k * dwell * fs))
        e = int(round(min(n, (k + 1) * dwell * fs)))
        des_all[s:e] = lvl
    eff = np.sign(des_all) * np.maximum(np.abs(des_all) - deadzone, 0.0)
    eff = np.clip(eff, -wmax, wmax)
    w_true, _ = lagged(eff)
    w_true += rng.normal(0.0, 0.02, n)

    # ---- 腿位置（正弦 + 阶跃），关节 1 用 mode=2 ----
    leg_steps = [(3.0, 0.20), (5.5, -0.25)]
    for i in range(16):
        tag = "%02d" % i
        if i % 4 == 3:                                  # 轮子
            cols["c_mode_%s" % tag][:] = 1
            cols["c_pos_%s" % tag][:] = 0.0
            cols["c_vel_%s" % tag][:] = des_all
            cols["c_kp_%s" % tag][:] = 3.0              # kvp
            cols["c_kd_%s" % tag][:] = 0.0
            cols["c_tau_%s" % tag][:] = 0.05            # ki
            cols["m_pos_%s" % tag][:] = 0.0
            cols["m_vel_%s" % tag][:] = w_true
            # 力矩: J·ω̇ + b·ω + 库仑摩擦
            J, b, fc = 0.010, 0.02, 0.15
            wdot = np.gradient(w_true, dt / 1000.0)
            cols["m_tau_%s" % tag][:] = (J * wdot + b * w_true
                                         + fc * np.tanh(w_true / 0.1)
                                         + rng.normal(0.0, 0.01, n))
            continue
        # 腿：慢漂移 + 阶跃（阶跃占主导，模拟真实「静止基线上打阶跃」的延迟测试）
        base = 0.05 * np.sin(2 * math.pi * 0.2 * t / 1000.0 + 0.3 * i)
        cmd = base.copy()
        for ts, dv in leg_steps:
            cmd[t >= ts * 1000.0] += dv
        pos, _ = lagged(cmd)
        pos += rng.normal(0.0, 0.0015, n)
        mode = 2 if i == 1 else 0
        cols["c_mode_%s" % tag][:] = mode
        cols["c_pos_%s" % tag][:] = cmd
        cols["c_vel_%s" % tag][:] = 0.0
        if mode == 2:                                    # POSITION: c_kp=kvp, c_kd=kp, c_tau=kvi
            cols["c_kp_%s" % tag][:] = 0.5
            cols["c_kd_%s" % tag][:] = 250.0
            cols["c_tau_%s" % tag][:] = 0.0
        else:                                            # IMPEDANCE
            cols["c_kp_%s" % tag][:] = 250.0
            cols["c_kd_%s" % tag][:] = 4.0
            cols["c_tau_%s" % tag][:] = 0.2
        cols["m_pos_%s" % tag][:] = pos
        cols["m_vel_%s" % tag][:] = np.gradient(pos, dt / 1000.0)
        cols["m_tau_%s" % tag][:] = (250.0 * (cmd - pos)
                                     - 4.0 * np.gradient(pos, dt / 1000.0)
                                     + rng.normal(0.0, 0.05, n))

    # ---- temp / vbus（与 s2r_dataset.cpp 的 PollSlowTelemetry 一致）----
    #   m_temp: 1Hz 轮询**全部 16 个**电机 → 16 路都非 0
    #   m_vbus: 1Hz 只轮询**每路 CAN 的 1 号电机**（i%4==0）→ 其余 12 路恒 0（或保持上次值）
    for i in range(16):
        tag = "%02d" % i
        cols["m_temp_%s" % tag][:] = temp + 2.0 * (i % 4) + rng.normal(0.0, 0.05, n)
        if i % 4 == 0:
            cols["m_vbus_%s" % tag][:] = vbus + rng.normal(0.0, 0.02, n)
        else:
            cols["m_vbus_%s" % tag][:] = 0.0

    # ---- IMU / 命令 ----
    cols["gyro_0"][:] = 0.01 * np.sin(2 * math.pi * 1.0 * t / 1000.0) + rng.normal(0, 0.002, n)
    cols["gyro_1"][:] = rng.normal(0, 0.002, n)
    cols["gyro_2"][:] = rng.normal(0, 0.002, n)
    roll = 0.02 * np.sin(2 * math.pi * 0.3 * t / 1000.0)
    cols["quat_w"][:] = 1.0
    cols["quat_x"][:] = roll
    cols["quat_y"][:] = 0.5 * roll
    cols["quat_z"][:] = 0.0
    for c in ("cmd_vx", "cmd_vy", "cmd_wz"):
        cols[c][:] = 0.0

    # ---- 时间轴 + 丢帧 ----
    keep = np.ones(n, dtype=bool)
    if drops > 0:
        for _ in range(drops):
            k0 = int(rng.integers(int(0.2 * n), max(int(0.2 * n) + 1, n - 20)))
            keep[k0:k0 + 4] = False                       # 8ms 缺口
    t_keep = t[keep]
    t_keep = t_keep - t_keep[0]
    start_epoch = 1750000000000
    cols = {k: v[keep] for k, v in cols.items()}
    cols["t_ms"] = t_keep
    cols["wall_ms"] = np.round(start_epoch + (t_keep + rng.normal(0, 0.3, t_keep.size))).astype(float)

    meta = ["note=synthetic dataset (delay_fit.py --selftest)",
            "start_ts=%d" % start_epoch,
            "freq=500Hz", "motor_order=can", "imu=Z_DOWN_X",
            "dropped=%d" % (n - int(keep.sum())),
            "weight=(synthetic)",
            "true_td_ms=%.3f" % td_ms, "true_tau_ms=%.3f" % tau_ms]
    return header, cols, meta


def write_synthetic_dataset(path, **kw):
    """写合成数据集 CSV + 同名 .meta.txt。返回 (path, meta_path, 真值dict)。"""
    header, cols, meta = build_synthetic_dataset(**kw)
    n = cols["t_ms"].size
    int_cols = set(["wall_ms"]) | set("c_mode_%02d" % i for i in range(16))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(header)
        for k in range(n):
            row = []
            for name in header:
                v = cols[name][k]
                if name in int_cols:
                    row.append("%d" % int(round(v)))
                elif name == "t_ms":
                    row.append("%.3f" % v)
                else:
                    row.append("%.6g" % v)
            w.writerow(row)
    meta_path = os.path.splitext(path)[0] + ".meta.txt"
    with open(meta_path, "w", encoding="utf-8") as f:
        f.write("\n".join(meta) + "\n")
    truth = {"td_ms": float(kw.get("td_ms", 18.0)), "tau_ms": float(kw.get("tau_ms", 40.0))}
    return path, meta_path, truth


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def print_joint_report(ds, i, cmd_col, fb_col, why, rows, summ, quiet=False):
    name = joint_name(i)
    if not quiet:
        print("\n---- 关节 %s  通道: %s (%s) ----" % (name, why, "%s → %s" % (cmd_col, fb_col)))
        print("  段  t_step(ms)      Δc    T_d_边沿  T_d_越阈  T_d_拟合  T_d_原始互相关  τ(ms)  K      fit")
        for k, r in enumerate(rows):
            print("  %3d %11.1f %8.4g %8.1f %9.1f %9.1f %13.1f %8.1f %6.3f  %s"
                  % (k, r["t_step"], r["dc"], r["td_edge"], r["td_thr"],
                     r["td_fit"], r["td_xcorr"], r["tau_ms"], r["K"], r["method"]))
    print("  [%s] 阶跃段 n=%d" % (name, summ["n"]))
    print("    T_d(推荐)        = %s   ← %s" % (fmt_ms(summ["rec"]), summ["rec_why"]))
    print("    T_d(互相关-边沿) = %s   ← 规范互相关法(微分边沿)，无偏但受 2ms 量化" % fmt_ms(summ["edge"]))
    print("    T_d(首次越阈)    = %s" % fmt_ms(summ["thr"]))
    print("    T_d(一阶拟合)    = %s" % fmt_ms(summ["fit"]))
    print("    T_d(综合中位)    = %s   ← 每段 {边沿,越阈,拟合} 中位数再取中位数" % fmt_ms(summ["all"]))
    print("    τ(一阶拟合)      = %s" % fmt_ms(summ["tau"]))
    if summ["n_tau_ok"] < summ["n"]:
        print("    [注意] %d/%d 段的 τ 撞到网格上限(600ms) → 该段阶跃响应不显著，"
              "τ/拟合 T_d 已从综合值中剔除" % (summ["n"] - summ["n_tau_ok"], summ["n"]))
    if summ.get("n_resp_ok", summ["n"]) < summ["n"]:
        print("    [注意] %d/%d 段反馈响应不显著（稳态变化 < 3σ 噪声，死区/卡住/未使能），"
              "已从统计中剔除" % (summ["n"] - summ["n_resp_ok"], summ["n"]))
    if summ.get("n_resp_ok", 1) == 0:
        print("    [警告] 所有段的反馈响应都不显著 → 本次辨识结果不可信，请检查电机使能/限位/死区")
    if np.isfinite(summ.get("bg_max", float("nan"))) and summ["bg_max"] > 0.25:
        print("    [注意] 段内背景运动(去趋势 std / 阶跃幅值) 最大 %.2f > 0.25 → "
              "τ 可信度下降，建议用纯阶跃段（静止基线）重录" % summ["bg_max"])
    print("    [参考] T_d(原始信号互相关) = %s（阶跃+一阶滞后时含 +≈0.7τ 偏置，勿直接用）"
          % fmt_ms(summ["xcorr"]))
    kval = [r["K"] for r in rows if np.isfinite(r["K"])]
    if kval:
        print("    K(静态增益)   = %.3f ± %.3f" % (float(np.median(kval)), float(np.std(kval))))


def analyze_joint(ds, i, signal="auto", max_events=64, use_scipy=True, quiet=False):
    """完整的单关节辨识；返回 (rows, summ, cmd_col, fb_col, why) 或 None。"""
    dt = ds.dt_ms()
    cmd_col, fb_col, why = pick_signal(ds, i, signal)
    if not ds.has(cmd_col) or not ds.has(fb_col):
        if not quiet:
            print("\n---- 关节 %s: 缺少列 %s/%s，跳过 ----" % (joint_name(i), cmd_col, fb_col))
        return None
    c = ds.get(cmd_col)
    if not np.isfinite(c).any():
        if not quiet:
            print("\n---- 关节 %s: %s 全为 NaN，跳过 ----" % (joint_name(i), cmd_col))
        return None
    events = detect_steps(ds.t, c, dt, max_events=max_events)
    if not events:
        if not quiet:
            print("\n---- 关节 %s (%s): 未检测到阶跃沿（|Δ| 未超 3σ 且保持），无法辨识 ----"
                  % (joint_name(i), why))
        return None
    rows = analyze_events(ds, i, cmd_col, fb_col, events, dt, use_scipy=use_scipy)
    if not rows:
        if not quiet:
            print("\n---- 关节 %s (%s): 有 %d 个初始沿，但后段窗口被污染/不足，无法辨识 ----"
                  % (joint_name(i), why, len(events)))
        return None
    summ = summarize(rows)
    return {"rows": rows, "summ": summ, "cmd": cmd_col, "fb": fb_col,
            "why": why, "i": i}


def run_analysis(ds, joint, signal="auto", max_events=64, use_scipy=True):
    """给程序内调用（selftest / 其它工具）的入口，返回 analyze_joint 的结果。"""
    return analyze_joint(ds, joint, signal=signal, max_events=max_events,
                         use_scipy=use_scipy, quiet=True)


def write_csv(path, results):
    """把若干关节的段明细落盘。results: list[(i, cmd, fb, rows)]"""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["joint_can", "joint_name", "cmd_col", "fb_col", "seg",
                    "t_step_ms", "dc", "td_edge_ms", "edge_peak_r",
                    "td_threshold_ms", "td_fit_ms", "td_rawxcorr_ms", "xcorr_peak_r",
                    "tau_ms", "K", "fit_rms", "fit_method"])
        for i, cmd, fb, rows in results:
            for k, r in enumerate(rows):
                w.writerow([i, joint_name(i), cmd, fb, k, "%.3f" % r["t_step"],
                            "%.6g" % r["dc"], "%.3f" % r["td_edge"], "%.4f" % r["edge_r"],
                            "%.3f" % r["td_thr"], "%.3f" % r["td_fit"],
                            "%.3f" % r["td_xcorr"], "%.4f" % r["xc_r"],
                            "%.3f" % r["tau_ms"], "%.5g" % r["K"],
                            "%.4g" % r["rms"], r["method"]])


def selftest(tmpdir="/tmp"):
    """生成合成数据(真值 T_d=18ms, τ=40ms)并校验估计误差 ≤5ms。"""
    print("=========== delay_fit.py 自测 ===========")
    path = os.path.join(tmpdir, "dsh_synth_delay.csv")
    path, meta, truth = write_synthetic_dataset(path)
    print("合成数据集: %s (+%s)" % (path, meta))
    print("真值: T_d = %.1f ms, τ = %.1f ms" % (truth["td_ms"], truth["tau_ms"]))
    ds = load_dataset(path)
    print("载入: %d 行 × %d 列, dt=%.3f ms, 时长 %.2f s, 丢行列 %d"
          % (ds.n, len(ds.header), ds.dt_ms(),
             (ds.t[-1] - ds.t[0]) / 1000.0, ds.n_mismatch))

    ok = True
    # 1) SPEED 轮（默认选 c_vel→m_vel）
    for i in (3, 7, 11, 15):
        res = run_analysis(ds, i)
        if res is None:
            print("[FAIL] 轮 can%02d 未辨识出结果" % i)
            ok = False
            continue
        td = res["summ"]["rec"][0]
        tau = res["summ"]["tau"][0]
        err = abs(td - truth["td_ms"])
        etau = abs(tau - truth["tau_ms"])
        flag = "PASS" if (err <= 5.0 and etau <= 5.0) else "FAIL"
        print("[%s] 轮 can%02d: T_d=%.1f ms (真值 %.1f, 误差 %.1f) | τ=%.1f ms (真值 %.1f, 误差 %.1f) | n=%d"
              % (flag, i, td, truth["td_ms"], err, tau, truth["tau_ms"], etau,
                 res["summ"]["n"]))
        ok = ok and flag == "PASS"
    # 2) 腿 IMPEDANCE（can00）
    res = run_analysis(ds, 0)
    if res is None:
        print("[FAIL] 腿 can00 未辨识出结果")
        ok = False
    else:
        td = res["summ"]["rec"][0]
        tau = res["summ"]["tau"][0]
        flag = "PASS" if (abs(td - truth["td_ms"]) <= 5.0 and abs(tau - truth["tau_ms"]) <= 5.0) else "FAIL"
        print("[%s] 腿 can00(IMPEDANCE): T_d=%.1f ms (误差 %.1f) | τ=%.1f ms (误差 %.1f) | n=%d"
              % (flag, td, abs(td - truth["td_ms"]), tau, abs(tau - truth["tau_ms"]),
                 res["summ"]["n"]))
        ok = ok and flag == "PASS"
    # 3) 腿 POSITION（can01, mode=2）
    res = run_analysis(ds, 1)
    if res is None:
        print("[FAIL] 腿 can01(mode=2) 未辨识出结果")
        ok = False
    else:
        td = res["summ"]["rec"][0]
        flag = "PASS" if abs(td - truth["td_ms"]) <= 5.0 else "FAIL"
        print("[%s] 腿 can01(POSITION,mode=2): T_d=%.1f ms (误差 %.1f) | n=%d"
              % (flag, td, abs(td - truth["td_ms"]), res["summ"]["n"]))
        ok = ok and flag == "PASS"
    # 4) 无阶跃通道的容错（temp 恒 0 通道）
    bad = analyze_joint(ds, 4, signal="tau", quiet=True)
    print("[INFO] 用 c_tau→m_tau 测 can04（恒定通道）: %s"
          % ("正确返回 None（无阶跃沿）" if bad is None else "意外得到结果"))
    print("=========== 自测%s ===========" % ("通过" if ok else "未通过"))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        prog="delay_fit.py",
        description="从阶跃/脉冲数据集估计端到端纯延迟 T_d 与一阶时间常数 τ（用法见模块 docstring）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python3 tool/delay_fit.py log/dataset_20261001_120000.csv --motor 3\n"
               "  python3 tool/delay_fit.py log/dataset_*.csv --auto --csv /tmp/delay.csv\n"
               "  python3 tool/delay_fit.py --selftest\n")
    p.add_argument("csv", nargs="?", help="统一数据集 CSV（188 列，500Hz）")
    p.add_argument("--motor", type=int, default=1,
                   help="关节索引 0..15（默认 CAN 序 i=can_port*4+motor_id-1；加 --policy-order 按 POLICY 序）")
    p.add_argument("--auto", action="store_true", help="扫描全部 16 关节，取阶跃沿最多者详细辨识")
    p.add_argument("--signal", choices=["auto", "pos", "vel", "tau"], default="auto",
                   help="激励/反馈通道（默认按行内 c_mode 自动选）")
    p.add_argument("--policy-order", action="store_true", help="--motor 按 POLICY 序(12 腿+4 轮)解释")
    p.add_argument("--csv", dest="out_csv", default=None, help="段明细落盘 CSV 路径")
    p.add_argument("--no-scipy", action="store_true", help="禁用 scipy 精修（只用网格搜索）")
    p.add_argument("--quiet", action="store_true", help="只输出汇总")
    p.add_argument("--selftest", action="store_true", help="跑合成数据自测（真值 T_d=18ms, τ=40ms）")
    p.add_argument("--tmpdir", default="/tmp", help="自测合成数据目录（默认 /tmp）")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest(args.tmpdir)
    if not args.csv:
        build_parser().print_help()
        print("\n[ERROR] 缺少数据集路径。")
        return 2

    for path in [args.csv]:
        print("=" * 78)
        print("端到端延迟辨识 (delay_fit.py): %s" % path)
        print("=" * 78)
        try:
            ds = load_dataset(path)
        except DatasetError as e:
            print("[ERROR] %s" % e)
            return 2
        for w in ds.warnings:
            print("[WARN] %s" % w)
        print("行数 %d, 时长 %.3f s, 中位采样间隔 %.3f ms" %
              (ds.n, (ds.t[-1] - ds.t[0]) / 1000.0, ds.dt_ms()))

        use_scipy = not args.no_scipy
        if use_scipy:
            try:
                import scipy  # noqa: F401
            except ImportError:
                use_scipy = False
                print("[INFO] 未安装 scipy，只用网格搜索（精度足够，τ 分辨率 40 点对数网格）")

        motor = args.motor
        if args.policy_order:
            if not (0 <= motor <= 15):
                print("[ERROR] --motor 超范围: %d（POLICY 序 0..15）" % motor)
                return 2
            can = POLICY_TO_CAN[motor]
            print("[INFO] POLICY%02d (%s) → CAN%02d (%s)"
                  % (motor, policy_name(motor), can, joint_name(can)))
            motor = can
        if not (0 <= motor <= 15):
            print("[ERROR] --motor 超范围: %d（CAN 关节索引 0..15）" % motor)
            return 2

        all_results = []
        if args.auto:
            found = []
            for i in range(16):
                res = analyze_joint(ds, i, args.signal, use_scipy=use_scipy, quiet=True)
                if res is not None:
                    found.append(res)
            if not found:
                print("\n[结论] 16 个关节都没有可用的阶跃沿 —— 该数据无法做延迟辨识。")
                print("       建议：录制时下发方波/阶跃（幅值 > 3σ 噪声）或检查 c_* 是否长期不变。")
                return 3
            found.sort(key=lambda r: -r["summ"]["n"])
            print("\n=== 各关节汇总（按阶跃段数排序）===")
            print("  can  关节            通道            n   T_d推荐(ms)  τ(ms)   K")
            for r in found:
                td = r["summ"]["rec"][0]
                tau = r["summ"]["tau"][0]
                kk = [x["K"] for x in r["rows"] if np.isfinite(x["K"])]
                print("  %2d   %-14s %-14s %3d %10.1f %8.1f %7.3f"
                      % (r["i"], joint_name(r["i"]).split("(")[0],
                         r["cmd"].replace("c_", ""), r["summ"]["n"], td, tau,
                         float(np.median(kk)) if kk else float("nan")))
            best = found[0]
            print("\n=== 最活跃关节详细报表 ===")
            print_joint_report(ds, best["i"], best["cmd"], best["fb"], best["why"],
                               best["rows"], best["summ"], quiet=args.quiet)
            all_results = [(r["i"], r["cmd"], r["fb"], r["rows"]) for r in found]
        else:
            res = analyze_joint(ds, motor, args.signal, use_scipy=use_scipy)
            if res is None:
                print("\n[结论] 关节 %s 没有足够的阶跃沿 —— 无法做延迟辨识。" % joint_name(motor))
                print("       提示：--auto 可扫描全部 16 关节找活跃通道；SPEED 关节看 c_vel。")
                return 3
            print_joint_report(ds, res["i"], res["cmd"], res["fb"], res["why"],
                               res["rows"], res["summ"], quiet=args.quiet)
            all_results = [(res["i"], res["cmd"], res["fb"], res["rows"])]

        print("\n[说明] 采样间隔 dt=%.3f ms → 单点延迟分辨率 %.1f ms；互相关峰值已做抛物线"
              "亚采样插值，但真实不确定度仍受 ±%.1f ms 离散化限制。"
              % (ds.dt_ms(), ds.dt_ms(), ds.dt_ms()))
        if all_results:
            i, cmd, fb, rows = all_results[0]
            print("       τ 由一阶+纯延迟拟合给出（网格 τ 40 点对数 + scipy 可选精修）。")
        if args.out_csv:
            try:
                write_csv(args.out_csv, all_results)
                print("[OK] 段明细已写入 %s" % args.out_csv)
            except OSError as e:
                print("[ERROR] 写 CSV 失败: %s" % e)
                return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
