#!/usr/bin/env python3
"""PACE / 反馈链路体检工具（2026-10-08）

一次跑完两份"体检"，输出判据表 + 结论；退出码 0 = 全部通过。

  PACE 采集体检（log/pace/chirp_*.csv，26 列）
      · 有效拍数 / 时长 / 平均周期偏差 / 最大单拍  → pace_export_dataset.py 的准入判据
      · 末行是否被截断（进程异常终止会断在行中）
      · **meas 刷新率**（12 关节整体 + 逐关节最长保持）→ 反侦查"反馈被攒批"
      · 哪些关节从头到尾没更新（= 该路 CAN 无反馈）

  500 Hz 数据集体检（log/dataset_*.csv，188 列）
      · 采样周期是否真的 500 Hz
      · m_pos / m_vel / c_pos 各自的有效更新率 → 一眼看出反馈是不是 ~10 Hz 阶梯

用法：
    python3 tool/pace_qc.py                          # 自动挑 log/pace 里最新的一份
    python3 tool/pace_qc.py log/pace/chirp_xxx.csv
    python3 tool/pace_qc.py log/dataset_xxx.csv
    python3 tool/pace_qc.py --compare A.csv B.csv    # 两份对比（例如：开/关 LD_PRELOAD 垫片）

为什么需要它：2026-10-08 用手工分析发现，所有 PACE 采集的 meas 都只有 ~10 Hz 刷新率
（8000 拍里只有 387 拍发生变化），根因是达妙 SDK 写死的 libusb 事件收割超时 100 ms。
详见 tool/libusb_fastpoll.c。这个脚本把那次手工分析固化下来。
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import statistics
import sys

JOINTS = ["LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
          "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"]

OK, BAD, WARN = "✅", "🔴", "⚠️ "


# ---------------------------------------------------------------- 通用
def read_csv(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    if not rows:
        return [], [], False
    hdr = rows[0]
    body = rows[1:]
    truncated = bool(body) and len(body[-1]) != len(hdr)
    body = [r for r in body if len(r) == len(hdr)]
    return hdr, body, truncated


def runs_of(col):
    """返回 (游程长度列表, 变化点索引列表)"""
    lens, changes, cur = [], [], 1
    for i in range(1, len(col)):
        if col[i] == col[i - 1]:
            cur += 1
        else:
            lens.append(cur)
            changes.append(i)
            cur = 1
    lens.append(cur)
    return lens, changes


def refresh_rate(col, dt):
    """给定逐拍取值与标称周期 dt(s)，返回有效更新率(Hz)"""
    lens, _ = runs_of(col)
    m = statistics.mean(lens) if lens else 1
    return 1.0 / (m * dt)


# ---------------------------------------------------------------- PACE
def read_meta(path):
    """读同名 *.meta.txt（采集时自动生成），拿 kp/kd/amp 等参数"""
    d = {}
    mp = path + ".meta.txt"
    if not os.path.exists(mp):
        return d
    with open(mp, encoding="utf-8") as fh:
        for line in fh:
            if "=" in line:
                k, v = line.strip().split("=", 1)
                try:
                    d[k] = float(v)
                except ValueError:
                    d[k] = v
    return d


def dyn_checks(hdr, body, meta, tau_max=None):
    # 命令力矩上限（include/motor/ele_motor_def.h::TORQUE_CMD_LIMIT）：Hip/Thigh 120、Calf 200。
    # ⚠ 不是 27 N·m（那是别的机型/示例的 P_MAX）；用错会把正常数据误报成"限幅"。
    if tau_max is None:
        lim = [120.0, 120.0, 200.0] * 4          # 每个 leg: HAA/HFE/KFE
    else:
        lim = [tau_max] * 12
    """动力学体检（2026-10-19 新增，针对"关节摩擦 ≈7 N·m"的现场条件）

    基于采集时 PD 的实际出力  τ = kp·(des−meas) − kd·v_meas，逐项判：
      · 摩擦占比：median|τ| 与激励上限 kp·amp 之比 —— 低于 3:1 时辨识条件差（摩擦吃掉激励）
      · 力矩限幅：|τ| > tau_max 的拍占比 —— 这些拍 PACE 用 kp·e 反推的力矩与实际不符，应剔除
      · 黏滞：|v| < 0.01 rad/s 的拍占比 —— 高则 stick-slip，动力学信息被摩擦淹没
      · 可用带宽：分频段 std(meas)/std(des)，找出增益降到 0.5 的频率（高频段无信息）
    纯标准库实现（不依赖 numpy），返回值 (行列表, 是否全部合格)
    """
    kp = float(meta.get("kp", 250.0))
    kd = float(meta.get("kd", 4.0))
    amp = float(meta.get("amp_rad", 0.06))
    idx_d = [hdr.index(f"des_{j}") for j in JOINTS]
    idx_m = [hdr.index(f"meas_{j}") for j in JOINTS]
    t = [float(r[hdr.index("t_actual_s")]) for r in body]
    n = len(body)
    dt = (t[-1] - t[0]) / (n - 1) if n > 1 else 0.0025

    rows = []
    tau_med_all, tau_stuck_all, sat_all, stick_all = [], [], [], []
    for i in range(12):
        d = [float(r[idx_d[i]]) for r in body]
        m = [float(r[idx_m[i]]) for r in body]
        v = [0.0] * n
        for k in range(1, n - 1):
            v[k] = (m[k + 1] - m[k - 1]) / (2 * dt)
        v[0], v[-1] = v[1], v[-2]
        e = [d[k] - m[k] for k in range(n)]
        tau = [kp * e[k] - kd * v[k] for k in range(n)]
        at = [abs(x) for x in tau]
        tau_med_all.append(statistics.median(at))
        vd = [0.0] * n
        for k in range(1, n - 1):
            vd[k] = (d[k + 1] - d[k - 1]) / (2 * dt)
        vd[0], vd[-1] = vd[1], vd[-2]
        # 黏滞 = 实测速度远小于命令速度（只在命令确实在动时统计，扣掉过零的几何成分）
        moving = [k for k in range(n) if abs(vd[k]) > 0.02]
        stuck = [at[k] for k in range(n) if abs(v[k]) < 0.2 * abs(vd[k])]
        if len(stuck) >= 20:
            tau_stuck_all.append(statistics.median(stuck))
        sat_all.append(sum(1 for x in at if x > lim[i]) / n)
        stick_all.append(sum(1 for k in moving if abs(v[k]) < 0.2 * abs(vd[k])) / len(moving)
                         if moving else 0.0)

    tau_med = statistics.median(tau_med_all)
    tau_amp = kp * amp
    # τ_f 估计：用"黏滞拍"的中位力矩（|v|<0.01 时 τ 主要就是静摩擦+重力），
    # 比 median|τ| 干净 —— 后者已包含动态力矩，拿它当分母是循环论证。
    tf_est = statistics.median(tau_stuck_all) if tau_stuck_all else tau_med
    ratio = tau_amp / tf_est if tf_est > 0 else 99.0
    sat = statistics.mean(sat_all)
    stick = statistics.mean(stick_all)

    # 分频段增益（8 段，等时长）
    nb = 8
    gains = []
    for b in range(nb):
        a0, a1 = b * n // nb, (b + 1) * n // nb
        gs = []
        for i in range(12):
            d = [float(r[idx_d[i]]) for r in body[a0:a1]]
            m = [float(r[idx_m[i]]) for r in body[a0:a1]]
            sd = statistics.pstdev(d) if len(d) > 1 else 0.0
            sm = statistics.pstdev(m) if len(m) > 1 else 0.0
            if sd > 1e-9:
                gs.append(sm / sd)
        gains.append(statistics.median(gs) if gs else 0.0)
    f0 = float(meta.get("f0_hz", 0.1))
    f1 = float(meta.get("f1_hz", 10.0))
    # ⚠ 采集被截断时，实际扫频只走到 f_actual = f0 + (f1-f0)*t_last/duration，
    #   分频段增益必须按**实际到达的频率**标注，否则会把 2.7 Hz 的数据标成 4.7 Hz。
    dur_meta = float(meta.get("duration_s", 20.0))
    t_last = float(t[-1]) if t else 0.0
    frac = min(1.0, t_last / dur_meta) if dur_meta > 0 else 1.0
    f1_eff = f0 + (f1 - f0) * frac
    freqs = [f0 + (f1_eff - f0) * (b + 0.5) / nb for b in range(nb)]
    bw = None
    for f, g in zip(freqs, gains):
        if g < 0.5:
            bw = f
            break

    # meta 与实测数据一致性：des 峰峰应为 2×amp（防止"环境变量没生效却看不出来"）
    pp = statistics.mean([max(float(r[idx_d[i]]) for r in body) -
                          min(float(r[idx_d[i]]) for r in body) for i in range(12)])
    amp_meas = pp / 2.0
    amp_ok = abs(amp_meas - amp) <= max(0.05 * amp, 2e-3)
    rows.append(("meta 与实际数据一致（幅值）", amp_ok,
                 f"meta 写 {amp:.4f} rad，实测 {amp_meas:.4f} rad"
                 + ("" if amp_ok else "  ← 参数没生效？检查环境变量/是否走了 sudo（sudo 吞 env）")))
    rows.append(("摩擦/激励比 ≥ 2.5:1（辨识条件）", ratio >= 2.5,
                 f"激励 {tau_amp:.1f} N·m / 静摩擦估计 {tf_est:.1f} N·m = {ratio:.1f}:1"))
    rows.append(("力矩限幅拍 < 1%", sat < 0.01,
                 f"|τ|超上限(Hip/Calf 120/200 N·m) 占 {sat*100:.1f}%"
                 + ("" if sat < 0.01 else "，这些拍应剔除")))
    # 黏滞只作信息：实测速度由 0.381 mrad 量化的位置差分得到，低频段量化噪声会主导，不能当门禁
    info = [("黏滞拍（信息，非门禁）", True, f"{stick*100:.1f}% 的'命令在动'采样里实测速度不足命令的 20%")]
    rows.append(("可用带宽 ≥ 2 Hz", (bw is None or bw >= 2.0),
                 ("全频段增益 ≥0.5" if bw is None else f"增益降到 0.5 约在 {bw:.1f} Hz")
                 + f"；实际扫到 {f1_eff:.2f} Hz（分段增益 {', '.join(f'{g:.2f}' for g in gains)}）"))
    ok = all(g for _, g, _ in rows)
    return rows + info, ok


def qc_pace(path, rate=400.0, tau_max=27.0, strict=False):
    hdr, body, truncated = read_csv(path)
    if not hdr or "t_actual_s" not in hdr:
        print(f"{BAD} 不是 PACE 采集文件（缺 t_actual_s 列）: {path}")
        return False
    need = [f"meas_{j}" for j in JOINTS]
    if not all(c in hdr for c in need):
        print(f"{BAD} 缺 meas_* 列: {path}")
        return False

    tgt = 1.0 / rate
    n = len(body)
    t = [float(r[hdr.index("t_actual_s")]) for r in body]
    dts = [t[i + 1] - t[i] for i in range(n - 1)]
    mean_dt = statistics.mean(dts) if dts else 0.0
    dev = (mean_dt - tgt) / tgt * 100 if tgt else 0.0
    max_dt = max(dts) if dts else 0.0

    idx = [hdr.index(f"meas_{j}") for j in JOINTS]
    changed = [any(body[i][k] != body[i - 1][k] for k in idx) for i in range(1, n)]
    n_changed = sum(changed)
    dur = t[-1] if t else 0.0
    upd = n_changed / dur if dur > 0 else 0.0

    # 逐关节最长"连续同值"
    hold = []
    for k in idx:
        lens, _ = runs_of([r[k] for r in body])
        hold.append(max(lens) * mean_dt * 1000 if lens else 0.0)
    dead = [JOINTS[i] for i, h in enumerate(hold) if h > dur * 1000 * 0.5]

    gates = [
        ("有效拍数 ≥ 8000（20s@400Hz）", n >= 8000, f"{n} 拍 / {dur:.2f}s"),
        ("未截断（进程正常结束）", not truncated, "末行完整" if not truncated else "末行被截断"),
        ("平均周期偏差 < 1%", abs(dev) < 1.0, f"{mean_dt*1000:.4f} ms（{dev:+.3f}%）"),
        ("最大单拍 < 3×周期", max_dt < 3 * tgt, f"{max_dt*1000:.3f} ms"),
        ("12 关节全部有反馈", not dead, "全部有" if not dead else f"无反馈: {','.join(dead)}"),
        ("meas 刷新率 ≥ 200 Hz", upd >= 200, f"{upd:.1f} Hz（{n_changed}/{n-1} 拍发生变化）"),
    ]
    print(f"\n===== PACE 体检: {os.path.basename(path)} =====")
    for name, good, detail in gates:
        print(f"  {OK if good else BAD} {name:<34} {detail}")
    print(f"  ·  逐关节最长保持(ms): " +
          " ".join(f"{j}={h:.0f}" for j, h in zip(JOINTS, hold)))

    # ---- 动力学体检（2026-10-19：针对"关节摩擦 ≈7 N·m"，单纯时间基合格不够）----
    meta = read_meta(path)
    meta_src = f"{os.path.basename(path)}.meta.txt" if meta else "无 meta.txt（用默认 kp=250/kd=4/amp=0.06）"
    print(f"  ·  动力学体检参数来源: {meta_src}")
    drows, dok = dyn_checks(hdr, body, meta, tau_max)
    for name, good, detail in drows:
        print(f"  {OK if good else WARN} {name:<34} {detail}")

    hard_ok = all(g for name, g, _ in gates[:5])
    if not hard_ok:
        print(f"  {BAD} 结论：**不可用于拟合**（先修上面标红项）")
    elif upd < 200:
        print(f"  {WARN} 结论：时间基已达标，可导出；但 meas 只有 {upd:.0f} Hz 刷新率 ⇒ "
              f"5~10 Hz 激励段不可观测，I_a / T_d 辨不出来（只有 d / τ_f / q̃_b 等低频项可信）")
    elif not dok:
        amp_now = meta.get("amp_rad", 0.06)
        tip = ("幅值已到位（≥0.084）⇒ 摩擦/激励比受限于 τ_f 本身；"
               "可接受，τ_f 会由辨识吸收" if amp_now >= 0.084
               else f"幅值偏小（{amp_now:.3f}）⇒ 建议提到 ≥0.084（推荐 0.10）")
        print(f"  {WARN} 结论：时间基/反馈率/带宽达标，**可用于拟合**；但摩擦占比偏高 ⇒ "
              f"τ_f、偏置可信，I_a / T_d 不确定度大。{tip}")
    else:
        print(f"  {OK} 结论：通过全部判据（含动力学条件）")
    if strict:
        return hard_ok and upd >= 200 and dok
    return hard_ok and upd >= 200


# ---------------------------------------------------------------- 数据集
def qc_dataset(path):
    hdr, body, truncated = read_csv(path)
    if not hdr or "m_pos_00" not in hdr:
        print(f"{BAD} 不是 500 Hz 数据集（缺 m_pos_00 列）: {path}")
        return False
    n = len(body)
    t = [float(r[hdr.index("t_ms")]) for r in body]
    dts = [t[i + 1] - t[i] for i in range(n - 1)]
    med_dt = statistics.median(dts) if dts else 0.0

    print(f"\n===== 500Hz 数据集体检: {os.path.basename(path)} =====")
    print(f"  ·  行数={n}  时长={t[-1]/1000:.2f}s  采样周期中位={med_dt:.3f} ms（{1000/med_dt:.0f} Hz）"
          f"  末行截断={truncated}")
    print(f"  {'信号':<10} {'不同取值':>8} {'游程中位(拍)':>12} {'有效更新率':>12}")
    worst = 1e9
    for name in ("m_pos_00", "m_vel_00", "m_tau_00", "c_pos_00"):
        if name not in hdr:
            continue
        col = [r[hdr.index(name)] for r in body]
        lens, _ = runs_of(col)
        m = statistics.median(lens)
        rate = 1.0 / (m * med_dt / 1000.0)
        if name.startswith("m_"):
            worst = min(worst, rate)
        print(f"  {name:<10} {len(set(col)):>8} {m:>12.0f} {rate:>10.1f} Hz")
    good = worst >= 200
    print(f"  {OK if good else BAD} 反馈刷新率（最差反馈通道）: {worst:.1f} Hz"
          f"{'' if good else '  ← 典型 ~10 Hz = 达妙 SDK 的 100 ms 事件收割超时，见 tool/libusb_fastpoll.c'}")
    return good


# ---------------------------------------------------------------- 入口
def pick_latest():
    cands = sorted(glob.glob("log/pace/chirp_*.csv"))
    if not cands:
        print("找不到 log/pace/chirp_*.csv；请传路径")
        sys.exit(2)
    return cands[-1]


def key_metrics(path):
    """给 --compare 用的精简指标"""
    hdr, body, truncated = read_csv(path)
    if "t_actual_s" in hdr:
        t = [float(r[hdr.index("t_actual_s")]) for r in body]
        dts = [t[i + 1] - t[i] for i in range(len(t) - 1)]
        mean_dt = statistics.mean(dts)
        idx = [hdr.index(f"meas_{j}") for j in JOINTS]
        ch = sum(1 for i in range(1, len(body)) if any(body[i][k] != body[i - 1][k] for k in idx))
        return (len(body), mean_dt * 1000, (mean_dt - 0.0025) / 0.0025 * 100,
                ch / t[-1] if t[-1] > 0 else 0.0)
    if "m_pos_00" in hdr:
        t = [float(r[hdr.index("t_ms")]) for r in body]
        med_dt = statistics.median([t[i + 1] - t[i] for i in range(len(t) - 1)])
        lens, _ = runs_of([r[hdr.index("m_pos_00")] for r in body])
        return (len(body), med_dt, 0.0, 1.0 / (statistics.median(lens) * med_dt / 1000.0))
    return (len(body), 0.0, 0.0, 0.0)


def main():
    ap = argparse.ArgumentParser(description="PACE / 反馈链路体检")
    ap.add_argument("csv", nargs="?", help="要体检的 CSV（省略则取 log/pace 最新一份）")
    ap.add_argument("--compare", nargs=2, metavar=("A", "B"), help="对比两份的 meas 刷新率")
    ap.add_argument("--tau-max", type=float, default=None,
                    help="统一力矩上限；省略则按关节类型（Hip/Thigh 120、Calf 200，见 ele_motor_def.h）")
    ap.add_argument("--strict", action="store_true", help="动力学条件不合格也让退出码非 0")
    a = ap.parse_args()

    if a.compare:
        print(f"{'文件':<44} {'行数':>7} {'周期ms':>8} {'偏差%':>8} {'meas刷新Hz':>12}")
        for p in a.compare:
            n, dt, dev, upd = key_metrics(p)
            print(f"{os.path.basename(p):<44} {n:>7} {dt:>8.3f} {dev:>+8.2f} {upd:>12.1f}")
        print("\n判读：第二份的 meas 刷新率若从 ~10 Hz 跳到数百 Hz，说明 LD_PRELOAD 垫片生效。")
        return 0

    path = a.csv or pick_latest()
    if not os.path.exists(path):
        print(f"找不到 {path}")
        return 2
    ok = (qc_dataset(path) if "dataset_" in os.path.basename(path)
          else qc_pace(path, tau_max=a.tau_max, strict=a.strict))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
