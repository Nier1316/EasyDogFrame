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
def qc_pace(path, rate=400.0):
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

    hard_ok = all(g for name, g, _ in gates[:5])
    if not hard_ok:
        print(f"  {BAD} 结论：**不可用于拟合**（先修上面标红项）")
    elif upd < 200:
        print(f"  {WARN} 结论：时间基已达标，可导出；但 meas 只有 {upd:.0f} Hz 刷新率 ⇒ "
              f"5~10 Hz 激励段不可观测，I_a / T_d 辨不出来（只有 d / τ_f / q̃_b 等低频项可信）")
    else:
        print(f"  {OK} 结论：通过全部判据")
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
    ok = qc_dataset(path) if "dataset_" in os.path.basename(path) else qc_pace(path)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
