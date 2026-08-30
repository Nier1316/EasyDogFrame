#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制腿关节扭矩曲线：hip(侧摆/髋) / thigh(大腿) / calf(膝)，4 腿 CAN0~3。

用法:
    python3 tool/plot_joint_torque.py <recv.csv> [out.png]
示例:
    python3 tool/plot_joint_torque.py log/recv_20260830_080628.csv /tmp/torque.png
"""
import csv
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from collections import defaultdict

# 注册系统中文字体（Noto Serif CJK），避免中文变方块
for _f in font_manager.findSystemFonts():
    if "Noto" in _f and "CJK" in _f:
        font_manager.fontManager.addfont(_f)
plt.rcParams["font.sans-serif"] = ["Noto Serif CJK SC", "Noto Sans CJK SC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

path = sys.argv[1] if len(sys.argv) > 1 else "log/recv_20260830_080628.csv"
out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/torque_joint.png"

rows = defaultdict(list)
with open(path) as f:
    for r in csv.DictReader(f):
        mi = int(r["motor_id"])
        if mi <= 3:   # 只画腿关节（hip/thigh/calf）
            try:
                rows[(r["can_port"], mi)].append(
                    (float(r["elapsed_ms"]) / 1000.0, float(r["cal_torque"])))
            except (TypeError, ValueError):
                pass

names = {1: "侧摆/髋 (hip)", 2: "大腿 (thigh)", 3: "膝 (calf)"}
legs = ["CAN0 FL", "CAN1 FR", "CAN2 RL", "CAN3 RR"]

fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
for mi in (1, 2, 3):
    ax = axes[mi - 1]
    for cp in range(4):
        rs = rows[(str(cp), mi)]
        if not rs:
            continue
        # 窗口降采样且保留峰值（每窗口取 |tau| 最大的点），避免瞬时尖峰（如 150 饱和）被跳过
        win = max(1, len(rs) // 2000)
        t, tau = [], []
        for i in range(0, len(rs), win):
            seg = rs[i:i + win]
            peak = max(seg, key=lambda x: abs(x[1]))
            t.append(peak[0])
            tau.append(peak[1])
        ax.plot(t, tau, label=legs[cp], lw=0.8)
    ax.set_ylabel(f"{names[mi]} 扭矩 (N·m)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
axes[-1].set_xlabel("时间 (s)")
fig.suptitle(f"关节扭矩曲线 - {path.split('/')[-1]}")
plt.tight_layout()
plt.savefig(out, dpi=100)
print(f"保存: {out}")
