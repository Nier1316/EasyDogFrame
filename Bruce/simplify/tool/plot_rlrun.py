#!/usr/bin/env python3
"""RL 遥测追踪图：读 S2RRecorder 落盘的 trace.csv → 3 张真机追踪图。

用法:
    python3 tool/plot_rlrun.py log/rlrun_<ts>/trace.csv [outdir]
    python3 tool/plot_rlrun.py log/rlrun_<ts>/            # 目录则取目录内 trace.csv

输出（默认写到 csv 同目录）:
    1_hip_pos.png    髋位置追踪：RL 下发 cmd(status) vs 电机反馈 fb(status) ×4髋
    2_leg_pos.png    腿位置追踪：FL/FR thigh+calf，q_target vs q (URDF)
    3_wheel_vel.png  轮速追踪：目标(下发) vs 实际 ×4轮

列约定（与 trace.csv header 一致，全 POLICY 序）:
    t_s,step,phase,cmd_vx,cmd_vyaw, qt0..15, qtv0..15, q0..15, qd0..15,
    trq0..15, wvel0..3, qw,qx,qy,qz,gx,gy,gz,yaw,roll,pitch,yaw_rate
    qt/q/qd/trq 均 URDF；hip status = (urdf - CONV_B)/CONV_A（CONV_B_hip=0.0297,A=+1）
"""
import sys, os, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CONV_B_HIP = 0.0297     # hip: urdf = A*status + B (A=+1)
LEG = ["FL", "FR", "RL", "RR"]
JN = ["hip", "thigh", "calf"]


def resolve_csv(path):
    if os.path.isdir(path):
        p = os.path.join(path, "trace.csv")
    else:
        p = path
    if not os.path.exists(p):
        sys.exit(f"[ERROR] 找不到 trace.csv: {p}")
    return p


def load(path):
    rows = list(csv.DictReader(open(path)))
    cols = rows[0].keys()
    def V(n):
        if n not in cols:
            sys.exit(f"[ERROR] 缺列 {n}（不是 S2RRecorder 的 trace.csv？）")
        return np.array([float(r[n]) for r in rows])
    d = {n: V(n) for n in ["t_s", "step", "phase", "cmd_vx", "cmd_vyaw"]}
    d["qt"] = np.stack([V(f"qt{i}") for i in range(16)], 1)
    d["qtv"] = np.stack([V(f"qtv{i}") for i in range(16)], 1)
    d["q"] = np.stack([V(f"q{i}") for i in range(16)], 1)
    d["wvel"] = np.stack([V(f"wvel{i}") for i in range(4)], 1)
    return d, rows[0].keys()


def fig_axes(nr, nc, title):
    fig, axs = plt.subplots(nr, nc, figsize=(12, 6.5 if nr * nc >= 4 else 4.5), sharex=True)
    return fig, np.atleast_2d(axs)


def phase_lines(ax, t):
    # 若含 phase 且存在 1 段，画段边界竖线（可读性）
    mx = t.max()
    pass


def plot1(d, outdir):
    t = d["t_s"]
    fig, axs = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for k, ax in enumerate(axs.flat):
        i = 3 * k  # hip policy idx
        cmd = (d["qt"][:, i] - CONV_B_HIP) / 1.0
        fb = (d["q"][:, i] - CONV_B_HIP) / 1.0
        ax.plot(t, cmd, color="C0", lw=1.6, label="RL cmd (status)")
        ax.plot(t, fb, color="C1", lw=1.1, label="motor fb (status)")
        ax.axhline(0, color="gray", lw=0.6, ls="--")
        ax.set_ylim(-0.35, 0.5)
        ax.set_ylabel(f"{LEG[k]} hip (rad)")
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
    axs[0, 0].set_title("Hip pos tracking: RL cmd vs motor fb")
    axs[1, 0].set_xlabel("t (s)")
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "1_hip_pos.png"), dpi=110); plt.close()


def plot2(d, outdir):
    t = d["t_s"]
    fig, axs = plt.subplots(2, 2, figsize=(12, 6.5), sharex=True)
    sub = [(1, "FL thigh"), (2, "FL calf"), (4, "FR thigh"), (5, "FR calf")]
    for ax, (j, nm) in zip(axs.flat, sub):
        ax.plot(t, d["qt"][:, j], color="C0", lw=1.5, label="q_target")
        ax.plot(t, d["q"][:, j], color="C1", lw=1.0, label="q actual")
        ax.set_ylabel(nm + " urdf(rad)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    axs[0, 0].set_title("Leg pos tracking (URDF: q_target vs q)")
    axs[1, 0].set_xlabel("t (s)")
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "2_leg_pos.png"), dpi=110); plt.close()


def plot3(d, outdir):
    t = d["t_s"]
    fig, axs = plt.subplots(2, 2, figsize=(12, 6.5), sharex=True)
    for k, ax in enumerate(axs.flat):
        ax.plot(t, d["qtv"][:, 12 + k], color="C0", lw=1.4, label="wheel target")
        ax.plot(t, d["wvel"][:, k], color="C1", lw=1.0, label="wheel actual")
        ax.set_ylabel(f"{LEG[k]} wheel (rad/s)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    axs[0, 0].set_title("Wheel speed tracking: target vs actual")
    axs[1, 0].set_xlabel("t (s)")
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "3_wheel_vel.png"), dpi=110); plt.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    csv_path = resolve_csv(sys.argv[1])
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(csv_path) or "."
    os.makedirs(outdir, exist_ok=True)
    d, _ = load(csv_path)
    plot1(d, outdir); plot2(d, outdir); plot3(d, outdir)
    print(f"  → {os.path.join(outdir, '1_hip_pos.png')}")
    print(f"  → {os.path.join(outdir, '2_leg_pos.png')}")
    print(f"  → {os.path.join(outdir, '3_wheel_vel.png')}")


if __name__ == "__main__":
    main()
