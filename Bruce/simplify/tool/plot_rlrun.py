#!/usr/bin/env python3
"""RL 遥测追踪图：读真机 trace.csv 或 sim2sim --record CSV → 3 张追踪图。

按列自动识别（真机 S2RRecorder trace 有 qt0；sim --record 有 act_00/qpos_00）：
  - real：q 等已录，hip 转 status 显示；输出 *_pos.png（1_hip/2_leg/3_wheel）
  - sim ：用 act 重建 q_target=clip(nominal+act*0.25,limits)、轮目标=act*12.5，
          q=qpos、轮速=obs vel 轮分量；输出 *_sim.png（与真机同名便于对照）

用法:
    python3 tool/plot_rlrun.py <trace.csv 或 rlrun 目录 或 sim --record.csv> [outdir]

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

# ---- sim2sim --record CSV 重建策略目标常量（对齐 dogurdf_sim2sim_deploy/src/robots/dogurdf.py）----
# sim record 只给 act/qpos/vel，策略目标需重建：q_target = clip(nominal + act*scale, limits)。
SIM_NOMINAL = np.array([0.0, 0.20, -0.35] * 4 + [0.0] * 4)   # POLICY 16（腿 default + 轮0）
SIM_LO  = np.array([-0.6, -0.7, -1.0] * 4 + [-1e6] * 4)
SIM_HI  = np.array([0.6, 1.75, 0.35] * 4 + [1e6] * 4)
SIM_ACTION_SCALE = 0.25
SIM_WHEEL_VEL_SCALE = 12.5      # 轮目标速度 = act * WHEEL_VEL_LIMIT


def resolve_csv(path):
    if os.path.isdir(path):
        p = os.path.join(path, "trace.csv")
    else:
        p = path
    if not os.path.exists(p):
        sys.exit(f"[ERROR] 找不到 trace.csv: {p}")
    return p


def load(path):
    """按列自动识别：S2RRecorder trace(real) 或 sim2sim --record(sim)。返回 (d, schema)。"""
    rows = list(csv.DictReader(open(path)))
    cols = list(rows[0].keys())
    if "qt0" in cols:
        return _load_trace(rows, cols), "real"
    if "act_00" in cols and "qpos_00" in cols:
        return _load_sim(rows, cols), "sim"
    sys.exit("[ERROR] 无法识别 CSV：既非 S2RRecorder trace(缺 qt0) 也非 sim --record(缺 act_00/qpos_00)")


def _load_trace(rows, cols):
    """S2RRecorder trace.csv：qt/qtv/q/qd/trq 已直接给，POLICY 序。"""
    def V(n):
        if n not in cols:
            sys.exit(f"[ERROR] 缺列 {n}（不是 S2RRecorder 的 trace.csv？）")
        return np.array([float(r[n]) for r in rows])
    d = {n: V(n) for n in ["t_s", "step", "phase", "cmd_vx", "cmd_vyaw"]}
    d["qt"] = np.stack([V(f"qt{i}") for i in range(16)], 1)
    d["qtv"] = np.stack([V(f"qtv{i}") for i in range(16)], 1)
    d["q"] = np.stack([V(f"q{i}") for i in range(16)], 1)
    d["wvel"] = np.stack([V(f"wvel{i}") for i in range(4)], 1)
    return d


def _load_sim(rows, cols):
    """sim2sim --record CSV：重建策略目标 q_target=clip(nominal+act*scale, limits)，
    轮目标速度=act_w*WHEEL_VEL_SCALE；q=qpos(实际)、wvel=obs 关节速度的轮分量。"""
    def V(n):
        if n not in cols:
            sys.exit(f"[ERROR] 缺列 {n}（不是 sim2sim --record 的 CSV？）")
        return np.array([float(r[n]) for r in rows])
    act = np.stack([V(f"act_{i:02d}") for i in range(16)], 1)
    qpos = np.stack([V(f"qpos_{i:02d}") for i in range(16)], 1)
    vel = np.stack([V(f"vel_{i:02d}") for i in range(16)], 1)   # obs 关节速度（含轮）
    qt = np.clip(SIM_NOMINAL + act * SIM_ACTION_SCALE, SIM_LO, SIM_HI)
    qtv = np.zeros_like(qt)
    qtv[:, 12:16] = act[:, 12:16] * SIM_WHEEL_VEL_SCALE
    d = {"t_s": np.arange(len(act)) * 0.02,           # 50Hz，sim 无 t_s 列
         "step": V("step"), "cmd_vx": V("cmd_vx"), "cmd_vyaw": V("cmd_wz"),
         "qt": qt, "qtv": qtv, "q": qpos, "wvel": vel[:, 12:16]}
    return d


def fig_axes(nr, nc, title):
    fig, axs = plt.subplots(nr, nc, figsize=(12, 6.5 if nr * nc >= 4 else 4.5), sharex=True)
    return fig, np.atleast_2d(axs)


def phase_lines(ax, t):
    # 若含 phase 且存在 1 段，画段边界竖线（可读性）
    mx = t.max()
    pass


def plot1(d, outdir):
    t = d["t_s"]
    sim = d.get("sim", False)
    suf = "_sim" if sim else ""
    off = 0.0 if sim else CONV_B_HIP      # real 转 status 坐标显示（竖直≈0）；sim 无 status 层用 urdf
    ylab = "hip (urdf)" if sim else "hip (status)"
    lcmd, lfb = ("policy target", "sim q") if sim else ("RL cmd (status)", "motor fb (status)")
    fig, axs = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for k, ax in enumerate(axs.flat):
        i = 3 * k  # hip policy idx
        ax.plot(t, d["qt"][:, i] - off, color="C0", lw=1.6, label=lcmd)
        ax.plot(t, d["q"][:, i] - off, color="C1", lw=1.1, label=lfb)
        ax.axhline(0, color="gray", lw=0.6, ls="--")
        ax.set_ylim(-0.35, 0.5)
        ax.set_ylabel(f"{LEG[k]} {ylab}")
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
    axs[0, 0].set_title("Hip tracking: target vs actual")
    axs[1, 0].set_xlabel("t (s)")
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "1_hip_pos" + suf + ".png"), dpi=110); plt.close()


def plot2(d, outdir):
    t = d["t_s"]
    suf = "_sim" if d.get("sim", False) else ""
    fig, axs = plt.subplots(2, 2, figsize=(12, 6.5), sharex=True)
    sub = [(1, "FL thigh"), (2, "FL calf"), (4, "FR thigh"), (5, "FR calf")]
    for ax, (j, nm) in zip(axs.flat, sub):
        ax.plot(t, d["qt"][:, j], color="C0", lw=1.5, label="q_target")
        ax.plot(t, d["q"][:, j], color="C1", lw=1.0, label="q actual")
        ax.set_ylabel(nm + " urdf(rad)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    axs[0, 0].set_title("Leg pos tracking (URDF: q_target vs q)")
    axs[1, 0].set_xlabel("t (s)")
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "2_leg_pos" + suf + ".png"), dpi=110); plt.close()


def plot3(d, outdir):
    t = d["t_s"]
    suf = "_sim" if d.get("sim", False) else ""
    fig, axs = plt.subplots(2, 2, figsize=(12, 6.5), sharex=True)
    for k, ax in enumerate(axs.flat):
        ax.plot(t, d["qtv"][:, 12 + k], color="C0", lw=1.4, label="wheel target")
        ax.plot(t, d["wvel"][:, k], color="C1", lw=1.0, label="wheel actual")
        ax.set_ylabel(f"{LEG[k]} wheel (rad/s)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    axs[0, 0].set_title("Wheel speed tracking: target vs actual")
    axs[1, 0].set_xlabel("t (s)")
    plt.tight_layout(); plt.savefig(os.path.join(outdir, "3_wheel_vel" + suf + ".png"), dpi=110); plt.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    csv_path = resolve_csv(sys.argv[1])
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(csv_path) or "."
    os.makedirs(outdir, exist_ok=True)
    d, schema = load(csv_path)
    d["sim"] = (schema == "sim")
    plot1(d, outdir); plot2(d, outdir); plot3(d, outdir)
    suf = "_sim" if schema == "sim" else ""
    print(f"  [{schema}] → {os.path.join(outdir, '1_hip_pos' + suf + '.png')}")
    print(f"  [{schema}] → {os.path.join(outdir, '2_leg_pos' + suf + '.png')}")
    print(f"  [{schema}] → {os.path.join(outdir, '3_wheel_vel' + suf + '.png')}")


if __name__ == "__main__":
    main()
