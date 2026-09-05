#!/usr/bin/env python3
"""真机摩擦辨识离线回归：读 Example54 落盘 log/fric_id/ → [b, fc] ± σ。

管线（2026-09-04 由"稳态配对"改为"逐样本直接回归"）：
  1. 遍历 log/fric_id/C{c}M{m}_R{r}.csv（按关节分组），读 info.txt 取 θ_c/amp/速度档
  2. 逐样本扣重力：τ_res = τ − g_est(θ)（阶段C 已实测 g(θ) 插值）
  3. 滤低速：|ω| < W_MIN(0.15) 剔除（sign 抖动/换向区）
  4. 回归 τ_res ~ [ω, sign(ω)] → b(粘性)/fc(库仑)（clip 非负）
  5. 每回合独立回归 → b±σ, fc±σ（误差带），质量检查 σ/均值 < 30% 且 R² > 0.7
旧配对法失效原因：位置斜坡+PD 跟踪下实际速度有纹波，|ω−v_des|<tol 稳态样本仅
  2~4%、正反向在 θ 上踩不齐 → 配对恒空（实测多关节 0 配对）。

用法：
  /home/sysu/miniconda3/envs/MJX/bin/python tool/friction_id_offline.py            # 回归真机数据
  /home/sysu/miniconda3/envs/MJX/bin/python tool/friction_id_offline.py --demo     # 生成模拟数据自测管线
"""

from __future__ import annotations

import argparse
import glob
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
DATA_DIR = os.path.join(PROJECT, "log", "fric_id")

N_BIN = 15          # θ 分箱数
STEADY_REL = 0.15   # 稳态速度容差（相对 v_des）
STEADY_ABS = 0.05   # 稳态速度容差下限（rad/s）
N_MIN = 5           # 每个配对 bin 最少样本数（旧配对法用，直接回归不再依赖）
SIGMA_RATIO_MAX = 0.30   # 误差带质量阈值：σ/均值
W_MIN = 0.15        # 直接回归最低 |ω| (rad/s)：滤 sign 抖动/换向区

J_NAMES = ["hip", "thigh", "calf"]


def read_info(path):
    """读 info.txt → dict。"""
    info = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if "=" in line:
                k, v = line.split("=", 1)
                info[k] = float(v)
    n_vel = int(info.get("n_vel", 3))
    info["vels"] = [info.pop(f"vel{i}") for i in range(n_vel)]
    return info


def read_csv(path):
    """读回合 CSV（elapsed_ms,theta,omega,tau,v_des）→ (theta, omega, tau, v)。"""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data[None, :]
    return data[:, 1], data[:, 2], data[:, 3], data[:, 4]


def read_grav(path):
    """读阶段C重力标定 CSV（theta,g）→ 插值函数。"""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data[None, :]
    g_theta, g_tau = data[:, 0], data[:, 1]

    def interp(theta):
        return np.interp(theta, g_theta, g_tau)
    return interp


def regress_round(theta, omega, tau, v, theta_c, amp, g_est=None):
    """单回合直接回归 → (b, fc, n, r2)。

    扣重力后逐样本回归 τ_res = b·ω + fc·sign(ω)：
      - τ_res = τ − g_est(θ)：阶段C 已实测重力，逐样本扣掉（无需同 θ 配对消重力）
      - 滤 |ω| < W_MIN：剔除换向/静止区的 sign 抖动
      - 回归矩阵 [ω, sign(ω)] → b(粘性 N·m·s/rad), fc(库仑 N·m)
    ⚠ 相比旧"稳态配对"法：位置斜坡+PD 跟踪下实际速度有纹波，稳态窗只捕获 2~4% 且
    正反向在 θ 上踩不齐 → 配对恒空。直接回归用全部中高速样本，抗纹波、样本量大。
    返回 b/fc 估计、有效样本数 n、线性拟合 R²。
    """
    tau = tau - g_est(theta) if g_est is not None else tau
    ok = np.abs(omega) > W_MIN            # 滤低速/换向 sign 抖动
    X = np.column_stack([omega[ok], np.sign(omega[ok])])
    Y = tau[ok]
    if len(Y) < 30:
        return None
    sol, *_ = np.linalg.lstsq(X, Y, rcond=None)
    b_hat, fc_hat = float(sol[0]), float(sol[1])
    pred = X @ sol
    ss_res = float(np.sum((Y - pred) ** 2))
    ss_tot = float(np.sum((Y - Y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0
    return b_hat, fc_hat, len(Y), r2


def identify_joint(cp, mi, n_rounds=4):
    """读某关节全部回合 → 每回合独立回归 → b/fc 均值±σ。"""
    prefix = f"C{cp}M{mi}"
    info = read_info(os.path.join(DATA_DIR, f"{prefix}_info.txt"))
    theta_c, amp = info["theta_c"], info["amp"]
    # 阶段C 重力标定插值（先扣重力减少 bin 内 Δg 污染）
    grav_path = os.path.join(DATA_DIR, f"{prefix}_grav.csv")
    g_est = read_grav(grav_path) if os.path.exists(grav_path) else None

    bs, fcs, r2s, npairs = [], [], [], []
    for r in range(n_rounds):
        path = os.path.join(DATA_DIR, f"{prefix}_R{r}.csv")
        if not os.path.exists(path):
            continue
        theta, omega, tau, v = read_csv(path)
        res = regress_round(theta, omega, tau, v, theta_c, amp, g_est)
        if res is None:
            continue
        b, fc, n, r2 = res
        bs.append(max(b, 0.0))
        fcs.append(max(fc, 0.0))
        r2s.append(r2)
        npairs.append(n)
    return bs, fcs, r2s, npairs, theta_c, amp


def main_real():
    print("=" * 70)
    print(f"真机摩擦辨识离线回归  数据目录: {DATA_DIR}")
    print("=" * 70)
    files = sorted(glob.glob(os.path.join(DATA_DIR, "C*M*_info.txt")))
    if not files:
        print("[ERROR] 未找到 log/fric_id/C*M*_info.txt（先跑 Example54 落盘）")
        print("        可用 --demo 自测管线。")
        return 1

    results = {}
    for f in files:
        name = os.path.basename(f).replace("_info.txt", "")
        cp = int(name[1])
        mi = int(name[3])
        bs, fcs, r2s, npairs, theta_c, amp = identify_joint(cp, mi)
        results[name] = (bs, fcs, r2s, npairs, theta_c, amp)

    print(f"\n{'关节':<8}{'θ_c':>7}{'amp':>6}  "
          f"{'b±σ':>18}{'fc±σ':>18}{'R²':>7}  质量")
    print("-" * 70)
    ok_all = True
    for name in sorted(results):
        bs, fcs, r2s, npairs, theta_c, amp = results[name]
        if not bs:
            print(f"{name:<8}{theta_c:>7.2f}{amp:>6.2f}  {'无有效回合数据':>18}")
            ok_all = False
            continue
        b_m, fc_m = float(np.mean(bs)), float(np.mean(fcs))
        b_s, fc_s = float(np.std(bs)), float(np.std(fcs))
        r2 = float(np.mean(r2s))
        # 质量：σ/均值 < 30% 且 R² > 0.7
        q = (b_m > 0 and fc_m > 0 and
             b_s / b_m < SIGMA_RATIO_MAX and fc_s / fc_m < SIGMA_RATIO_MAX and
             r2 > 0.7)
        ok_all &= q
        print(f"{name:<8}{theta_c:>7.2f}{amp:>6.2f}  "
              f"{b_m:>7.4f}±{b_s:<7.4f}{fc_m:>7.4f}±{fc_s:<7.4f}"
              f"{r2:>7.2f}  {'OK' if q else '⚠'}")
    print("-" * 70)
    print("质量 OK：σ/均值<30% 且 τ_diff 线性 R²>0.7")
    print("\n[INFO] 将 b/fc 填入 rl_controller.cpp 的 LEG_FF_FV[12]/LEG_FF_FC[12]")
    print("       （注意 CSV 是 CAN 顺序 per-leg: hip/thigh/calf，POLICY 顺序同为")
    print("        FL_hip,FL_thigh,FL_calf,FR_hip,...，两者一致）")
    return 0 if ok_all else 2


# =====================================================================
# --demo 自测：生成模拟真机落盘数据，验证回归管线能恢复已知 b/fc
# =====================================================================
def demo_generate():
    """生成一个关节的模拟落盘数据（已知 b/fc + 真实量级重力 + 噪声）。"""
    os.makedirs(DATA_DIR, exist_ok=True)
    rng = np.random.RandomState(42)
    b_true, fc_true = 0.50, 0.20
    theta_c, amp = 0.0, 0.35
    vels = [0.3, 0.5, 0.8]
    cyc = 5
    dt = 0.002
    tau_noise = 0.05

    with open(os.path.join(DATA_DIR, "C0M1_info.txt"), "w") as f:
        f.write(f"theta_c={theta_c:.6f}\namp={amp:.6f}\n")
        f.write(f"n_vel={len(vels)}\n")
        for i, v in enumerate(vels):
            f.write(f"vel{i}={v:.3f}\n")

    # 重力 g(θ)（量级参考 sim：hip 6~15 Nm）
    def g(theta):
        return 10.0 + 8.0 * theta

    # 阶段C 重力标定落盘（模拟：20 点采样；噪声对应真机 4 遍平均后的残差）
    gpts = np.linspace(theta_c - amp, theta_c + amp, 20)
    gnoise = rng.randn(20) * 0.03
    np.savetxt(os.path.join(DATA_DIR, "C0M1_grav.csv"),
               np.column_stack([gpts, g(gpts) + gnoise]), delimiter=",",
               header="theta,g", comments="")

    for r in range(4):
        rows = []
        for v in vels:
            n_scan = int(2 * amp / v / dt)
            pos = theta_c - amp
            dir = +1.0
            for cyc_ in range(cyc * 2):
                for s in range(n_scan):
                    pos = np.clip(pos + v * dir * dt, theta_c - amp, theta_c + amp)
                    omega = v * dir + rng.randn() * 0.03      # 恒速 + 微小波动
                    theta = pos + rng.randn() * 0.002         # 位置反馈噪声
                    tau = (g(theta) + b_true * omega
                           + fc_true * np.sign(omega) + rng.randn() * tau_noise)
                    rows.append((0.0, theta, omega, tau, v * dir))
                dir = -dir
        arr = np.array(rows)
        np.savetxt(os.path.join(DATA_DIR, f"C0M1_R{r}.csv"), arr, delimiter=",",
                   header="elapsed_ms,theta,omega,tau,v_des", comments="")
    print(f"[demo] 已生成模拟数据 → {DATA_DIR}/C0M1_*（b_true={b_true}, fc_true={fc_true}）")


def main_demo():
    demo_generate()
    print("=" * 70)
    print("自测：模拟数据回归（应恢复 b=0.50, fc=0.20）")
    print("=" * 70)
    bs, fcs, r2s, npairs, theta_c, amp = identify_joint(0, 1)
    if not bs:
        print("[FAIL] 无有效数据")
        return 1
    print(f"θ_c={theta_c:.2f} amp={amp:.2f} 回合数={len(bs)}")
    print(f"b  = {np.mean(bs):.4f} ± {np.std(bs):.4f}  (真值 0.50)")
    print(f"fc = {np.mean(fcs):.4f} ± {np.std(fcs):.4f}  (真值 0.20)")
    print(f"R² = {np.mean(r2s):.3f}")
    ok = (abs(np.mean(bs) - 0.50) < 0.05 and abs(np.mean(fcs) - 0.20) < 0.03)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="真机摩擦辨识离线回归")
    parser.add_argument("--demo", action="store_true", help="生成模拟数据自测管线")
    args = parser.parse_args()
    raise SystemExit(main_demo() if args.demo else main_real())
