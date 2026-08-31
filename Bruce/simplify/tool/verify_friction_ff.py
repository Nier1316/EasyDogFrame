#!/usr/bin/env python3
"""摩擦前馈辨识算法验证（sim2sim，参考 project_2 / project_5 / real_robot_identification_plan）。

验证「辨识摩擦参数 [b, fc]」的两条路径：

方案 A（合成验证，参考 project_2_friction_identification）：
   friction_target 用真值公式直接算 b·dq + fc·sign(dq)（加噪声）→ 验证最小二乘
   算法在 dogurdf 关节运动分布下数学正确（不碰真机提取问题）。

方案 B（重力抵消法，参考 real_robot_identification_plan.md §核心思路）：
   真机可实施的辨识协议。真机只能测到 τ_motor = 惯性 + 科氏 + 重力 + 摩擦 的总和，
   τ_friction 无法直接测量。利用「同一关节角度下正反向重力矩相同」：
       τ(+θ̇) = τ_grav(θ) + b·θ̇ + fc
       τ(−θ̇) = τ_grav(θ) − b·θ̇ − fc
       相减：  τ_diff = 2b·θ̇ + 2fc     ← 重力项被完全消去
   前提：匀速(θ̈≈0)消惯量、其他关节锁定消科氏、悬空固定 base 消接触/支撑。
   实现（sim 验证）：真机速度模式恒速时电机输出保持扭矩 = 重力 g(θ) + 摩擦（恒速
   θ̈=0、科氏=0）。在 sim 中直接构造该恒速状态，用 mj_rne 算【真实】重力 g(θ)（保留
   腿自重重力，非置零），叠加上注入的真值摩擦 + 扭矩噪声，得「理想恒速电机测量」：
       τ_motor(+v) = g(θ) + b·v + fc,  τ_motor(−v) = g(θ) − b·v − fc
   同 θ 分箱配对相减消重力 → τ_diff(v) = 2b·v + 2fc → 多档线性回归 [b, fc]。
   对照组：不抵消重力、直接用 τ_motor 回归 → 定量展示腿自重重力 g(θ) 对 [b, fc] 的污染
   （dogurdf 一条腿 ~11.5kg，hip 静止重力矩 ~11 Nm，远超摩擦 ~0.5 Nm）。

用法：
    /home/sysu/miniconda3/envs/MJX/bin/python tool/verify_friction_ff.py
"""

from __future__ import annotations

import os
import re

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
XML = os.path.join(PROJECT, "dogurdf_sim2sim_deploy/assets/bot_model/dogurdf/dogurdf.xml")

LEG_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
]
N_ACTUATOR = 12
FREE_DOF = 6
K_TANH = 100.0        # tanh 光滑参数（回归基函数）
LAM = 1e-4            # Tikhonov 正则化

# ---- 方案 B 参数（重力抵消法）----
VEL_STEPS = [0.3, 0.5, 0.8, 1.2]   # 双向恒速档位 (rad/s)，真机 plan 建议 0.3~2.0
AMP = {"hip": 0.35, "thigh": 0.50, "calf": 0.40}   # 扫掠半幅 (rad)，限位内
N_BIN = 10                      # 关节角 θ 分箱数
N_SWEEP = 40                    # 每方向时序扫掠步数
TAU_NOISE = 0.05                # 扭矩反馈噪声 (Nm)，真机电流环换算量级


def load_env():
    """加载 dogurdf，并把 base(freejoint) 固定为悬空架（torso 焊死，消除强制清零冲击）。"""
    with open(XML) as f:
        xml = f.read()
    # 修正 mesh 相对路径为绝对路径（from_xml_string 无法解析相对 meshdir）
    xml = xml.replace('meshdir="./meshes"',
                      f'meshdir="{os.path.dirname(XML)}/meshes"')
    if "<freejoint/>" in xml:
        # 删除 freejoint：无 joint 的 body 固定在世界（等价悬空架固定 base）
        xml = xml.replace("<freejoint/>", "")
        # 同步修正 keyframe home 的 qpos（去掉 free 的 7 个值，剩 16 关节）
        xml = re.sub(
            r'(<key name="home"\s+qpos=")[^"]*(")',
            r'\g<1>0 0.20 -0.70 0 0 0.20 -0.70 0 '
            r'0 0.20 -0.70 0 0 0.20 -0.70 0\g<2>',
            xml, flags=re.DOTALL)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    leg_dofs = np.array([
        m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n + "_joint")]
        for n in LEG_NAMES
    ])
    # 腿 actuator 索引（CAN 顺序 per-leg: hip/thigh/calf/wheel）
    leg_act = np.array([(i // 3) * 4 + (i % 3) for i in range(N_ACTUATOR)])
    leg_qadrs = np.array([
        m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n + "_joint")]
        for n in LEG_NAMES
    ])
    return m, d, leg_dofs, leg_act, leg_qadrs


def inject_true_friction(m, leg_dofs):
    """注入【已知】摩擦真值（每关节不同，模拟真实差异）。"""
    b_true = np.array([0.50, 0.55, 0.45, 0.52, 0.48, 0.50,
                       0.47, 0.53, 0.46, 0.51, 0.49, 0.44])
    fc_true = np.array([0.20, 0.25, 0.18, 0.22, 0.24, 0.19,
                        0.21, 0.26, 0.17, 0.23, 0.25, 0.20])
    m.dof_damping[leg_dofs] = b_true
    m.dof_frictionloss[leg_dofs] = fc_true
    return b_true, fc_true


def setup_pose(m, d, leg_qadrs):
    """初始站立姿态（base 已固定为悬空架）。"""
    mujoco.mj_resetData(m, d)
    for k, n in enumerate(LEG_NAMES):
        jt = n.split("_")[1]
        d.qpos[leg_qadrs[k]] = {"hip": 0.0, "thigh": 0.20, "calf": -0.35}[jt]
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)


def identify(W, Y, n_joints):
    """最小二乘 + Tikhonov 正则化，clip 非负（参考 friction_identification.py）。"""
    A = W.T @ W + LAM * np.eye(2 * n_joints)
    phi = np.linalg.solve(A, W.T @ Y)
    phi = np.maximum(phi, 0.0)
    return phi[:n_joints], phi[n_joints:], np.linalg.cond(W.T @ W)


def regress(dq, target, noise_ratio):
    """dq/target 都是 (T, 12)；加相对观测噪声后回归 [b, fc]。"""
    rng = np.random.RandomState(7)
    dq_n = dq + rng.randn(*dq.shape) * np.maximum(np.std(dq, 0), 1e-8) * noise_ratio
    Y = (target + rng.randn(*target.shape) * np.maximum(np.std(target, 0), 1e-8) * noise_ratio).reshape(-1)
    T = dq.shape[0]
    W = np.zeros((T * N_ACTUATOR, 2 * N_ACTUATOR))
    for i in range(N_ACTUATOR):
        rows = np.arange(T) * N_ACTUATOR + i
        W[rows, i] = dq_n[:, i]
        W[rows, N_ACTUATOR + i] = np.tanh(K_TANH * dq_n[:, i])
    return identify(W, Y, N_ACTUATOR)


def report(name, b_hat, fc_hat, b_true, fc_true):
    rel_fc = np.linalg.norm(fc_hat - fc_true) / np.linalg.norm(fc_true) * 100
    rel_b = np.linalg.norm(b_hat - b_true) / np.linalg.norm(b_true) * 100
    tag_fc = "PASS" if rel_fc < 5 else "FAIL"
    tag_b = "PASS" if rel_b < 5 else "FAIL"
    print(f"  {name:<46} fc={rel_fc:6.2f}%[{tag_fc}]  b={rel_b:6.2f}%[{tag_b}]")
    return rel_fc, rel_b


def _set_state(m, d, leg_qadrs, leg_dofs, q_hold, i, theta, dq_i):
    """构造恒速状态：目标关节 θ + dq_i，其他关节锁在初始姿态。
    ⚠ fancy indexing 的 d.qpos[idx][:] = ... 是副本写入不生效，必须用 d.qpos[idx] = ..."""
    mujoco.mj_resetData(m, d)
    d.qpos[leg_qadrs] = q_hold
    d.qpos[leg_qadrs[i]] = theta
    d.qvel[leg_dofs] = 0.0
    d.qvel[leg_dofs[i]] = dq_i
    mujoco.mj_forward(m, d)


def _grav_torque(m, d, leg_dofs, i):
    """真实重力保持扭矩 g(θ)（rne bias，flg_acc=0，不含摩擦）。"""
    bias = np.zeros(m.nv)
    mujoco.mj_rne(m, d, 0, bias)
    return bias[leg_dofs[i]]


def gravity_cancel_identify(m, d, leg_dofs, leg_act, leg_qadrs, b_true, fc_true):
    """方案 B：理想恒速数据 + 重力抵消辨识。

    真机速度模式恒速时，电机输出保持扭矩 = 重力 g(θ) + 摩擦（恒速 → θ̈=0、科氏=0）。
    在 sim 中直接构造该状态（qvel=v·sgn，其他关节锁定），用 mj_rne 算真实重力 g(θ)，
    叠加上注入的真值摩擦，加扭矩噪声，即「理想恒速电机测量」数据：
        τ_motor(+v) = g(θ) + b·v + fc
        τ_motor(−v) = g(θ) − b·v − fc
    同 θ 相减消重力：τ_diff = 2b·v + 2fc → 多档线性回归 [b, fc]。

    对照组（不抵消）：± 扫掠 θ 区间不对称（端点偏移 δ，真机正反向扫掠无法精确对称），
    直接回归 τ_motor ~ [v·sgn, sign(v·sgn)]，展示重力 g(θ) 对 [b, fc] 的污染。
    """
    nv = m.nv
    rng = np.random.RandomState(5)

    b_hat = np.zeros(N_ACTUATOR)
    fc_hat = np.zeros(N_ACTUATOR)
    ctrl_tau, ctrl_v = [], []   # 对照组

    setup_pose(m, d, leg_qadrs)
    q_hold = d.qpos[leg_qadrs].copy()

    print(f"  {'关节':<10}{'g(θ)范围(Nm)':>16}{'Δg':>7}   {'摩擦fc+b·1.2':>13}")
    for i in range(N_ACTUATOR):
        jt = LEG_NAMES[i].split("_")[1]
        amp = AMP[jt]
        q_c = q_hold[i]
        lo, hi = q_c - amp, q_c + amp

        # 打印扫掠区间内真实重力矩范围（腿自重重力影响量级）
        gs = []
        for theta in np.linspace(lo, hi, 20):
            _set_state(m, d, leg_qadrs, leg_dofs, q_hold, i, theta, 0.0)
            gs.append(_grav_torque(m, d, leg_dofs, i))
        print(f"  {LEG_NAMES[i]:<10}{min(gs):>8.2f}~{max(gs):>6.2f}"
              f"{max(gs)-min(gs):>7.2f}   {fc_true[i]+b_true[i]*1.2:>13.2f}")

        diffs = []          # (v, tau_diff)
        for v in VEL_STEPS:
            bins_p = [[] for _ in range(N_BIN)]
            bins_n = [[] for _ in range(N_BIN)]
            for sgn in (+1.0, -1.0):
                # 时序扫掠：+v 时 θ 从 lo→hi，−v 时 θ 从 hi→lo
                thetas = (np.linspace(lo, hi, N_SWEEP) if sgn > 0
                          else np.linspace(hi, lo, N_SWEEP))
                for theta in thetas:
                    _set_state(m, d, leg_qadrs, leg_dofs, q_hold, i, theta, v * sgn)
                    g_i = _grav_torque(m, d, leg_dofs, i)
                    fric = b_true[i] * (v * sgn) + fc_true[i] * np.sign(v * sgn)
                    tau_m = g_i + fric + rng.randn() * TAU_NOISE
                    b_ = int((theta - lo) / (2 * amp) * N_BIN)
                    b_ = min(max(b_, 0), N_BIN - 1)
                    if sgn > 0:
                        bins_p[b_].append(tau_m)
                    else:
                        bins_n[b_].append(tau_m)
            # 同 θ 配对相减 → τ_diff(v)
            diff_vals = [np.mean(p) - np.mean(n)
                         for p, n in zip(bins_p, bins_n) if p and n]
            diffs.append((v, np.mean(diff_vals)))
            print(f"  {LEG_NAMES[i]:<10} v={v:<4.1f} τ_diff={np.mean(diff_vals):+.3f}")

        # 回归 τ_diff = 2b·v + 2fc
        X = np.array([[2 * v, 2.0] for v, _ in diffs])
        Y = np.array([td for _, td in diffs])
        sol = np.linalg.lstsq(X, Y, rcond=None)[0]
        b_hat[i], fc_hat[i] = np.maximum(sol[0], 0.0), np.maximum(sol[1], 0.0)

    # 对照组：± 扫掠区间不对称（−v 端点偏移 δ，模拟真机正反向扫掠无法精确对称），
    # 不配对直接回归 → 重力 g(θ) 变化泄漏进 [b, fc]
    DELTA = 0.08   # 扫掠端点不对称偏移 (rad)
    for i in range(N_ACTUATOR):
        amp = AMP[LEG_NAMES[i].split("_")[1]]
        q_c = q_hold[i]
        for v in VEL_STEPS:
            for sgn in (+1.0, -1.0):
                off = 0.0 if sgn > 0 else DELTA
                lo = q_c - amp + off
                hi = q_c + amp + off
                thetas = (np.linspace(lo, hi, N_SWEEP) if sgn > 0
                          else np.linspace(hi, lo, N_SWEEP))
                for theta in thetas:
                    _set_state(m, d, leg_qadrs, leg_dofs, q_hold, i, theta, v * sgn)
                    g_i = _grav_torque(m, d, leg_dofs, i)
                    fric = b_true[i] * (v * sgn) + fc_true[i] * np.sign(v * sgn)
                    tau_m = g_i + fric + rng.randn() * TAU_NOISE
                    ctrl_tau.append(tau_m)
                    ctrl_v.append(v * sgn)

    tau_arr = np.array(ctrl_tau)
    v_arr = np.array(ctrl_v)
    Wc = np.hstack([v_arr[:, None], np.sign(v_arr)[:, None]])
    solc = np.linalg.lstsq(Wc, tau_arr, rcond=None)[0]
    b_ctrl, fc_ctrl = solc[0], solc[1]

    return b_hat, fc_hat, b_ctrl, fc_ctrl


def main() -> int:
    m, d, leg_dofs, leg_act, leg_qadrs = load_env()
    b_true, fc_true = inject_true_friction(m, leg_dofs)
    setup_pose(m, d, leg_qadrs)
    dt = m.opt.timestep

    # ============ 方案 A：构造 dq 数据 + 合成 friction_target ============
    # 纯算法验证（project_2 复刻）：target 用真值公式，与动力学/重力无关。
    # dq 直接用多频正弦组合构造（丰富速度分布 + 过零），不依赖仿真环境。
    T = 2000
    ta = np.arange(T) * 0.002
    dq_a = np.zeros((T, N_ACTUATOR))
    for i in range(N_ACTUATOR):
        dq_a[:, i] = (0.6 * np.sin(2 * np.pi * 0.5 * ta + 0.7 * i)
                      + 0.4 * np.sin(2 * np.pi * 2.0 * ta + 1.3 * i)
                      + 0.25 * np.sin(2 * np.pi * 5.0 * ta + 2.1 * i))
    target_a = (b_true[np.newaxis, :] * dq_a
                + fc_true[np.newaxis, :] * np.sign(dq_a))
    b_hat, fc_hat, cond_a = regress(dq_a, target_a, 0.01)
    print("=" * 78)
    print(f"方案 A：构造 dq（多频正弦）+ 合成 friction_target（真值公式），噪声 1%")
    print(f"  cond(W'W) = {cond_a:.2e}")
    report("方案A fc/b 相对误差", b_hat, fc_hat, b_true, fc_true)

    # ============ 方案 B：重力抵消法（理想恒速数据） ============
    print("\n" + "=" * 78)
    print("方案 B：重力抵消法（保留重力 + 理想恒速 + 同 θ 配对相减）")
    print("  原理：τ_diff = τ(+θ̇) − τ(−θ̇) = 2b·θ̇ + 2fc，同一角度下重力项相消")
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT   # 悬空（无地面接触）
    b_gc, fc_gc, b_ctrl, fc_ctrl = gravity_cancel_identify(
        m, d, leg_dofs, leg_act, leg_qadrs, b_true, fc_true)
    print("\n  ==== 重力抵消法辨识结果 vs 真值 ====")
    print(f"  {'关节':<12}{'fc真值':>8}{'fc辨识':>9}{'fc误%':>8}   {'b真值':>7}{'b辨识':>8}{'b误%':>7}")
    for i in range(N_ACTUATOR):
        print(f"  {LEG_NAMES[i]:<12}{fc_true[i]:>8.3f}{fc_gc[i]:>9.3f}"
              f"{abs(fc_gc[i]-fc_true[i])/fc_true[i]*100:>7.1f}%   "
              f"{b_true[i]:>7.3f}{b_gc[i]:>8.3f}{abs(b_gc[i]-b_true[i])/b_true[i]*100:>6.1f}%")
    print()
    report("重力抵消法  fc/b 相对误差", b_gc, fc_gc, b_true, fc_true)
    report("对照组(不抵消重力) fc/b", np.array([b_ctrl]), np.array([fc_ctrl]),
           np.array([np.mean(b_true)]), np.array([np.mean(fc_true)]))
    print("""
结论：
  方案 A 验证最小二乘算法数学正确（合成 target）。
  方案 B 复现 real_robot_identification_plan.md 的真机辨识协议（双向恒速 + 重力抵消），
    在 sim 中保留重力、悬空架场景下逐关节辨识，验证协议能恢复注入的摩擦真值。
    对照组（不抵消重力直接回归）展示腿自重重力对 [b, fc] 的污染量级，
    证明「重力抵消」是真机辨识的必要步骤。""")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
