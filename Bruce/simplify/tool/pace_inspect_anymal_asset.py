#!/usr/bin/env python3
"""在 Isaac Lab 里加载 ANYmal-D 资产并打印物理参数（不训练、不渲染、不控制）。

用途：PACE Phase 0 的"资产可比性"核对 —— 拿到 NVIDIA USD 的真实 body 质量/惯量/关节限位，
与上游 ANYbotics `anymal_d_simple_description`（commit d5bf4dc）的 URDF 声明值逐项对照。
见 docs/PACE_PHASE0_PARITY.md §2.3/§5。

⚠️ 本脚本**只用于 PACE Phase 0 的仿真器调研**，需要 Isaac Lab 环境；与狗的控制代码无关。

用法（首次运行会从 NVIDIA Nucleus 下载 USD，且会编译内核，可能十几分钟）：

    /home/sysu/miniconda3/envs/env_isaaclab/bin/python \
        tool/pace_inspect_anymal_asset.py 2>&1 | tee /tmp/anymal_asset.txt

输出（尽量紧凑、逐段 flush，便于后台观察）：
  1) 关节名/body 名与数量（用于与 PACE 的 joint_order 比对）
  2) body 质量：总计 + 最重的若干个（用于与 URDF 的 57.0279 kg 对照）
  3) 12 个关节的限位（与 URDF 的 ±0.785/±0.611、HFE/KFE ±9.42 对照）
  4) 基座是否焊死（fix_root_link）
  5) Isaac 默认执行器参数（PACE 会用 PaceDCMotorCfg 覆盖，这里只为记录差异）
"""

from __future__ import annotations

import sys


def log(*a):
    print(*a, flush=True)


def main() -> int:
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=True)
    simulation_app = app_launcher.app

    import torch  # noqa: F401
    import numpy as np
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab_assets.robots.anymal import ANYMAL_D_CFG

    log("=" * 78)
    log("ANYmal-D 资产核查（Isaac Lab）")
    log("=" * 78)

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.0025, device="cuda:0"))
    sim.set_camera_view([2.0, 2.0, 1.5], [0.0, 0.0, 0.8])

    robot_cfg = ANYMAL_D_CFG.replace(prim_path="/World/Robot")

    # ---- 4) 基座是否焊死（先打印，因为它决定后面数据的解读方式）----
    try:
        fx = robot_cfg.spawn.articulation_props.fix_root_link
    except Exception as e:  # noqa: BLE001
        fx = f"<读取失败: {e}>"
    log(f"\n[基座] ANYMAL_D_CFG.spawn.articulation_props.fix_root_link = {fx}")

    log("\n[USD] 正在创建 articulation（首次会从 Nucleus 下载 USD，请耐心）...")
    robot = Articulation(robot_cfg)
    sim.reset()
    log("[USD] 加载完成")

    # ---- 1) 名称与数量 ----
    joint_names = list(robot.joint_names)
    body_names = list(robot.body_names)
    log(f"\n[1] 关节数 = {robot.num_joints}，body 数 = {robot.num_bodies}")
    log(f"    关节名（按 Isaac Lab 顺序）: {joint_names}")
    log(f"    body 名: {body_names}")

    # PACE 的 joint_order（官方 anymal_pace_env_cfg.py:36）—— 用于比对顺序
    pace_joint_order = [
        "LF_HAA", "LF_HFE", "LF_KFE",
        "RF_HAA", "RF_HFE", "RF_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE",
        "RH_HAA", "RH_HFE", "RH_KFE",
    ]
    same = joint_names == pace_joint_order
    log(f"    与 PACE joint_order 完全一致? {same}")
    if not same:
        log(f"    PACE joint_order = {pace_joint_order}")
        log(f"    缺失: {[j for j in pace_joint_order if j not in joint_names]}")
        log(f"    多余: {[j for j in joint_names if j not in pace_joint_order]}")

    # ---- 2) 质量 ----
    try:
        masses = robot.root_physx_view.get_masses()   # (num_instances, num_bodies)
        masses = masses[0].detach().cpu().numpy()
        total = float(masses.sum())
        log(f"\n[2] 总质量 = {total:.4f} kg   （上游 URDF 声明合计 = 57.0279 kg；"
            f"MuJoCo 编译该 URDF 得 30.4490 kg）")
        order = np.argsort(-masses)[:15]
        log("    最重的 body:")
        for i in order:
            nm = body_names[i] if i < len(body_names) else f"<body{i}>"
            if masses[i] > 0:
                log(f"      {nm:34s} {masses[i]:8.4f} kg")
        # 基座（通常是第 0 个 body）
        log(f"    body[0] = {body_names[0] if body_names else '?'} 质量 = {masses[0]:.4f} kg")
    except Exception as e:  # noqa: BLE001
        log(f"\n[2] 读取质量失败: {type(e).__name__}: {e}")

    # ---- 2b) 惯量（可选）----
    try:
        inertias = robot.root_physx_view.get_inertias()[0].detach().cpu().numpy()
        log(f"    惯量矩阵 shape = {inertias.shape}；body[0] 惯量对角 = "
            f"{np.diag(inertias[0]) if inertias.ndim == 3 else inertias[0][:3]}")
    except Exception as e:  # noqa: BLE001
        log(f"    读取惯量失败（不影响主结论）: {type(e).__name__}")

    # ---- 3) 关节限位 ----
    try:
        lim = robot.data.joint_pos_limits[0].detach().cpu().numpy()  # (num_joints, 2)
        log("\n[3] 关节限位 (rad):")
        for i, nm in enumerate(joint_names):
            log(f"      {nm:10s} [{lim[i, 0]:+.4f}, {lim[i, 1]:+.4f}]")
    except Exception as e:  # noqa: BLE001
        log(f"\n[3] 读取关节限位失败: {type(e).__name__}: {e}")

    # ---- 5) Isaac 默认执行器参数（PACE 会覆盖，仅记录）----
    log("\n[5] ANYMAL_D_CFG.actuators（PACE 会用 PaceDCMotorCfg 覆盖，此处仅记录）:")
    try:
        for act in robot_cfg.actuators:
            for k in ("joint_names_expr", "stiffness", "damping", "effort_limit",
                      "velocity_limit", "saturation_effort", "friction",
                      "dynamic_friction", "viscous_friction", "armature"):
                if hasattr(act, k):
                    log(f"      {k:20s} = {getattr(act, k)}")
            log(f"      (class = {type(act).__name__})")
    except Exception as e:  # noqa: BLE001
        log(f"      读取执行器参数失败: {type(e).__name__}: {e}")

    log("\n[完成] 请把以上输出回填 docs/PACE_PHASE0_PARITY.md 的资产 diff 表。")
    sim.clear_instance() if hasattr(sim, "clear_instance") else None
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
