"""复刻「转弯刮地 → 髋外摆回不去」的诊断脚本。

真机现象：纯 yaw 转弯时轮子侧向刮地，把腿往外推（髋外摆）；转弯结束回到
0 指令后，接地轮的侧向摩擦让髋「收不动」，实际外摆回不到 default。

本脚本在原生 MuJoCo 中复现该现象，用于方案 A（改奖励）前的基准验证：
  - 命令序列：先纯 yaw 转弯 TURN_STEPS 步，再 0 指令停止 STOP_STEPS 步；
  - 记录每步髋位置相对 default 的偏差（policy order 的 hip 索引 0/3/6/9）；
  - 记录轮子侧向速度（机体 y 分量，即「刮地」量）；
  - 输出转弯阶段 / 停止阶段的髋偏差，判断「停止后是否回不去」。

用法：
    conda activate MJX && cd dogurdf_sim2sim_deploy
    python src/turn_reproduce.py --yaw 1.0 --friction 1.36
    python src/turn_reproduce.py --yaw 1.0 --friction 2.5   # 扫高摩擦复现外摆

摩擦：MuJoCo 用各向同性滑动摩擦（sliding μ）。真机轮胎侧向摩擦可能大于
纵向，故用「调大 μ」近似「侧向更黏、更刮地」。
"""

from __future__ import annotations

import argparse
import os
import sys

import jax
import jax.numpy as jnp
import mujoco
import numpy as np

os.environ.setdefault("JAX_PLATFORMS", "cpu")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sim2sim import (  # noqa: E402
    CONTROL_DT,
    DECIMATION,
    LEG_KD,
    LEG_KP,
    NUM_JOINTS,
    NUM_LEG_JOINTS,
    NUM_WHEELS,
    WHEEL_KD,
    WHEEL_KP,
    JointIndexer,
    _build_observation,
    _compute_torques_policy,
    _default_pose_policy,
    _load_params,
    _policy_joint_limits,
    _reset_pose,
)
from cfg.experiments import get_experiment  # noqa: E402
from networks import ActorCriticMLP  # noqa: E402

# policy order 的 hip 索引（FL/FR/RL/RR）
HIP_POLICY_IDX = [0, 3, 6, 9]
LEG_NAMES = ["FL", "FR", "RL", "RR"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=str,
                    default="../checkpoints/dogurdf_velocity/iteration_450.pkl")
    ap.add_argument("--friction", type=float, default=1.36,
                    help="滑动摩擦系数 sliding μ（默认 1.36；地垫可 1.5~2.5）")
    ap.add_argument("--friction_torsional", type=float, default=0.005,
                    help="扭转摩擦系数（默认 0.005；软地垫轮胎陷进去，调大如 0.2~1.0）")
    ap.add_argument("--friction_rolling", type=float, default=0.0001,
                    help="滚动摩擦系数（默认 0.0001；软地垫滚动阻力，调大如 0.02~0.1）")
    ap.add_argument("--yaw", type=float, default=1.0, help="转弯 yaw 指令 rad/s")
    ap.add_argument("--turn_steps", type=int, default=150, help="转弯步数（3s）")
    ap.add_argument("--stop_steps", type=int, default=150, help="停止步数（3s）")
    ap.add_argument("--out", type=str, default="", help="可选：把髋偏差序列存为 .npy")
    args = ap.parse_args()

    ckpt = args.checkpoint if os.path.isabs(args.checkpoint) \
        else os.path.abspath(os.path.join(os.path.dirname(__file__), args.checkpoint))

    spec = get_experiment("dogurdf_velocity")
    ppo_cfg = spec.build_ppo_config()
    mj_model = mujoco.MjModel.from_xml_path(str(spec.robot.asset_path))
    mj_data = mujoco.MjData(mj_model)

    # 覆盖摩擦三参数（sliding / torsional / rolling）。
    # 地垫复刻要点：软垫让轮胎「陷进去」，扭转/滚动阻力大，侧向刮地更严重，
    # 应重点调大 torsional / rolling，而非只调 sliding。
    mj_model.geom_friction[:] = [
        args.friction, args.friction_torsional, args.friction_rolling
    ]
    print(f"friction = sliding {args.friction} / torsional "
          f"{args.friction_torsional} / rolling {args.friction_rolling}")

    indexer = JointIndexer(mj_model)
    default_pose = _default_pose_policy()
    lower, upper = _policy_joint_limits()

    kp = np.array([LEG_KP] * NUM_LEG_JOINTS + [WHEEL_KP] * NUM_WHEELS)
    kd = np.array([LEG_KD] * NUM_LEG_JOINTS + [WHEEL_KD] * NUM_WHEELS)
    torque_limit = np.array([150.0] * NUM_LEG_JOINTS + [53.0] * NUM_WHEELS)

    _reset_pose(mj_model, mj_data, indexer, default_pose)

    network = ActorCriticMLP(
        action_size=NUM_JOINTS,
        actor_hidden_dims=tuple(ppo_cfg.network.policy_hidden_layer_sizes),
        critic_hidden_dims=tuple(ppo_cfg.network.value_hidden_layer_sizes),
        init_noise_std=ppo_cfg.network.init_noise_std,
        activation=ppo_cfg.network.activation,
    )
    params = _load_params(ckpt)
    use_phase = int(params["params"]["actor_mlp_0"]["kernel"].shape[0]) == 64

    @jax.jit
    def policy_mean(p, obs):
        return network.apply(p, obs, method=network.act)

    def run_step(command, last_action, step):
        obs = _build_observation(mj_data, indexer, default_pose, last_action,
                                 command, step, use_phase=use_phase)
        action = np.asarray(policy_mean(params, jnp.asarray(obs[None]))[0])
        action = np.where(np.isfinite(action), action, 0.0)
        for _ in range(DECIMATION):
            jp = indexer.joint_pos(mj_data)
            jv = indexer.joint_vel(mj_data)
            tau = _compute_torques_policy(action, jp, jv, default_pose,
                                          lower, upper, kp, kd, torque_limit)
            mj_data.qfrc_applied[:] = 0.0
            mj_data.qfrc_applied[indexer.dof_adr] = tau
            mujoco.mj_step(mj_model, mj_data)
        return action

    # 轮子侧向速度（机体 y 分量）：先求基座线速度与姿态，逐轮算世界速度→机体坐标
    def wheel_lateral_vel():
        # 轮子 body 是世界系，取其 cvel 线速度，减基座线速度后转机体坐标，取 y
        base_vel = mj_data.qvel[0:3].copy()
        base_quat = mj_data.qpos[3:7].copy()
        # 简化：用 mujoco 直接读每个 wheel body 的世界线速度
        from robots.dogurdf import LEG_PREFIXES
        lat = []
        for leg in LEG_PREFIXES:
            bid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY,
                                    f"{leg}_wheel_link")
            vel_world = mj_data.cvel[bid][3:6].copy()  # 世界线速度
            # 世界 → 机体：四元数逆旋转
            vel_body = _rotate(vel_world, _quat_inv(base_quat))
            rel = vel_body - base_vel
            lat.append(rel[1])  # 机体 y = 侧向
        return np.array(lat)

    last_action = np.zeros(NUM_JOINTS)
    hip_traj = []
    lateral_traj = []

    # ---- 阶段 1：纯 yaw 转弯 ----
    turn_cmd = np.array([0.0, 0.0, args.yaw])
    for step in range(args.turn_steps):
        last_action = run_step(turn_cmd, last_action, step)
        jp = indexer.joint_pos(mj_data)
        hip_traj.append((jp[HIP_POLICY_IDX] - default_pose[HIP_POLICY_IDX]).copy())
        lateral_traj.append(wheel_lateral_vel().copy())

    # ---- 阶段 2：0 指令停止 ----
    stop_cmd = np.array([0.0, 0.0, 0.0])
    for step in range(args.stop_steps):
        last_action = run_step(stop_cmd, last_action, args.turn_steps + step)
        jp = indexer.joint_pos(mj_data)
        hip_traj.append((jp[HIP_POLICY_IDX] - default_pose[HIP_POLICY_IDX]).copy())
        lateral_traj.append(wheel_lateral_vel().copy())

    hip_traj = np.array(hip_traj)
    lateral_traj = np.array(lateral_traj)

    # ---- 诊断输出 ----
    turn = slice(0, args.turn_steps)
    stop = slice(args.turn_steps, args.turn_steps + args.stop_steps)

    print("\n========== 髋外摆诊断 ==========")
    print(f"转弯阶段：髋偏差均值 |dev| = {np.abs(hip_traj[turn]).mean():.4f} rad"
          f"（峰值 {np.abs(hip_traj[turn]).max():.4f}）")
    print(f"  轮子侧向速度 |v_lat| 均值 = {np.abs(lateral_traj[turn]).mean():.4f} m/s"
          f"（峰值 {np.abs(lateral_traj[turn]).max():.4f}）")
    dev_start = np.abs(hip_traj[args.turn_steps]).mean()
    dev_end = np.abs(hip_traj[-1]).mean()
    print(f"停止阶段：起始髋偏差 |dev| = {dev_start:.4f} rad"
          f"  →  结束 {dev_end:.4f} rad")
    print(f"  停止后偏差变化 = {dev_end - dev_start:+.4f} rad"
          f"（>0 = 回不去/继续外摆）")
    print(f"  停止后轮子侧向速度 |v_lat| 均值 = "
          f"{np.abs(lateral_traj[stop]).mean():.4f} m/s")
    print(f"\n  各腿最终髋偏差: " + "  ".join(
        f"{LEG_NAMES[i]}={hip_traj[-1, i]:+.4f}" for i in range(4)))

    if args.out:
        np.save(args.out, hip_traj)
        print(f"  髋偏差序列已存: {args.out}")

    return 0


def _quat_inv(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def _rotate(v, q):
    # q 旋转向量 v（brax 约定，与 sim2sim.py 一致）
    qw, qx, qy, qz = q
    s = 2 * qw * qw - 1
    cx = qy * v[2] - qz * v[1]
    cy = qz * v[0] - qx * v[2]
    cz = qx * v[1] - qy * v[0]
    w2 = qw * 2
    d = (qx * v[0] + qy * v[1] + qz * v[2]) * 2
    return np.array([
        v[0] * s - cx * w2 + qx * d,
        v[1] * s - cy * w2 + qy * d,
        v[2] * s - cz * w2 + qz * d,
    ])


if __name__ == "__main__":
    raise SystemExit(main())
