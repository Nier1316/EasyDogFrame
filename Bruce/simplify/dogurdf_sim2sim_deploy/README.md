# dogurdf 轮足四足 · sim2sim 部署包

**对角交替迈步原地转弯策略 · 站高 0.45m · 原生 MuJoCo 仿真验证**

本包是训练好的 dogurdf 轮足策略（16 DoF，MJX 训练）在**原生 MuJoCo** 环境中的离线部署验证工具。策略已通过仿真验收（对角步态 + yaw 跟踪达标），可直接用于**真实机器人移植前**的行为验证、手感确认和参数标定。

> ⚠️ 当前包内是**纯仿真**（sim2sim）。真实部署需要在 `simplify/` 的 C++ 嵌入式框架（CANET 驱动）中实现控制律、观测对齐和通讯。本文档最后一节是给部署环节的关键约束清单。

---

## 1. 策略卡片（关键信息速览）

| 项 | 值 |
|---|---|
| 机器人 | dogurdf 轮足四足，16 DoF（4×[hip, thigh, calf, wheel]） |
| 训练框架 | MJX/JAX PPO（自研管线），8192 envs |
| **checkpoint** | `checkpoints/dogurdf_velocity/iteration_1000.pkl`（traj_v7） |
| 观测维度 | **64**（含 gait-phase 时钟） |
| 动作维度 | 16（12 腿位置目标 + 4 轮速目标） |
| **控制频率** | **50 Hz**（控制步 0.02 s；仿真步 0.005 s，decimation=4） |
| 站立高度 | torso 中心 **0.45 m**（default pose: hip=0, thigh=0.20, calf=−0.35） |
| 轮半径 | 0.113 m |
| 关键步态 | 纯 yaw 命令下**对角交替迈步**（左转抬 FL+RR，右转抬 FR+RL），非蹭轮 |

---

## 2. 目录结构

```
dogurdf_sim2sim_deploy/
├── run_sim2sim.sh                    # 一键启动脚本（见 §4）
├── README.md                         # 本文档
├── src/
│   ├── sim2sim.py                    # 主程序：策略推理 + PD 控制 + 手柄 + 录像
│   ├── cfg/                          # 实验配置（站高/奖励/PPO 参数）
│   ├── networks/                     # ActorCriticMLP 网络定义（flax）
│   ├── robots/                       # dogurdf 规格（关节名/限位/扭矩/站高常量）
│   └── tasks/                        # 观测维度规格
├── assets/bot_model/dogurdf/
│   ├── dogurdf.xml                   # MJCF 模型（meshes/ 引用，sim2sim 加载这个）
│   ├── meshes/*.STL                  # 视觉/碰撞网格
│   └── urdf/                         # 原始 URDF（参考，不直接使用）
└── checkpoints/dogurdf_velocity/
    └── iteration_1000.pkl            # 训练好的策略权重（~5MB）
```

---

## 3. 环境依赖

- Python 3.10（conda env `MJX`）
- `jax`, `jaxlib`（CPU 即可跑 sim2sim，GPU 可选）
- `numpy`, `mujoco`, `flax`
- 可选：`brax`（角速度旋转用，实测已引入）、`pygame`（手柄）、`mediapy`（录视频）

> sim2sim 是 **CPU 后端**，不占 GPU，可与训练并行。

---

## 4. 快速开始

```bash
cd dogurdf_sim2sim_deploy

# 1) headless 冒烟测试（默认向前走 1000 步）
./run_sim2sim.sh

# 2) 手柄实时驾驶（推荐：验证转弯手感）
./run_sim2sim.sh --gamepad
#    左摇杆 上下 = 前进/后退（最大 ±1.0 m/s）
#    右摇杆 左右 = 左转/右转（最大 ±1.0 rad/s）
#    摇杆方向反了先跑: ./run_sim2sim.sh --gamepad_debug
#    再用 --axis_x N --axis_yaw M 覆盖轴号

# 3) 固定命令验证转弯
./run_sim2sim.sh --cmd_vel_x 0.0 --cmd_vel_yaw 1.0 --episode_length 300

# 4) 录制视频
./run_sim2sim.sh --save_video --video_path /tmp/turn.mp4 \
                 --cmd_vel_x 0.0 --cmd_vel_yaw 1.0 --episode_length 300
```

**参数说明**（全部可选）：
`--episode_length` 步数、`--cmd_vel_x/y/yaw` 固定命令、`--viewer` 实时窗口、
`--gamepad` 手柄（隐含 viewer）、`--save_video`、`--watch_dir` 热重载训练中最新 checkpoint。

---

## 5. 控制律（真机部署必须复现）

在 `sim2sim.py` 中实现，与**训练环境完全一致**：

```
腿（位置环）: q_target = clip(default_pose + 0.25 * action[0:12], 关节限位)
               tau = kp * (q_target - q) + kd * (0 - qd)        # kp=150, kd=4
轮（速度环）:  w_target = 12.5 * action[12:16]                   # WHEEL_VEL_SCALE
               tau = kd_w * (w_target - qd)                     # kd_w=2, kp_w=0
```

| 参数 | 腿 (hip/thigh/calf) | 轮 |
|---|---|---|
| kp | **150.0** | 0.0 |
| kd | **4.0** | **2.0** |
| 扭矩上限 | 150 N·m | 53 N·m |
| 速度上限 | 14.0 rad/s | 12.5 rad/s（→ 线速度 1.41 m/s） |
| 动作缩放 | action_scale = 0.25 | wheel_vel_scale = 12.5 |

**关节限位**：hip ±0.6 rad；thigh −0.7 ~ 1.75 rad；calf −1.0 ~ 0.35 rad。

**控制周期**：每 50 Hz 控制步 = 4 × 0.005 s 仿真步。轮速控制带宽和扭矩限制直接影响站姿维持能力——**真机必须先验证 PD 与力矩上限能托住 0.45 m 站姿**。

---

## 6. 观测空间（64 维，真机对齐关键）

`_build_observation()` 按以下顺序拼接（policy order）：

| # | 段 | 维 | 说明 |
|---|---|---|---|
| 1 | base_lin_vel | 3 | **恒为 0**（sim2sim 未接入速度估计，训练端同样置零） |
| 2 | base_ang_vel | 3 | 世界角速度 → body 系（用 brax quat 旋转） |
| 3 | projected_gravity | 3 | body 系重力投影（IMU 可算） |
| 4 | joint_pos_rel | 12 | 腿位置 − default_pose（仅 12 个腿关节） |
| 5 | joint_vel | 16 | 全部关节速度（含 4 轮） |
| 6 | last_action | 16 | 上一控制步的 action |
| 7 | command | 3 | [vx, vy, wz] 指令 |
| 8 | **gait_phase** | 8 | 步态相位时钟 sin/cos（每脚 2 维） |

**gait_phase 关键**：`phi = (step * 0.02 / 0.6 + offset) mod 1`，offset = [0, 0.5, 0.5, 0]（FL, FR, RL, RR）。每脚输出 `[sin(2πφ), cos(2πφ)]`，共 8 维。
**step 是 episode 内控制步计数，reset 时归 0**——真机每次启停必须从 0 重新计数，否则相位错位会导致步态混乱。

观测值 clip 到 [−100, 100]。**无 running normalization**（训练端也未做 obs 归一化，直接喂原始值）。

---

## 7. 策略行为特征（验收依据）

在原生 MuJoCo 下、纯 yaw ±1.0 rad/s 命令诊断（`diagnose_turn` 工具）：

| 指标 | iter_1000 |
|---|---|
| 左转对角抬腿（FL+RR 同空比例） | 0.67 |
| 右转对角抬腿（FR+RL 同空比例） | 0.65 |
| peak lift | 0.08 ~ 0.18 m |
| yaw rate 跟踪 | ±1.03 rad/s（指令 ±1.0，跟随良好） |
| 平面漂移 | ~0.10 m/s（原地转弯，非平移） |

> ⚠️ **已知风险（务必阅读）**：训练在 iter ~4000+ 出现 reward hacking（四轮高频乱摆腿刷抬腿奖励）。**本包选用的 iter_1000 是达标版本**，但这是**训练后期退化前的 checkpoint**。部署时不要对更高 iteration 的 checkpoint 抱期望；若需重新训练，参考 `src/cfg/` 的奖励配置并注意早期停止。

---

## 8. 真机部署注意事项（安全优先）

1. **安全保护**：任何情况下保证 `torque_limit` 生效（腿 150 / 轮 53 N·m）；建议软件限位 + 看门狗，异常立即断电。
2. **站姿验证**：先不开策略，用 PD 保持 default pose（站高 0.45 m），确认电机能托住重量、无低频抖动、无过热。再逐步引入策略。
3. **起步要慢**：从 `--cmd_vel_yaw 0.2` 小指令开始，确认对角步态稳定后逐步加大到 1.0。
4. **相位时钟**：真机每次上电/复位，`step` 计数器从 0 开始；轮速单位 rad/s。
5. **时延**：训练未含时延随机化（`randomize_action_latency=False`）。若通讯引入明显时延，需先补偿再上线。
6. **base_lin_vel 置零**：真机若可提供里程计速度，也应保持该段为 0（与训练/sim2sim 一致），除非重新训练。
7. **轮滑/摩擦**：sim 摩擦系数 0.8。真机地面摩擦低于此值时步态可能退化，可考虑摩擦垫。

---

## 9. 训练溯源

- 训练框架：`/home/sysu/Desktop/Project/Bruce/RL_Train/code`（MJX/JAX PPO）
- 本策略：`run_name=traj_v7`，`checkpoints_20260819_001958_traj_v7/iteration_1000.pkl`
- 站高 0.45 m（default pose thigh=0.20, calf=−0.35，FK 实测 0.449 m，抬腿能力 0.10 m）
- 奖励要点：`feet_air_time_turn=50` + `feet_lift_turn=3` + `rotation_gait_symmetry=15`（242a371 成功配方），`phase_gait=0`
- 达标判定：diagnose_turn 对角同空比例 > 0.6、yaw 跟踪、漂移小

**给部署 agent 的起步建议**：先跑 `./run_sim2sim.sh --gamepad` 手感确认策略行为，再对照 §5/§6 在 C++ 框架中实现控制律与观测，最后按 §8 安全流程上真机。
