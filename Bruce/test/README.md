# BigDog RL 仿真项目

## 项目结构

```
.
├── config/
│   ├── config.yaml              # 配置文件 (路径、关节参数、缩放因子等)
│   ├── policy.pt                # 训练好的 TorchScript 策略模型
│   └── robot/bigdog/xml/
│       ├── scene.xml            # MuJoCo 场景定义
│       └── bigdog.xml           # BigDog 机器人模型
├── demo1/
│   └── demo.py                  # Demo1: 完整仿真 — 手柄控制 + RL 策略 + PD 执行
└── demo2/
    └── demo2.py                 # Demo2: 模型推理演示 — 加载模型 + 构建输入 + 推理
```

---

## 依赖安装

两个 Demo 共用以下依赖，建议使用 Conda 创建独立环境：

```bash
conda create -n bigdog python=3.10
conda activate bigdog
pip install torch numpy pyyaml mujoco pygame
```

| 包 | 用途 |
|---|---|
| `torch` | 策略网络加载与推理 |
| `numpy` | 数值计算、传感器数据处理 |
| `pyyaml` | 读取 YAML 配置文件 |
| `mujoco` | 物理仿真引擎 (3.0+) |
| `pygame` | 手柄/游戏杆输入 (仅 Demo1) |

> **注意**: MuJoCo 3.0 需要从 [github.com/google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) 下载预编译库，或通过 `pip install mujoco` 安装 Python 绑定。

---

## Demo1: 完整仿真 (`demo1/demo.py`)

### 功能

完整的 BigDog 四足机器人强化学习控制仿真闭环：

```
手柄/零指令 → 观测构建 (归一化) → 策略网络推理 → 动作反归一化 → PD 控制器 → MuJoCo 物理仿真 → 传感器反馈 → (循环)
```

- 支持 Xbox 手柄实时控制机器人的前进/后退/平移/转向/身高
- 无手柄时以零指令运行，机器人原地站立
- 自动检测 CUDA，优先使用 GPU 推理

### 运行

```bash
cd demo1
python demo.py
```

---

## Demo2: 模型推理演示 (`demo2/demo2.py`)

### 功能

演示如何用 PyTorch 加载策略模型、构建符合真实数据分布的输入、执行推理：

1. `torch.jit.load` 加载 TorchScript 模型
2. 按观测向量各段的真实数值范围生成模拟输入
3. `model(input)` 前向传播得到 16 维动作输出

### 运行

```bash
cd demo2
python demo2.py
```

---

## 模型输入/输出详解

### 模型格式

- **格式**: TorchScript (`torch.jit.load`)
- **网络结构**: MLP (多层全连接网络)
- **用途**: 将观测向量映射为 16 个关节的目标位置偏移量

### 输入: 348 维 (6 帧 × 58 维/帧)

策略网络的输入是 **6 帧历史观测的拼接**。单帧观测的 58 维构成如下：

| 索引 | 维度 | 内容 | 数据类型 | 原始量纲 |
|------|------|------|----------|----------|
| 0–2 | 3 | IMU 陀螺仪角速度 × `scale_ang_vel` | 角速度 | rad/s |
| 3–5 | 3 | 重力在机体坐标系的投影 | 方向向量 | 无 (单位向量) |
| 6–9 | 4 | 控制指令 × `commands_scale` | 速度/角度/高度 | m/s, rad/s, m |
| 10–25 | 16 | 关节位置误差 × `scale_dof_pos` | 角度 | rad |
| 26–41 | 16 | 关节速度 × `scale_dof_vel` | 角速度 | rad/s |
| 42–57 | 16 | 上一帧输出的动作 | 角度偏移 | rad (已缩放) |
| **合计** | **58** | | | |

6 帧按时间顺序排列 (最新帧在前)，展平为 348 维：

```
obs_seq = [obs(t), obs(t-1), obs(t-2), obs(t-3), obs(t-4), obs(t-5)]  → 348 维向量
```

### 输出: 16 维

网络输出 16 个值，对应 16 个关节的**目标位置偏移量**（相对默认站立位置）。4 条腿 × 4 个关节/腿：

```
FL: [hip, thigh, calf, wheel]
FR: [hip, thigh, calf, wheel]
RL: [hip, thigh, calf, wheel]
RR: [hip, thigh, calf, wheel]
```

---

## 归一化流程 (原始传感器数据 → 网络输入)

训练时出于数值稳定性考虑，将不同量纲的物理量通过缩放因子映射到相近范围。推理时必须使用**完全相同的缩放因子**。

### 缩放因子配置 (`config.yaml`)

```yaml
scale_factors:
  scale_lin_vel: 2.0    # 线速度缩放
  scale_ang_vel: 0.25   # 角速度缩放
  scale_dof_pos: 1.0    # 关节位置缩放
  scale_dof_vel: 0.05   # 关节速度缩放
  scale_height: 2.0     # 身高偏移缩放
```

### 各分量的归一化公式

| 分量 | 归一化公式 | 缩放后典型范围 |
|------|-----------|---------------|
| `obs[0:3]` | `imu_gyro * 0.25` | [-2, 2] |
| `obs[3:6]` | `world2self(imu_quat, [0,0,-1])` | [-1, 1] |
| `obs[6:10]` | `[vx, vy, yaw_rate, height] * [2.0, 2.0, 0.25, 2.0]` | [-2, 2] |
| `obs[10:26]` | `(dof_pos - default_dof_pos) * 1.0` | [-0.5, 0.5] |
| `obs[26:42]` | `dof_vel * 0.05` | [-2, 2] |
| `obs[42:58]` | `actions` (网络上帧输出，已处于缩放空间) | [-0.25, 0.25] |

### 偏航角指令的特殊处理

偏航角不直接使用 heading_target，而是计算**航向误差**后乘以 `yaw_kp` 转为角速度指令：

```python
yaw_now = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))   # 四元数 → 偏航角
yaw_err = atan2(sin(target - yaw_now), cos(target - yaw_now))  # 处理 ±π 环绕
commands[2] = yaw_kp * yaw_err     # 2.5 * 航向误差 → 角速度指令
```

---

## 反归一化流程 (网络输出 → 执行层)

### 第 1 步: 动作缩放 (反归一化)

```python
actions_scaled = actions * actions_scale   # actions_scale = 0.25
```

网络输出的 16 维值先乘以 `actions_scale`，从归一化空间恢复到物理角度偏移量 (rad)。

### 第 2 步: 目标速度计算 (仅轮子)

```python
vel_ref = zeros(16)
vel_ref[wheel_ids] = actions[wheel_ids] * vel_scale   # vel_scale = 10.0
```

轮子关节使用**速度控制**而非位置控制——网络的输出直接缩放到目标角速度 (rad/s)。

### 第 3 步: PD 控制器 (输出力矩)

```python
# 位置误差 = 目标位置 (default + 网络偏移) - 当前位置
#            = (default_dof_pos + actions_scaled) - dof_pos
#            = actions_scaled + (default_dof_pos - dof_pos)
#            = actions_scaled + dof_err

torque = Kp * (actions_scaled + dof_err) + Kd * (vel_ref - dof_vel)
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `Kp` | 70 (位置关节) / 0 (轮子) | 比例增益 |
| `Kd` | 2.0 (位置关节) / 0.5 (轮子) | 微分增益 |
| `torque` | 最终输出，clip 到 [-100, 100] | 关节力矩 (N·m) |

### 第 4 步: 力矩执行

```python
d.ctrl[:] = clip(torque, -100, 100)   # 写入 MuJoCo 控制信号
mujoco.mj_step(m, d)                  # 物理仿真前进一步
```

### 完整数据流总结

```
┌─────────────────────────────────────────────────────────────┐
│                      归一化 (观测构建)                        │
│                                                             │
│  IMU 陀螺仪 ──×0.25──▶ obs[ 0: 3]                           │
│  重力方向   ──旋转──▶ obs[ 3: 6]                            │
│  控制指令   ──×[2,2,0.25,2]──▶ obs[ 6:10]                   │
│  关节位置差 ──×1.0──▶ obs[10:26]                             │
│  关节速度   ──×0.05──▶ obs[26:42]                            │
│  上帧动作   ──直接──▶ obs[42:58]                             │
│                                                             │
│  6帧拼接展平 → 348维                                         │
└──────────────────┬──────────────────────────────────────────┘
                   ▼
         ┌─────────────────┐
         │   策略网络 (MLP)  │
         │  348维 → 16维    │
         └────────┬────────┘
                  ▼
┌─────────────────────────────────────────────────────────────┐
│                    反归一化 & 执行                            │
│                                                             │
│  policy_output[16]                                          │
│      │                                                      │
│      ├── × actions_scale (0.25) ──▶ 目标位置偏移量 (rad)     │
│      │       + dof_err ──▶ 位置误差                          │
│      │       × Kp (70) ──▶ 位置力矩分量                      │
│      │                                                      │
│      └── wheel 关节: × vel_scale (10.0) ──▶ 目标速度 (rad/s) │
│              - dof_vel ──▶ 速度误差                          │
│              × Kd (2.0/0.5) ──▶ 阻尼力矩分量                  │
│                                                             │
│      torque = Kp*(target_pos - dof_pos) + Kd*(target_vel - dof_vel) │
│      clip(torque, -100, 100) → MuJoCo d.ctrl                │
└─────────────────────────────────────────────────────────────┘
```

### 轮子关节的特殊性

4 个轮子关节（FL/FR/RL/RR_wheel_joint，索引 3, 7, 11, 15）：

- **位置增益 Kp = 0**：轮子可无限旋转，不控制位置
- **仅用速度控制**：`torque = Kd * (actions[wheel] * 10.0 - dof_vel)`
- **观测中位置置零**：`dof_pos[wheel_ids] = 0`

### 初始化阶段

仿真启动时，先用纯 PD 控制器将机器人稳定到 `stand_dof_pos` 站立姿态 200 步，确保 RL 策略接手时处于稳定状态：

```python
torque = 1.5 * Kp * (stand_dof_pos - dof_pos) - Kd * dof_vel   # 1.5 倍增益，站立更"硬"
```

> **关键约束**: 所有归一化/反归一化参数必须与训练时**严格一致**，否则网络输入分布将偏离训练分布，导致控制效果下降甚至完全失效。
