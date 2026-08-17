# 四足机器人运控与强化学习框架 —— 知识库

> 调研整理于 2026-08，面向本工程（16 电机四足：12 关节 + 4 轮，CAN 阻抗控制，C++ 实机框架）的后续运控与 RL 对接选型。
> 每个框架标注：**定位 / 语言 / 与本工程的相关性**。链接为官方仓库或文档。

---

## 0. 选型速览（结合本工程）

| 需求 | 首选 | 备选 |
|---|---|---|
| RL 训练（四足步态，快速上手） | **legged_gym + rsl_rl**（Isaac Gym） | Isaac Lab、engineai_legged_gym |
| RL 训练（长期、多机器人/多任务） | **Isaac Lab**（NVIDIA） | MuJoCo MJX、Genesis |
| 免费/开源 GPU 并行仿真 | **MuJoCo MJX** | Genesis、Brax |
| 非 RL 的经典运控（MPC/WBC） | **legged_control**（qiayuanl，ROS2） | OCS2、CHAMP、Cheetah-Software |
| 刚体动力学/正逆动力学 | **Pinocchio** | Drake、MuJoCo、RBDL |
| 实机部署（ONNX/LibTorch） | **legged_rl_deploy**（ONNX Runtime + LibTorch） | engineai_legged_gym、rl_sar |

> 本工程已有自研 C++ 实机层（`MotorManager` + `leg_kinematics` + 阻抗控制），不直接套用现成实机 SDK；
> 上述框架主要用于**仿真训练侧**与**部署侧的代码范式**参考，需为本机写自定义 URDF 与环境。

---

## 1. RL 训练框架

### legged_gym（ETH legged robotics）
- 定位：基于 Isaac Gym 的四足机器人 RL 环境，业界事实标准。
- 语言：Python；内置 ANYmal、Unitree Go1/Go2/A1 等环境，PPO 训练。
- 相关性：**高**。环境结构（`BaseTask`/`LeggedRobot`）与奖励/域随机化写法是最直接的参考模板；换本机需自建 URDF + 环境类。

### rsl_rl（ETH legged robotics）
- 定位：GPU 加速的 PPO 训练库，`legged_gym` 默认训练器。
- 语言：Python + PyTorch。
- 相关性：**高**。与 `legged_gym` 配套；训练循环/日志/checkpoint 范式可直接复用。

### Isaac Lab（NVIDIA）
- 定位：Isaac Gym/Sim 的下一代，模块化机器人 RL 框架，官方主推方向。
- 语言：Python；支持多种 RL 后端（rsl_rl、skrl、sb3 等），支持 Newton 物理、sim2real 工具链。
- 相关性：**高（长期）**。功能最全但较重；官方四足/人形示例丰富。

### rsl_rl_sac / isaaclab-sac
- 定位：为腿足运动补的 Soft Actor-Critic 变体（PPO 之外的选择）。
- 相关性：中。连续控制、样本效率更高，适合后续探索。

### rapid-locomotion-rl（MIT Improbable-AI）
- 定位：MIT 的快速四足运动 RL（人形/四足），重 sim2real 与敏捷步态。
- 相关性：中。训练范式与领域随机化值得参考。

### engineai_legged_gym（逐际动力 EngineAI）
- 定位：国内 EngineAI 的四足 RL 训练 + 部署一体仓库。
- 相关性：**高**。含 C++ 部署代码，是国内最接近「训练→实机」闭环的开源样例。

---

## 2. MPC / 整机控制（非 RL 路线）

### legged_control（qiayuanl）
- 定位：ROS2 下的 NMPC + 整机控制（WBC）+ 状态估计，四足/人形通用，社区最活跃的 MPC 参考。
- 语言：C++ + ROS2。
- 相关性：**高**。若走「模型预测控制 + 整机控制」而非 RL，这是首选参考。

### OCS2（ETH legged robotics）
- 定位：切换系统最优控制（SLQ/iLQR MPC）工具箱，`legged_control` 的底层依赖。
- 语言：C++。
- 相关性：中-高。底层 MPC 求解器，配合 URDF→OCP 管线。

### CHAMP（chvmp）
- 定位：开源的 ROS 四足控制框架，模块化、支持多机型、带步态（trot/walk 等）。
- 语言：C++/Python + ROS；仿真用 Gazebo。
- 相关性：中。上手快，适合快速搭起步态；控制精度不如 MPC/RL。

### Cheetah-Software（MIT Biomimetics）
- 定位：MIT Mini Cheetah 的经典参考实现：凸 MPC + WBC + 状态估计。
- 语言：C++。
- 相关性：中-高。是四足 MPC 的「教科书」级实现，理解原理价值大。

### Drake（MIT / Toyota Research Institute）
- 定位：动力学建模 + 控制 + 非线性优化的工具箱，含微分 IK、WBC、MPC。
- 语言：C++/Python。
- 相关性：中。偏通用；可作整机控制/动力学分析底座。

---

## 3. 仿真器 / 物理引擎

### Isaac Gym（NVIDIA，已由 Isaac Lab 取代）
- 定位：早期 GPU 并行 RL 仿真的标杆，`legged_gym` 的运行时。
- 相关性：中。新项目建议直接用 Isaac Lab；存量四足教程仍大量基于它。

### MuJoCo + MJX（Google DeepMind）
- 定位：MuJoCo 是精准刚体动力学引擎；MJX 是其 JAX 加速的 GPU 版，速度与 Isaac 同级。
- 语言：C/C++ + Python/JAX。
- 相关性：**高**。开源、轻量、免费；MJX 已成 RL 训练主流选择之一（人形/四足）。

### Genesis（Genesis-Embodied-AI）
- 定位：新一代极速生成式物理引擎，训练速度号称达数十倍。
- 语言：Python。
- 相关性：中-高。新且热，适合大规模 RL 与 foundation model 评测，生态仍在快速演进。

### Brax（Google）
- 定位：纯 JAX 的并行刚体物理引擎。
- 语言：Python/JAX。
- 相关性：中。轻量、可微分，适合研究型训练。

### PyBullet / Gazebo
- 定位：经典仿真；PyBullet 通用、Gazebo 与 ROS 深度集成。
- 相关性：中。PyBullet 适合原型验证；Gazebo 适合 ROS 集成与可视化（CHAMP 用它）。

---

## 4. 动力学库

### Pinocchio
- 定位：刚体动力学库（正/逆动力学、雅可比、质心动力学），速度快、绑定完善。
- 语言：C++（Python 绑定）。
- 相关性：**高**。WBC/MPC 的动力学计算底座，与本工程 `leg_kinematics` 互补（它是通用库，本工程是手写三关节闭式解）。

### RBDL
- 定位：经典刚体动力学库。
- 相关性：中。Pinocchio 的早期替代。

### MuJoCo（作为动力学库）
- 定位：除仿真外，其接触/动力学模型也常被直接用作动力学引擎。
- 相关性：中。

---

## 5. sim2real 部署

### legged_rl_deploy（Renkunzhao）
- 定位：将 `legged_gym` 训练的 PPO 策略部署到实机（Unitree 系），ONNX Runtime + LibTorch 双后端。
- 语言：C++。
- 相关性：**高**。部署范式（obs 归一化→推理→act 反归一化/clamp→下发）与本工程 100Hz 控制循环直接对应。

### engineai_legged_gym（部署部分）
- 定位：训练 + 部署一体，含 C++ 推理与实机闭环样例。
- 相关性：**高**。

### rl_sar（fan-ziqi）
- 定位：四足 + 机械臂（SAR）RL，含 Go2 实机部署，ONNX/JIT。
- 相关性：中-高。若后续加机械臂可参考。

### Isaac Lab sim-to-real 工具链
- 定位：NVIDIA 官方 sim2real（Newton 物理 + 域随机化）文档与工具。
- 相关性：中-高。了解 sim2real 最佳实践（噪声建模、延迟、执行器建模）。

---

## 6. 对本工程的对接建议

1. **训练侧**：以 `legged_gym + rsl_rl` 起步（成熟、教程多），为本机写自定义 URDF + 环境类（12 关节位置/速度/扭矩 + 4 轮）；长期迁 `Isaac Lab` 或 `MuJoCo MJX`（开源免费、速度快）。
2. **动作接口对齐**：仿真环境中的动作→力矩，应与本工程 `SendImpedance(pos, vel, kp, kd, tau_ff)` 语义一致（位置增量 + 前馈力矩），`kp/kd` 用 `GetJointImpedance()`（默认 200/20）。
3. **部署侧**：训练导出 ONNX，参考 `legged_rl_deploy` 在 C++ 侧用 ONNX Runtime 推理，挂进 100Hz 线程；obs/act 的归一化与 clamp 与仿真保持一致。
4. **非 RL 备选**：若先做可解释的运控，用 `legged_control`（qiayuanl）+ `OCS2` 做 NMPC/WBC，或 `CHAMP` 快速搭步态。
5. **sim-real 一致**：本工程 `leg_kinematics.h` 与仿真 `leg_kinematics.m` 已对齐；连杆/限位/`kp/kd` 统一由 `robot_calibration.h` 定义，仿真侧参数要与之一致。
