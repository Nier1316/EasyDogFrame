# 四足机器人运控与强化学习框架 —— 知识库

> 调研整理于 2026-08，面向本工程（16 电机四足：12 关节 + 4 轮，CAN 阻抗控制，C++ 实机框架）的后续运控与 RL 对接选型。
> 每个框架标注：**定位 / 语言 / 与本工程的相关性**。链接为官方仓库或文档。
>
> ⚠️ **2026-09-29 时效说明**：第 1~5 章的**选型调研清单仍然有效**（框架生态没变），可继续作为备选参考。
> 但第 **§6「对本工程的对接建议」已被实际实现推翻**——工程实际走的是：**50Hz** 策略环（`rl::CONTROL_DT=0.02`）、
> **手写 MLP**（`include/strategy/mlp.h`，零依赖，不用 ONNX Runtime / LibTorch）、腿 **kp/kd = 250/4**（`rl::LEG_KP/KD`，非 200/20）。
> 详见 §6 开头的订正块。当前部署权重 `weights/iteration_9754.pkl`，`main.cpp` 激活 **Example37_RLTeleopControl**。

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

> ⚠️ 注：本工程**最终没有采用** ONNX Runtime / LibTorch，而是用自己的手写 MLP（`include/strategy/mlp.h`）+ 内嵌权重头文件 `policy_weights.h`。
> 上表仅作**选型参考**，不代表现状。详见 §5/§6 订正。

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
- 相关性：**高（仅作范式参考）**。部署范式（obs 归一化→推理→act 反归一化/clamp→下发）与本工程的控制循环**思路**对应；
  ⚠️ 频率不是 100Hz 而是 **50Hz**（`rl::CONTROL_DT=0.02`），推理也不是 ONNX Runtime 而是**手写 MLP**（`include/strategy/mlp.h`）。

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

> ## 🔴 **本节建议已被实际实现推翻（2026-09-29）**
>
> 下面这条清单是 2026-08 的**调研期设想**，工程最终**没有照它做**。保留仅供追溯，勿再当作现状或待办：
>
> | 本节旧建议 | 实际实现（以代码为准） |
> |---|---|
> | 100Hz 控制循环 | **50Hz** 策略环（`rl::CONTROL_DT=0.02f`，见 `include/strategy/rl_controller.h`）；电机收发独立线程 2ms/500Hz |
> | ONNX Runtime 推理 | **手写 C++ MLP**（`include/strategy/mlp.h`，64→512→256→128→16，ELU），零外部依赖，权重由 `tool/export_policy.py` 导出成 `include/strategy/policy_weights.h` |
> | `kp/kd = 200/20` | 腿 **`LEG_KP=250 / LEG_KD=4`**（对齐 traj_v28 训练）；起立另有 `JOINT_IMPEDANCE` hip 300/10、thigh/calf 250/10 |
> | obs 归一化 | **无 running normalization**，观测直接 clip 到 [−100,100] 后喂网络 |
> | 力矩 clamp | 真机 `TORQUE_CMD_LIMIT` = Hip 120 / Thigh 120 / Calf 200 / Wheel 52 N·m |
> | 轮子速度目标 | **固件 SPEED 速度环**（`SendSpeed`，`WHEEL_KVP=3.0/WHEEL_KVI=0.05`），不是上位机 500Hz 闭环 |
>
> 结论：**§1~§5 的选型清单可继续用于后续技术选型**（如换 Isaac Lab / MuJoCo MJX 重训），但 §6 的"怎么接"必须以现有代码为准。

1. **训练侧**：以 `legged_gym + rsl_rl` 起步（成熟、教程多），为本机写自定义 URDF + 环境类（12 关节位置/速度/扭矩 + 4 轮）；长期迁 `Isaac Lab` 或 `MuJoCo MJX`（开源免费、速度快）。⚠ 实际训练走的是自研 MJX/JAX PPO 管线（`RL_Train/code`），本仓库 `dogurdf_sim2sim_deploy/` 是其原生 MuJoCo 验证器。
2. **动作接口对齐**：仿真环境中的动作→力矩，与本工程 `SendImpedance(pos, vel, kp, kd, tau_ff)` 语义一致（位置增量 + 前馈力矩）这一点仍然成立；但 `kp/kd` 取 `rl::LEG_KP/KD = 250/4`（RL 闭环）或 `JOINT_IMPEDANCE`（起立，hip 300/10、thigh/calf 250/10）——**不是**旧文写的 `GetJointImpedance()` 默认 200/20。
3. **部署侧（已被推翻）**：旧建议"导出 ONNX + ONNX Runtime + 100Hz 线程"**未采用**。现状：`tool/export_policy.py` 把手写网络权重导出为 C 头文件，`mlp_forward()` 直接内联推理，挂进 50Hz RL 循环。clamp 仍与仿真保持一致（动作/限位/扭矩）。
4. **非 RL 备选**：若先做可解释的运控，用 `legged_control`（qiayuanl）+ `OCS2` 做 NMPC/WBC，或 `CHAMP` 快速搭步态。
5. **sim-real 一致**：本工程 `leg_kinematics.h` 与仿真 `leg_kinematics.m` 已对齐；连杆/限位/站立姿态由 `robot_calibration.h` 定义，**RL 的 `kp/kd` 由 `include/strategy/rl_controller.h` 定义**（250/4），仿真侧参数要与之一致；sim2sim 侧另见 `dogurdf_sim2sim_deploy/src/sim2sim.py`（`SIM_DT=0.005 / DECIMATION=4`，默认与真机同权重 `weights/iteration_9754.pkl`）。

---

## 7. 专题深潜：触地检测 · 相位检测 · sim2real gap 补偿

本文件（§1~§6）解决"**用哪个框架**"；专题文档解决"**这些框架具体怎么做触地/接触检测、相位检测，以及怎么补 sim2real gap**"：

> 📌 **`docs/CONTACT_PHASE_SIM2REAL.md`**（2026-09-29）
>
> - **§2 框架地图**：本文件之外的补充对象 —— `unitree_rl_mjlab`、`extreme-parkour`、`HIM`、`DreamWaQ`、`RMA`、`Barkour`、`Cheetah-Software`、`OCS2`、`Quad-SDK`、`unitree_guide`、HyQ/Camurri、`mcx-lab/legged_state_estimator`、`invariant-ekf`，以及轮足专用的 ANYmal-on-Wheels / Go2-W / CTBC / Wheel-Legged-Gym / Ascento / `Ros2Go2Estimator`。
> - **§3 触地/接触检测**：六类方法对照（相位调度 / JFD / GMO / 力传感器 / 逻辑回归 / 学习式）+ 各框架**代码级**做法与阈值（legged_gym 的 `>1 N`、`mcx-lab` 的 `β0=−20,β1=0.7` ⇒ `F_n≈28.6 N`、CTBC 的 3 帧滑窗、walk-these-ways 的 `contact_estimate>200`）。
> - **§4 相位检测**：开环时钟 / 概率化相位奖励 / 接触触发重置 / AFO-CPG / 学习式相位，五种做法的代码位置与超参；**轮足为什么倾向抛弃相位**（Lee 2024 显式删 CPG、WB-MPC 用"运动学腿效用"替代）。
> - **§5 sim2real gap**：按 5 类 gap（执行器动力学 / 时序延迟 / 接触摩擦地形 / 感知状态估计 / 域随机化工程）组织的"症状 → 手段 → 量化效果 → 代价"，含大量可直接抄的参数（DR 区间、延迟区间、`DCMotor` 公式、辨识方法）。
> - **§6 轮足专章** + **§7 本项目分阶段落地建议**（v0 前提 → v1 纯本体指示器 → v2 支撑腿筛选/接触锚定里程计 → v3 进策略）+ **§8 存疑清单**。
