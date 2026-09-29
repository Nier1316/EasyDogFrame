# 触地检测 · 相位检测 · sim2real gap 补偿 —— 开源框架调研与本项目落地建议

> **调研日期**：2026-09-29（与代码库 HEAD `98ef8ea` + 本次审查修复对齐）
> **调研方式**：直接读一手来源 —— 克隆/抓取 GitHub 仓库源码逐行核对、arXiv 论文原文/HTML、官方文档、厂商 URDF/MJCF、本驱动器手册。
> **证据标注**：
> - **【源码】** = 已读到该仓库的具体代码/常量；**【论文】** = 论文或官方文档原文；**【手册】** = 本项目 `Doc/集成驱动器使用说明-V1.pdf`；
> - **【本项目】** = 本仓库代码；**【推断】** = 工程经验或基于前述事实的外推，**未直接验证**。
>
> ⚠️ 第 8 节列出本次**未能核实**的事项，请勿把那里的内容当事实传播。

---

## 0. 一页速览（TL;DR）

**关于"主流怎么做"**

1. **经典 MPC/WBC 栈几乎不做"测量式"触地检测**：MIT Cheetah 的 `ContactEstimator` 就是把步态调度器的计划相位直接当接触估计（源码注释自承是 pass-through），CMU Quad-SDK 的接触表由"计划索引取模"生成（`phase = current_plan_index % period_`）。**接触序列是规划出来的，不是测出来的。**
2. **RL 训练栈里"接触"绝大多数只出现在奖励里，不出现在部署观测里**：`feet_air_time` 在 legged_gym / IsaacGymEnvs / Isaac Lab / unitree_rl_gym / unitree_rl_mjlab 里逐字相同（判据 `contact_forces[...,2] > 1.` N，目标滞空 0.5 s）。真机上唯一被验证可用的"接触观测"是 walk-these-ways 部署端的 `contact_estimate > 200`（一个**关节力矩/模型估计量，标度不是牛顿**）。
3. **真做"接触触发相位校正"的产线级开源实现只有一家**：ANYbotics OCS2 的 `GaitAdaptation`，且**只做"提前触地"一种**（窗口 0.1 s），其余三种象限全部返回 `None` —— 这是刻意的保守设计。它改的是**离散模式序列**，不是连续相位。
4. **无足端力传感器时的三条主流技术路线**：(a) 关节力矩残差 / **广义动量观测器（GMO）**；(b) 概率模型（Hwangbo HMM、Camurri **逻辑回归**）；(c) 学习式**连续接触概率**。
5. **一个反直觉但重要的结论**：真机上**硬阈值 GRF 的下游里程计精度反而不如"连续接触概率"**（ATE 4.25 m vs 4.09 m，[arXiv 2606.05501](https://arxiv.org/html/2606.05501v1) 表 1/2）。所以触地检测应输出**连续置信度**，而不是二值位。
6. **打滑 ≠ 失去接触**：轮子在打滑时仍然着地。任何只做"是否着地"的检测都覆盖不了打滑，必须并行两路信号。
7. **相位方面**：主流 RL 栈全部是**开环时钟**（`φ = (episode_len·dt) % T / T`），接触只用于奖励塑形。**轮足尤甚**：ANYmal-on-Wheels 的"驱动"被定义成**"所有腿全程保持接触、不排任何 lift-off"的退化步态**；Lee et al.（*Science Robotics* 2024）在把 Miki 的足式控制器移植到轮足时**显式删除了 CPG**，理由原文是"remove any engineered bias in the motion"。
8. **sim2real gap 应按类别分别治理**，7 类手段对应 5 类 gap 有明确的"谁治谁"映射（见 §5.1）：执行器建模/辨识治动力学误差；延迟测量+随机化治时序误差；摩擦/地形随机化治接触误差；特权学习/历史隐变量治感知误差；结构化 DR+课程治泛化。**"20 秒空中数据 + CMA-ES"的解析辨识（顺带辨识出全局延迟 7.5 ms）是投入产出比最高的一项**（[arXiv 2509.06342](https://arxiv.org/abs/2509.06342)）。

**关于本项目（先看结论）**

9. 本项目**目前没有任何触地/接触检测代码**（全仓 grep 只命中"腿悬空不触地"之类的注释）。相位是**开环** `gait_phase`（8 维分组 `[sin×4, cos×4]`）。sim2real 走的是**参数级辨识**路线（摩擦前馈、重力前馈、延迟辨识），**没有状态级自适应**。
10. **好消息**：16 个电机（含 4 个轮）**每个都有力矩反馈**（12 bit，经 `ApplyMotorCalibrationTorque`），500 Hz；腿正逆运动学已有；轮子可接地 ⇒ 有天然的连续接触源。**做接触估计的信号条件是具备的。**
11. **拦路石**：`include/motion/robot_calibration.h` 的 `LINK_DYNAMICS` 全为 0、`BODY_MASS = 0`（TODO 未实测），且 `leg_kinematics.h` **没有雅可比**。**没有质量模型 + 没有 J，就算不出绝对 GRF**。所以第一步不是写检测器，而是**补质量/惯量辨识 + 加解析雅可比**（§7.0）。
12. **建议路线**：v0 力矩通道校验 + 轮空载摩擦基线 → v1 纯本体指示器（打滑 `δ>0.1 m/s`、轮驱动扭矩残差）→ v2 支撑腿筛选 + 接触锚定里程计 → v3 才考虑把接触概率作为策略观测（**需重训**）。**不要做接触触发的相位重置**（轮足无清晰触地上升沿，噪声下可能永不重置）。

---

## 1. 本项目现状：有什么、缺什么

### 1.1 现有能力盘点

| 能力 | 现状 | 位置 |
|---|---|---|
| 电机反馈 | 16 路 **位置/速度/力矩**，**2 ms（500 Hz）**，`SCHED_FIFO` 优先级 80 | `src/runtime/motor_io.cpp`、`src/motor/ele_motor.cpp` 周期回帧分支 |
| 力矩反馈 | 12 bit 解码 `t = uint_to_float(i_int, −t_max, t_max, 12)`，**含 4 个轮子**，经 `ApplyMotorCalibrationTorque` 对齐符号 | `src/motor/ele_motor.cpp`、`include/motor/motor_calibration.h` |
| 力矩量程 | Hip/Thigh ±120、Calf ±200、Wheel ±52 N·m（`TORQUE_CMD_LIMIT`） | `include/motor/ele_motor_def.h` |
| IMU | 维特 HWT606，**100 Hz**，6 轴（gyro + quaternion），安装 `Z_DOWN_X` | `src/strategy/imu_device.cpp` |
| 策略 | 50 Hz（`CONTROL_DT=0.02`），64 维观测，**无任何接触项** | `src/strategy/rl_controller.cpp` |
| 相位 | 开环 `gait_phase` 8 维，**分组** `obs[56..59]=sin×4, obs[60..63]=cos×4`，`GAIT_CYCLE=0.6 s`，`GAIT_OFFSET={0,0.5,0.5,0}` | 同上 |
| 轮控 | 固件 **SPEED 速度环** `SendSpeed(vel, kvp=3.0, ki=0.05)`；轮速反馈一阶低通 α=0.2@500Hz（τ≈10 ms） | `src/strategy/rl_controller.h`、`FilterWheelVel` |
| 轮速保护 | `WHEEL_ESTOP_VEL=15 rad/s` 超速自动急停（瞬态）+ 手动急停（保持）；2 s 静默窗 | `src/motor/motor_manager.cpp` |
| 腿运动学 | `leg_fk` / `leg_ik` / `hip_rotation_matrix` / `leg_fk_all`；**2026-09-29 新增解析雅可比** `leg_jacobian`/`leg_jacobian_body`/`leg_cond_proxy`/`leg_foot_force_body`/`leg_foot_force_to_torque`（已与 FK 有限差分逐元素校验，误差 <5e-4）；连杆 `LEG_L1/L2/L3=0.1308/0.34/0.343` | `include/motion/leg_kinematics.h`、`robot_calibration.h` |
| 多圈解算 | 轮编码器 **unwrap** 技术（跨 ±12.5 rad 回绕补全 25 rad） | `include/motion/wheel_position_loop.h`（已废弃但技术可复用） |
| 摩擦前馈 | 腿库仑摩擦 `LEG_FF_FC[12]` = 1.64~6.17 N·m（Ex54 吊装辨识），`fv` 全 0，`fc·tanh(τ_pd/2)` 在 **500 Hz** 发送线程重算 | `src/strategy/rl_controller.cpp`、`src/app/examples/examples_common.cpp` |
| 重力前馈 | `JOINT_IMPEDANCE[..].tau_ff` = hip −10 / thigh +5 / calf +12(前) +20(后) N·m（**thigh 与注释冲突，待现场确认**） | `include/motor/motor_calibration.h` |
| 500 Hz 钩子先例 | `MotorManager::SetLegTauFFOverride(fn)` —— 在 `SendOnce` 里用最新反馈重算并叠加前馈 | `include/motor/motor_manager.h` |
| 遥测落盘 | `S2RRecorder` 50 Hz：`qt/qtv/q/qd/trq(16) + wvel + quat/gyro + 欧拉角` | `src/common/s2r_recorder.cpp` |
| 实测延迟 | 真机纯传输延迟 **≈24 ms（18–30 ms）**，建议 `action_delay_steps=1` | `docs/ACTION_DELAY_MEASURE.md` |
| 固件速度环 | 手册明确 `τ_des = Kvp(ω_des−ω_act) + Kvi·Dt·Σ(ω_des−ω_act)`，**Dt = 50 µs**（固件内 20 kHz） | **【手册】**`Doc/集成驱动器使用说明-V1.pdf` |

### 1.2 与主流的四条关键差距

1. **没有接触/打滑检测**（0 行代码）。
2. **相位纯开环、且不被任何反馈校正**（连 ANYmal 那种"提前触地"都没有）。
3. **没有状态估计器**：`base_lin_vel` 在策略观测里**恒置 0**（训练端如此设计），真机上也没有任何线速度/高度估计。所有 domain randomization、特权学习、隐变量自适应都建立在"有个估计器"之上，本项目这块是空的。
4. **没有域随机化/执行器建模**（在真机侧）：仿真侧的 DR 情况见 §7.5。本项目走的是"参数辨识 + 前馈"的**经典栈路线**，这条路线与随机化路线并不冲突，但需要补齐辨识覆盖的 gap。

### 1.3 三条硬约束（决定方案取舍）

| 约束 | 影响 |
|---|---|
| **无足端/轮端力传感器** | 排除"力传感器阈值法"；必须走**关节力矩残差 / GMO / 学习式接触概率** |
| **无质量与惯量模型**（`LINK_DYNAMICS` 全 0、`BODY_MASS=0`） | **无法计算绝对 GRF**；`f = J⁻ᵀτ` 的绝对值不可信，只能做**相对/归一化**判据。必须先补质量辨识（工具已就绪：`Example59`，数值待实测） |
| **无视觉、无高程图** | 排除 perceptive locomotion 路线；只能"隐式本体地形"（学习式）或不做地形 |

补充约束：IMU 只有 100 Hz 且无加速度输出（比姿态融合器慢 20 倍，做冲击检测时要注意 50 ms 量级的采样间隔）。
雅可比原本缺失，**已于 2026-09-29 补齐**（见 §7.0）。

---

## 2. 开源框架地图（按类别整理）

### 2.1 RL 训练 / 部署栈

| 框架 | 定位 | 接触怎么做 | 相位怎么做 | 与本项目相关性 |
|---|---|---|---|---|
| [legged_gym](https://github.com/leggedrobotics/legged_gym) + [rsl_rl](https://github.com/leggedrobotics/rsl_rl) | ETH 事实标准训练栈 | 仅奖励：`feet_air_time`（`>1 N`）、`collision`、`stumble`、`feet_contact_forces`（100 N）、终止（`>1 N`） | **完全没有相位**（基座版无 `gait_phase`，网上常被误归属） | ⭐⭐⭐ 奖励/DR 写法模板；**注意它没有相位** |
| [Isaac Lab](https://github.com/isaac-sim/IsaacLab) | NVIDIA 官方，含 sim2real 工具链 | `ContactSensor`（`force_threshold=1.0 N` 二值）、`feet_air_time`（0.5 s）、`illegal_contact` | DR-Legs 贡献任务有 `[sin,cos]` 时钟（period 1.0 s）；**velocity 任务默认没有** | ⭐⭐⭐ 执行器模型（`DCMotor`/`DelayedPDActuator`/`ActuatorNet`）是最好的一手参考 |
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | Unitree 官方（legged_gym fork） | 逐字照抄 legged_gym（1 N / 0.5 s） | H1 任务有 `sin/cos`（period 0.8 s，offset 0.5） | ⭐⭐⭐ **本项目观测结构与它同构**（见 §7.1） |
| [unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) | Unitree 新版（MuJoCo/mjlab） | **奖励 + 仅 critic 观测**（`foot_contact`、`sign(F)·log1p(|F|)` 对数压缩） | **actor 观测 + 奖励参考**：`phase`（period 0.6 s，站立归零）+ `feet_gait`（`leg_phase<0.56` 与接触一致率，w=0.5） | ⭐⭐⭐⭐ **最现代、最值得抄的一份** |
| [walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) | Improbable AI，行为多样性 | 期望接触概率进奖励（von Mises κ=0.07 软化）；`observe_contact_states` 默认关 | 相位 = **命令 + 观测 + 奖励** 三重身份；clock 是 **4 维纯 sin**（无 cos） | ⭐⭐⭐ 相位/奖励的教科书；延迟处理激进（固定 6 步=120 ms） |
| [extreme-parkour](https://github.com/chengxuxin/extreme-parkour) | 跑酷 RL | **接触进 actor 观测**（`contact_filt−0.5`，±0.5 二值）+ 100 帧接触 buffer 给 critic | 无相位（用 10 帧 proprio 历史） | ⭐⭐ 接触进观测的唯一开源范例 |
| [HIM / HIMLoco](https://github.com/InternRobotics/HIMLoco) | 隐式内模自适应 | 无（不显式建模） | 无（6 帧历史 + 16 维对比学习隐变量） | ⭐⭐⭐⭐ **传感器配置与本项目完全一致**（纯本体），最该复刻的架构 |
| [DreamWaQ](https://github.com/Manaro-Alpha/DreamWaQ) | β-VAE 本体自适应 | 无 | 无（CENet：速度头 + 重建头） | ⭐⭐⭐ 无外感知下≈height-map oracle |
| [RMA / rl_locomotion](https://github.com/antonilo/rl_locomotion) | 快速运动自适应 | 无（用 `slope_dots` 近似） | 无 | ⭐⭐ 三段式范式；**原版状态含足端接触开关，本项目没有 → 直接选 HIM/DreamWaQ 路线** |
| [Barkour](https://github.com/google-deepmind/barkour_robot) | Google DeepMind | — | — | ⭐ 只有硬件/固件，**无训练代码**；但其真机辨识参数在 MuJoCo Menagerie 里 |
| [ASE](https://github.com/nv-tlabs/ASE) / AMP | 对抗式运动先验 | 无 | 无 | ⭐ 不是 sim2real 方法 |

### 2.2 经典 MPC / WBC 真机栈

| 项目 | 接触检测 | 相位 | sim2real 手段 | 相关性 |
|---|---|---|---|---|
| [Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)（MIT） | **无**：`ContactEstimator` 是 pass-through，把步态相位当接触；**无足端力**（TI 板回传的 `force[3]` 是阻抗控制算出的**期望力**） | `GaitScheduler` / `OffsetDurationGait` 开环时钟；**无接触重置** | 状态估计噪声参数化 + `high_suspect_number=100` 的 Q/R 门控 + 协方差截断；控制 1 kHz | ⭐⭐⭐⭐ **`high_suspect_number` 门控最干净的参考**；噪声参数可直接当起点 |
| [OCS2](https://github.com/leggedrobotics/ocs2)（ANYbotics） | `Gait`=参数化模式表；`GaitAdaptation` **需要外部传入** `measuredContactFlags`（生产者未开源） | 开环时钟 + **唯一的产线级"提前触地"重置**（窗口 0.1 s，改离散模式序列） | 靠模型精度 + 在线约束（摩擦锥/零力/零速约束），**不做随机化** | ⭐⭐⭐⭐ 相位校正的唯一范本 |
| [qiayuanl/legged_control](https://github.com/qiayuanl/legged_control)（A1/Go1） | **足端力传感器阈值**：`footForce[i] > contact_threshold(=40，原始 ADC 计数)` | OCS2 `ModeSchedule` 开环 | KF 用接触标志把非支撑腿 Q/R ×100；**只在触地时更新足底高度** | ⭐⭐⭐⭐ 接触门控 + 足高自适应的干净实现 |
| [unitree_guide](https://github.com/unitreerobotics/unitree_guide) | 无（`WaveGenerator` 开环定时波，接触只用于平衡控制器约束矩阵） | `_normalT = fmod(passT − period·bias, period)/period`，trot `bias={0,.5,.5,0}` | — | ⭐⭐ |
| [Quad-SDK](https://github.com/robomechanics/quad-sdk)（CMU） | **无**：`phase = current_plan_index % period_`，查名义接触表 | 离散索引时钟；trot `period=0.36 s, duty=[0.5]*4, offsets=[0,.5,.5,0]` | **强依赖动捕**（无 mocap 直接 return false）；落脚点用 Raibert + 可达圆 | ⭐⭐ 参数可直接抄；其状态估计路线（动捕）本项目不可用 |
| HyQ / Camurri（IIT+牛津） | **接触检测最扎实的一手资料**：固定阈值 50 N → Schmitt 双阈值迟滞 → **逻辑回归接触概率** | — | 接触概率当权重做基座速度加权平均；Mahalanobis 距离门控丢异常腿 | ⭐⭐⭐⭐⭐ **动态步态上：固定阈值漂移 1.79 → 迟滞 0.75 → 逻辑回归 0.43 cm/m** |
| [mcx-lab/legged_state_estimator](https://github.com/mcx-lab/legged_state_estimator) | **力矩→GRF→逻辑回归**：`β0=−20, β1=0.7` ⇒ **F_n≈28.6 N 是 50% 赤点**；力矩先过 **10 Hz** 低通 | — | 接触力协方差 `α(ΔF_n)²` → InEKF 的 R 矩阵 | ⭐⭐⭐⭐⭐ **无传感器 GRF 的标准做法** |
| [invariant-ekf](https://github.com/RossHartley/invariant-ekf) | 接触辅助 InEKF（接触点进状态） | — | 位置/偏航不可观；漂移 <5% 行驶距离；可 >2000 Hz | ⭐⭐⭐ 若要加状态估计 |

### 2.3 轮足专用

| 项目 | 轮端传感 | 接触/相位 | sim2real |
|---|---|---|---|
| ANYmal on Wheels（[Keep Rollin', RA-L 2019](https://arxiv.org/abs/1809.03557)；[WB-MPC, ICRA 2021](https://arxiv.org/abs/2010.06322)） | 无轮端 F/T；**16 关节全力矩控制**（SEA） | **"驱动" = 所有腿全程接触、不排 lift-off 的退化步态**；地形法向由**最近轮接触位置最小二乘平面拟合** | 摩擦锥约束保证 no-slip；4 m/s、CoT 降 83% |
| [Lee et al. 2024, *Science Robotics*](https://arxiv.org/abs/2405.01792)（ANYmal-on-wheels RL） | 无 | **显式删除 CPG**："discarded the use of the CPG ... to remove any engineered bias"；**无相位、无 clock** | 本体感觉历史（RNN）+ 特权学习（接触状态/力只进 teacher）；**明确放弃状态估计器**（"conventional state estimators often result in high errors in case of wheel slippage"） |
| [Go2-W 长距离导航, arXiv 2606.21387](https://arxiv.org/html/2606.21387v1) | 无（轮毂电机 + 充气胎） | 无相位；用**两个 EMA**（0.029 s / 0.144 s）捕捉周期性抬脚 | 轮位置观测**恒置 0**（速度控制）；摩擦 U(0.05,1.5)；**因充气胎难判接触而放弃轮式里程计**；**热集中**（84 °C 关机 → 奖励重塑后 60 °C） |
| [CTBC / Tron1, arXiv 2509.02986](https://ar5iv.labs.arxiv.org/html/2509.02986) | 真机纯本体 | **轮-障碍接触力 xy 分量 + 3 帧滑窗**触发抬腿（"Isaac Gym 里轮地接触会 flicker"）；相位 = **0.6 s 正弦前馈**，训练后期**退火到 0** | 摩擦 [0.2,1.6]、延迟 [0,20] ms；`wheel_spin` 惩罚阈值 **0.1 m/s**（系数 −5.0） |
| [Contact-Anchored Proprioceptive Odometry, arXiv 2602.17393](https://ar5iv.labs.arxiv.org/html/2602.17393) | 无 | `f^W_z ≤ f_th` 判接触（**阈值论文未给绝对值**）+ 触地事件；**轮足专用"有效滚动角补偿"** `Δψ_eff = Δψ − Δβ`，`β = θ_pitch + q_thigh + q_calf` | 轮足实测：**700 m 环 7.68 m 误差（≈1.1%）** |
| [Wheel-Legged-Gym](https://github.com/clearlab-sustech/Wheel-Legged-Gym) | 仿真用 `net_contact_force` | **无相位 clock** | 轮关节 `stiffness=0, damping=0.5`（=P 型速度环）+ `vel_action_scale=10`；摩擦 [0.1,2.0]；**动作延迟 [0,10] ms** |
| Ascento（[ICRA 2019](https://ar5iv.labs.arxiv.org/html/2005.11435)） | 无（轮电机电流环） | **无步态相位**；跳跃有 5 段显式相位；**落地检测 = 髋关节力矩阈值** | LQR + 在 10 个腿高上线性化插值；仿真假设 no-slip |

### 2.4 状态估计 / 接触估计专用库

| 项目 | 用途 |
|---|---|
| [mcx-lab/legged_state_estimator](https://github.com/mcx-lab/legged_state_estimator) | InEKF + 力矩残差 GRF + 逻辑回归接触概率（A1 示例，**可直接对照参数**） |
| [RossHartley/invariant-ekf](https://github.com/RossHartley/invariant-ekf) | 接触辅助右不变 EKF（C++，可 >2 kHz） |
| [ShineMinxing/Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator) | 接触锚定本体感觉里程计（**含轮足验证**） |
| [UMich-CURLY/slip_detection_DOB](https://github.com/UMich-CURLY/slip_detection_DOB) | 滑移速度作为状态 + 卡方检验做打滑二分类 |

---

## 3. 触地 / 接触检测：主流怎么做

### 3.1 六类方法总表

| 方法 | 判据 | 精度 | 成本 | 适用 |
|---|---|---|---|---|
| **A. 相位调度（不做检测）** | `phase ∈ [0, switchingPhase] ⇒ contact` | 完全开环；地形突变/打滑时不知道 | 0 | 平坦地形 + MPC 需要确定接触序列（Cheetah、Quad-SDK、ANYmal） |
| **B. 关节力矩偏差（JFD）** | `M q̈ + C q̇ + g − Jᵀτ + κ_fric`；简化版 `τ_meas − τ_model > τ_th` | 依赖模型；**需要 q̈（二次微分放大噪声）** | 低 | 有力矩反馈但无足端力传感器 —— **本项目场景**（建议用 C 而非 B） |
| **C. 广义动量观测器（GMO）** | `ṙ = K_o[κ_c − r]` ⇒ `r(s) = K_o/(s+K_o)·κ_c(s)` | 一阶低通；`K_o` 大→快但噪声大 | 低 | **同 B 但无需加速度，最实用** |
| **D. 力传感器阈值** | `F_n > F_th` | 最快最准 | 高（传感器/走线/标定） | 高动态步态（Unitree Go1 `footForce > 40`） |
| **E. 逻辑回归接触概率** | `P = 1/(1+exp(−β₁F_n−β₀))` | **trot 上漂移 1.79→0.43 cm/m**（比固定阈值好 4.2×） | 需带真值数据拟合 β | 能反解 GRF 且步态多样 |
| **F. 学习式连续接触概率** | 本体历史 → 接触概率 | 最高（Contact-CNN 单腿 97.9–98.6%；硬阈值仅 70–73%） | 高（需标签/自监督） | 研究级 |

### 3.2 关键代码级细节（可复用）

**（1）GRF 反解（无传感器）** —— [mcx-lab/legged_state_estimator](https://github.com/mcx-lab/legged_state_estimator) 的 `ContactEstimator`：
```cpp
// tauJ 是 10 Hz 低通后的关节力矩；getJointInverseDynamics() 是刚体逆动力学项
f_contact[i] = -( J_contact(i).block<3,3>(0, 3*i).transpose().inverse()
                  * (tauJ.segment<3>(3*i) - invdyn.segment<3>(3*i)) );
F_n[i]       = f_contact[i].dot(contact_surface_normal_[i]);
p[i]         = 1.0 / (1.0 + exp(-beta1[i]*F_n[i] - beta0[i]));   // beta0=-20, beta1=0.7
```
▶ **推论：`F_n ≈ 20/0.7 ≈ 28.6 N` 是接触概率 50% 的赤点。** 滤波配置：`lpf_tauJ_cutoff=10 Hz`、`lpf_dqJ_cutoff=10 Hz`、`lpf_ddqJ_cutoff=5 Hz`。**【源码】**

**（2）去抖 / 迟滞的工程配方**
```cpp
// legged_gym：官方注释自承 "contact reporting of PhysX is unreliable on meshes"
contact      = contact_forces[:, feet, 2] > 1.0;         // 只取 z 分量，抑制水平刮擦
contact_filt = contact | last_contacts;                  // 一步迟滞（OR 滤波，容忍漏检）
last_contacts = contact;
```
```cpp
// CTBC：仿真/真机都会 flicker → 3 帧滑窗，全超阈值才算稳定接触
stable = force_xy[k-2] > th && force_xy[k-1] > th && force_xy[k] > th;
```
```cpp
// Pronto / HyQ 的 Schmitt 双阈值（结构确认，具体数值论文未给）
if (!contact && f > F_high)                      { contact = true;  t_touch = now; }
if ( contact && f < F_low && now-t_touch > T_min){ contact = false; }
// F_high/F_low 常用比 1.5~3×，T_min 20~50 ms
```

**（3）触地弹跳（bouncing）是最大假正例来源**
Lin et al.（CoRL 2021）原文记录："we observe a **bouncing effect** on the robot's foot upon touch down"，处理办法是对足高信号**低通后取局部极值**作为标签，从而去掉这些假正例。真机上同理：**不要用单帧力矩/力做阈值比较**。

**（4）真机上唯一验证过的"接触观测"**
[walk-these-ways 部署端](https://github.com/Improbable-AI/walk-these-ways/blob/master/go1_gym_deploy/utils/cheetah_state_estimator.py#L281)：
```python
self.contact_state = 1.0 * (np.array(msg.contact_estimate) > 200)
```
—— `contact_estimate` 是底层发布的**关节力矩/模型估计量**，标度不是牛顿。⚠️ **不要把仿真里的 1 N 直接搬到真机当阈值。**

**（5）接触力进观测时的压缩技巧**（[unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab)）
```python
foot_contact_forces = sign(F) * log1p(|F|)      # 接触力跨 0~2000 N，对数压缩后才适合进网络
```

### 3.3 无足端力传感器时的推荐链路

```
关节力矩 τ（低通 10 Hz）
   ↓ 减去重力项 g(q) 与摩擦项 τ_f(q̇)          ← 不扣这两项，残差会被吞掉
τ_ext = τ − g(q) − τ_f(q̇)
   ↓ 雅可比转置映射（或 GMO 一阶观测器，K_o = 30~100 rad/s）
f^B = J(q)^{-T} · τ_ext   →   f^W = R_WB · f^B
   ↓ 投影到接触法向 + 3 帧滑窗 + 双阈值迟滞
p_contact ∈ [0,1]（连续置信度，不要只给 0/1）
```

### 3.4 接触信息拿去做什么（用途矩阵）

| 用途 | 做法 | 代表 |
|---|---|---|
| **状态估计门控** | `trust = contact ? 1 : 0`；`Q_leg *= (1+(1−trust)·100)`，`R_leg` 同理；**只在触地时更新足底高度** | Cheetah、qiayuanl/legged_control |
| **接触锚定里程计** | 触地记 `c_i^W`；支撑期 `p̃ = c_i^W − R·p_ee^B`，`ṽ = −R(ω×p_ee^B + ṗ_ee^B)`；多腿平均 | arXiv 2602.17393（轮足 700 m/1.1%） |
| **量测协方差自适应** | `R_t = (1/(p_stance+ε))·I₃` —— **连续概率优于硬阈值** | arXiv 2606.05501 |
| **冲击抑制** | 触地上升沿 → 该腿 `kd` ×2~3、`kp` 降 20%，持续 50 ms | 【推断】 |
| **安全** | 打滑 + 无支撑 → 限轮速；台阶顶住（`τ_w` 突增 + `ω_w→0`）→ 禁止继续加轮速（**防热集中，Go2-W 的真实故障模式**） | Go2-W |
| **相位校正** | 仅"提前触地"一种（见 §4.3） | ANYmal |

### 3.5 打滑检测（必须与"失去接触"分开）

```
滑移率         s = (ω·r − v_body)/(ω·r)           |s|>0.15~0.3 判打滑
滑移速度       v_slip = ω·r − v_body
本体感觉版     δ_i = |ω_w,i·r| − ‖v_foot,i,xy‖    δ>0.1 m/s（= CTBC 的 wheel_spin 阈值）
摩擦-滑移曲线  μ(v_slip) = w₁·tanh(w₂·v_slip)
```
- **无真值线速度时的主流做法**：把滑移当"速度量测偏置"，写进 RIEKF 状态，自回归建模 `u̇ = −αu + Rw_u`，观测 `y_ENC = Rᵀv + Rᵀu`，**卡方检验**做二分类（[arXiv 2209.15140](https://ar5iv.labs.arxiv.org/html/2209.15140)，开源 [slip_detection_DOB](https://github.com/UMich-CURLY/slip_detection_DOB)）。
- **轮足的坑**：Go2-W 因充气胎难以判接触而**彻底放弃轮速+腿运动学里程计**，改用"速度指令历史 + IMU → 监督学习相对位移"（100 Hz，T=30）。
- **必须在 `n_support ≥ 3` 且 `|roll/pitch| < 15°` 时才启用**，否则坡道/松软地面/单腿腾空会误报。

---

## 4. 相位 / 步态检测：主流怎么做

### 4.1 四类相位来源对照

| 类型 | 机制 | 是否受接触反馈 | 频率 | 缺陷 |
|---|---|---|---|---|
| **开环时钟** | `φ ← (φ + f·Δt) mod 1` | ❌ | 固定或命令给定 | 速度变化时相位对不上（大步幅→滑移）；相位与真实支撑相漂移 |
| **周期性奖励（概率化相位）** | 相位仍是开环，但用 von Mises / 正态 CDF **软化相位边界** | ❌ | 固定 | 同上，但鲁棒性更好 |
| **接触触发相位重置** | 触地事件把 φ 吸到 AEP / 提前切模式序列 | ✅ 离散 | 自带 | 噪声下可能**永不重置** |
| **自适应频率振荡器 / CPG** | `φ̇ = ω − K·e(t)·sin φ`，`ω̇ = −η·e(t)·cos φ` | ⚠️ 连续耦合 | **自学** | 收敛慢（秒级）；公式本轮未拿到一手 PDF 逐字核验 |
| **学出来的相位** | 历史帧 → 隐状态（RNN/Transformer/隐变量） | ✅ 隐式 | 涌现 | 延迟敏感、可解释性差、需重训 |

### 4.2 各框架的代码级做法

| 框架 | 相位来源（实际代码） | 关键超参 |
|---|---|---|
| legged_gym（基座） | **根本没有相位**（观测 = lin/ang vel + gravity + cmd + dof_pos + dof_vel + actions） | — |
| [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | `φ = (episode_len·dt) % period / period`，输出 `sin`+`cos` | `period=0.8 s`，`offset=0.5` |
| [unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) | 同上 + **站立时（‖cmd‖<0.1）相位归零** | `period=0.6 s`；`feet_gait` 阈值 `leg_phase<0.56`，w=0.5 |
| [Isaac Lab DR-Legs](https://github.com/isaac-sim/IsaacLab) | `(episode_time % period)/period` → `[sin, cos]` | `period=1.0 s` |
| [walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) | `gait_indices += dt·freq`，每足 `sin(2πφ_i)`（**只有 sin，没有 cos**） | 频率是**命令**（2~4 Hz）；`κ=0.07`（von Mises），`σ_f=50`、`σ_v=0.5` |
| [champ / libchamp](https://github.com/chvmp/libchamp) | `elapsed = now − last_touchdown_`（**`last_touchdown_` 是内部整周期计数，不是触地事件**）+ 归一化斜坡信号 | swing 周期**硬编码 0.25 s** |
| [quad-sdk](https://github.com/robomechanics/quad-sdk) | `phase = current_plan_index % period_` | `period=0.36 s`，`duty=[0.5]*4`，`offsets=[0,.5,.5,0]` |
| [Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software) | `dphase = phaseScale·(dt/T)`，`phase = fmod(phase+dphase,1)` | TROT `T=0.5 s, switching=0.5`；`OffsetDurationGait` 用 MPC 迭代计数当钟 |
| [OCS2](https://github.com/leggedrobotics/ocs2) | `advancePhase(phase, dt, gait)` 递归跨步态积分 | `Gait{duration, eventPhases, modeSequence}`；trot `0.35/0.35` |

**结论**：**主流 RL 框架里"触地 → 相位校正"这条链是断的** —— 接触只进奖励/终止，不进相位。完整实现这条链的只有 CPG 控制系统和状态估计器，而它们都不在 RL 框架内。

### 4.3 接触→相位校正：唯一的生产级实现与它的失效模式

**ANYmal `GaitAdaptation`（[源码](https://github.com/leggedrobotics/ocs2)）** —— 四象限策略，**只实现一种**：
```
计划支撑 + 实测支撑 → None
计划支撑 + 实测摆动 → None      （不处理"晚离地"）
计划摆动 + 实测接触 → 若 hasLiftedSinceLastContact_ 且 距计划触地 < 0.1 s ⇒ EarlyContact
计划摆动 + 实测摆动 → None
```
动作不是改连续 φ，而是 `setContactStateOfLegBetweenModes(true, curMode, nextContactMode, leg, gait)` —— **把中间模式序列里该腿强制置为支撑**，让 MPC 提前把它当支撑腿。唯一超参：`earlyTouchDownTimeWindow = 0.1 s`。

**失效模式（必须知道）**：Sun et al.（[Front. Robot. AI 2021](https://www.frontiersin.org/articles/10.3389/fncir.2021.706064/full)）的带噪实验原文：加噪后 GRF **随机穿越阈值**，"the regular phase resetting process was **destroyed**. In the worst case, **the CPG phase would never be reset**" —— 离散重置（PR）快但"rapid but intermittent, with random success"，连续调制（PM）慢但稳定。

**对轮足的推论【推断】**：轮子滚动接触**几乎没有清晰的 GRF 上升沿**，所以 Aoi/Sun 式的相位重置在轮足上比在足式上更脆弱。

### 4.4 轮足的答案：相位可以不要

- **ANYmal-on-Wheels（MPC 路线）**：`Gait` 里把"驱动"定义成**每条腿都排在支撑相、不设 lift-off 的步态**（"driving is defined by a gait pattern where each leg is scheduled to stay in contact, and no lift-off events are set"）；接触约束从足式的**全向零速**退化为**非完整滚动约束**（平面情形只剩向心加速度 `[0,0,r₀(χ̇+θ̇)²]ᵀ`）。
- **WB-MPC（ICRA 2021）** 用**连续的运动学腿效用**替代离散相位：
  `u_i(t) = 1 − √((π_∥(r̃_Ei)/λ_∥)² + (π_⊥(r̃_Ei)/λ_⊥)²) ∈ [0,1]`；`u_i → 0` 即"该腿需要回收"→ 自动发现**非周期步态**，CoT 降最多 85%。
- **Lee et al. 2024（RL 路线）**：**显式删除 CPG**，改用本体感觉历史 + 高度图 + 特权学习，8.3 km 自主导航。
- **Go2-W（RL 路线）**：无相位，用两个 EMA（0.029 s / 0.144 s）捕捉周期性抬脚；轮位置观测恒 0。
- **CTBC**：用 **0.6 s 正弦前馈**注入 hip/knee pitch（膝幅=髋 2 倍），训练后期**把系数退火到 0** 实现零样本迁移。

> **对本项目的直接含义**：我们的策略**有**显式 8 维相位（等价 unitree_rl_gym / IsaacLab 的写法，**不是** walk-these-ways 的 4 维纯 sin），说明训练侧把它当作"轮足混合步态（轮推进 + 腿小幅抬落）的节律锚点"。**这是合理的、主流的**，但三条升级路径按性价比排序是：① 把周期从常量改成**命令**；② 用**运动学腿效用**补充"时机"信息；③ 不要做接触重置。

---

## 5. sim2real gap 补偿：按 gap 分类

### 5.1 总表

| Gap | 真机症状 | 主流手段 | 量化效果 | 硬件 | 重训 | 工程量 |
|---|---|---|---|---|---|---|
| **1 执行器/动力学** | 同向偏置；抬不起腿/整体下沉；仿真好真机发散；**发热降额** | ①解析 dc_motor 模型 ②系统辨识+CMA-ES ③执行器网 ④摩擦前馈/随机化 ⑤功率/热约束 | ActuatorNet 力矩 RMS **0.74 vs 理想模型 3.55 N·m**；Bjelonic 辨识后 CoT **−32%**；Go2-W **84 °C→60 °C** | 力矩传感器（ActuatorNet）**或不需要**（UAN/辨识） | 是 | 中–大 |
| **2 时序/延迟** | 高频振荡/极限环；相位滞后；脚蹭地；过冲 | ①延迟测量（脉冲/相位斜率）②延迟随机化 ③上一帧动作进观测 ④观测历史插值 | 实测：PD 环 3 ms、策略环 15–19 ms、纯状态推理 4 ms、带视觉 40 ms；辨识出的全局延迟 **7.5 ms** | 无 | 是 | **小–中（性价比最高）** |
| **3 接触/摩擦/地形** | 打滑、下陷、弹跳、GRF 估计错、硬地抖/软地陷 | ①摩擦+恢复随机化 ②地形课程 ③接触参数（solref/solimp/condim/frictionloss）标定 ④感知闭环（本项目无视觉）⑤无传感器接触估计 | Lee 2020 零样本跨泥/雪/水；Miki **1.2 m/s vs 纯本体 0.6 m/s** | 无（除视觉） | 是 | 小–大 |
| **4 感知/状态估计** | 速度跟踪差；基座高度漂；接触误判；步态不对称；滑地上估计发散 | ①特权→学生蒸馏 ②历史隐变量自适应（RMA/HIM/DreamWaQ）③本体感受状态估计（接触辅助 KF/InEKF）④**删掉不可观测量** | RMA **73.5% vs 纯 DR 62.4% vs 教师 76.2%**；HIM 楼梯 **100% vs RMA 60%**；DreamWaQ ≈ height-map oracle | 无（RMA 原版用足端接触开关） | 是 | 中–大 |
| **5 域随机化/工程** | 泛化差或"过度保守"（慢、僵、能耗高） | ①结构化 DR（少而准）②课程 ③真机在线微调 ④安全/退化策略 | DR 的代价有论文证据：**均值回报更低、方差更小**（"随机化不是免费午餐"）；ERFI 2 个参数即 **+53%** 泛化 | 无 | 是/否 | 小–大 |

### 5.2 Gap 1：执行器 / 动力学建模

**解析 DC 电机四象限饱和**（[Isaac Lab `DCMotor`](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/actuators/actuator_pd.py)）：
```
τ_max(q̇) = clip(τ_stall·(1 − q̇/q̇_max), −∞, τ_con)
τ_min(q̇) = clip(τ_stall·(−1 − q̇/q̇_max), −τ_con, ∞)
τ_applied = clip(τ_computed, τ_min(q̇), τ_max(q̇))
```
外加总功率上限 `|τ|ᵀ|q̇| ≤ P_max`（UAN 论文明确说这是为避免触发厂商**功率保护**而必须加的项）。

**可直接抄的参数**：
- Go2：`effort_limit=23.5, saturation_effort=23.5, velocity_limit=30.0, stiffness=25, damping=0.5`
- ANYdrive：`effort_limit=80, velocity_limit=7.5, stiffness=40, damping=5`；ANYmal C 用 LSTM 执行器网
- Barkour（真机辨识 → MuJoCo）：**`Kp=50, Kd=0.5, forcerange=±18 N·m, damping=0.024, frictionloss=0.13, armature=0.011`**
- MuJoCo Menagerie Go2：`damping=2, armature=0.01, frictionloss=0.2`
- ⚠️ **Unitree 官方 URDF 的坑**：A1 有 `damping="0.01" friction="0.2"`，而 **Go1 / Go2-W 的 URDF 是 0**（Go2-W 的 4 个轮关节连 damping/friction 都没写）。**直接拿来仿真等于把摩擦设成 0。**

**"20 秒空中数据"的解析辨识**（[arXiv 2509.06342](https://arxiv.org/abs/2509.06342)，投入产出比最高）：
```
单关节模型：I_a·q̈ + d·q̇ = sat(P_τ·(q̂ − q + q̃_b) − D_τ·q̇ + τ_comp) + τ_f
传递函数：  H_q(s) = e^{−s·T_d} · P_τ / (I_a·s² + (d+D_τ)·s + P_τ)
参数向量：  p = [I_a, d, τ_f, q̃_b, T_d]ᵀ ∈ R^{4n+1}（≈49 个）
损失：      ℓ_e = (1/k)Σ‖q_real − q_sim(p)‖²
```
- 激励：**0.1–2.0 Hz 位置 chirp**（上限取 f_policy/2）；单关节另做 1 Hz–1250 Hz 电流 chirp
- 优化：**CMA-ES**（比贝叶斯优化稳），4096 并行环境回放，10–24 h 收敛
- 结果：**ANYmal 与 Tytan 都辨识出全局指令延迟 T_d = 7.5 ms**；阻尼 ~5 N·m·s/rad；电流环带宽 ≈346 Hz、死区 ≈400 µs
- **不做任何动力学随机化**（只随机化推力/地面摩擦/地形），CoT 降低 32%
- 本项目所需：**吊起 + 位置 chirp + ≥1 kHz 日志**（现有 Ex47 chirp 链路可复用，需提高采样率与时长）

**学习型执行器模型对比**：
| 名称 | 输入 | 结构 | 效果 | 备注 |
|---|---|---|---|---|
| ActuatorNet | (位置误差, 速度) 历史 | MLP 3×32 softsign | RMS **0.740 N·m** | 需关节力矩传感器 |
| ActuatorNetLSTM | **只用当前** | LSTM hidden=8 | 输入简化性能不降 | **仿真步长不得 <0.005 s**（否则不稳） |
| UAN | 过去 20 步位置/速度误差 | MLP [128,128] ELU | 输出**修正力矩 δτ** | **不需要力矩传感器**；每 5 ms 执行 |

**摩擦前馈的轮足一手公式**（ANYmal-on-wheels, *Science Robotics* 2024）：
```
τ_t = K_τ·GR·I_t + τ_friction
τ_friction,C = −C₁·φ̇      （粘滞）
τ_friction,S = −C₂·sgn(φ̇) （库仑）
```
⚠️ **LuGre/Stribeck 在腿足上未找到报告辨识数值的实例** —— 因为高减速比关节通常已由**厂商固件做了摩擦补偿**（Bjelonic 专门把 `τ_comp` 单列），再叠加 LuGre 容易重复建模。

### 5.3 Gap 2：时序与延迟

**真机延迟实测对照**：

| 环节 | 实测 | 来源 |
|---|---|---|
| MCU PD 伺服 | **3 ms** | Tan 2018 |
| 板载计算机策略环 | **15–19 ms** | Tan 2018 |
| 关节/IMU 传感 | 2.5 ± 1 ms | MMDR |
| 网络推理（仅状态 / 带视觉） | **4 ms / 40 ± 9 ms** | MMDR |
| 深度图传感 | 33 ± 4 ms | MMDR |
| 辨识出的全局指令延迟 | **7.5 ms** | Bjelonic 2026 |
| **本项目实测** | **≈24 ms（18–30 ms）** | `docs/ACTION_DELAY_MEASURE.md` |

**延迟随机化区间对照**：

| 工作 | 区间 | 实现 |
|---|---|---|
| Tan 2018 | latency 0–40 ms，control step 3–20 ms | 观测历史按 `n·Δt − t_latency` **线性插值** |
| DreamWaQ | system delay **0–15 ms** | DR 表 |
| **Go2-W（轮足）** | action latency **[3,8] 步 @50 Hz = 60–160 ms** | 随机化 + 作为 1 维**特权观测**给 critic |
| walk-these-ways | **固定 6 步 = 120 ms**（`randomize_lag_timesteps` 只控制开关！） | `lag_buffer`（长 7）滑动 |
| Isaac Lab | `DelayedPDActuator(min_delay, max_delay)`，单位**物理步**，**reset 时重采样** | `DelayBuffer.set_time_lag()` |
| HIM | 0–3 子步 = **0–6 ms**（模拟 PD 环内相位滞后） | decimation 内一阶保持 |
| RMA | **固定 20 ms**（+ 随机仿真子步数） | `act_history[size−3]` |

**实践建议**：50 Hz 策略下取 **1–3 步（20–60 ms）** 覆盖多数平台；板载栈复杂时 3–8 步。另需注意：**滤波器群延迟必须计入 T_d** —— 一个 10 Hz 低通就带来 ~16–30 ms 延迟，与 20 ms 策略周期同量级。

**零成本的隐式延迟建模**：几乎所有工作都把 `a_{t−1}` 放进观测；Go2-W 平地策略还用**两个 EMA（0.029 s / 0.144 s）**替代单步历史。

### 5.4 Gap 3：接触 / 摩擦 / 地形

**域随机化区间对照（可直接抄的量级）**：

| 工作 | 摩擦 | 其他 |
|---|---|---|
| legged_gym | **[0.5, 1.25]**（64 buckets） | push 每 15 s、1.0 m/s；质量默认关（ANYmal C `[−5,5]` kg） |
| Lee 2020 | **[0.4, 1.0]** | 只训刚性地形却零样本过泥/雪/水 |
| RMA | 训练 **[0.05, 4.5]**，测试 [0.04, 6.0] | 分形地形 |
| DreamWaQ | **[0.2, 1.25]** | 10 级地形课程，倾角 [0°, 22°] |
| **Go2-W（轮足）** | **[0.05, 1.5]** | 质量 [−1, 5] kg |
| CTBC（轮足） | **[0.2, 1.6]** | restitution [0,1]、CoM ±3 cm、Kp/Kd [0.8,1.2]、力矩 [0.8,1.2]、**延迟 [0, 20] ms** |
| Wheel-Legged-Gym | **[0.1, 2.0]**（20× 跨度） | 惯量 [0.8,1.2]、Kp/Kd [0.9,1.1]、**延迟 [0, 10] ms** |
| unitree_rl_mjlab | 足底 **(0.3, 1.6)** | 编码器零偏 ±0.015 rad、CoM ±0.05 m |
| Isaac Lab velocity | **(0.8, 0.8)/(0.6, 0.6) 即不随机化** | 只随机化基座质量 ±5 kg 与 CoM ±0.05 m（**与 legged_gym 默认不同，迁移易踩坑**） |

**接触模型旋钮**：
- Isaac Gym：`contact_offset=0.01, rest_offset=0.0, bounce_threshold_velocity=0.5, max_depenetration_velocity=1.0, num_position_iterations=4, solver_type=tgs`
- Isaac Lab / Newton：`NewtonShapeCfg(margin=0.0, ke=160000.0, kd=1100.0)`（接触刚度/阻尼可随机化）
- MuJoCo（Menagerie Go2）：足端 `condim=6, friction="0.8 0.02 0.01", solimp="0.015 1 0.022"`；关节 `damping=2, armature=0.01, frictionloss=0.2`
- ⚠️ **`armature`（转子折算惯量 = 转子惯量×减速比²）与 `frictionloss`（库仑摩擦）是最容易被忽略、但对高减速比关节影响最大的两个量。**

**无视觉时怎么补接触**：HIM 把高程图/摩擦/恢复系数当"扰动"用内模思想估计；DreamWaQ 用 β-VAE 从本体历史"想象"地形（论文称无外感知下**接近有 height map 的 oracle**）。

### 5.5 Gap 4：感知与状态估计

| 方案 | 结构/超参 | 效果 | 出处 |
|---|---|---|---|
| **特权 → 学生蒸馏** | teacher 用 noiseless 地形/摩擦/外力；student 用历史（TCN-20=0.4 s / TCN-100=2 s / GRU） | Miki 1.2 m/s vs 纯本体 0.6 m/s | Lee 2020、Miki 2022 |
| **RMA** | base π MLP(128×3)，输入 `x(30)+a_{t−1}(12)+z(8)`；adaptation φ = MLP+3×1D CNN，输入 **50 步历史**，**10 Hz 跑 φ / 100 Hz 跑 π** | 仿真 **73.5% vs 纯 DR 62.4%**；实机 15 cm 下台阶 80%、油面 90%、负重 12 kg | [arXiv 2107.04034](https://arxiv.org/abs/2107.04034) |
| **HIM** | 历史 **H=5**；显式速度回归 + **z∈R¹⁶ 对比学习（Sinkhorn-Knopp, 32 prototypes, T=3）**；actor 输入 `45+3+16=64` | 短楼梯 **100% vs RMA 60%**；跟踪误差 0.073 vs 0.158 | [arXiv 2312.11460](https://arxiv.org/abs/2312.11460) |
| **DreamWaQ** | CENet：共享编码器 → 速度头(MSE) + β-VAE 头(重建 o_{t+1})；`L = L_est + L_VAE` | ≈ height-map oracle | [arXiv 2301.10602](https://arxiv.org/abs/2301.10602) |
| **接触辅助 InEKF** | 状态放矩阵李群；接触点进状态 | 漂移 **<5% 行驶距离**；可 >2000 Hz | [arXiv 1904.09251](https://arxiv.org/abs/1904.09251) |
| **删掉不可观测量** | yaw、基座线速度、接触力**不进 actor**，交给 critic | "紧凑观测空间帮助迁移" | Tan 2018 |

⚠️ **关键提醒**：**RMA 原版的状态里含 4 维二值足接触指示，来自足端力传感器**（论文原文 "binarized foot contact indicators from the foot sensors"）。**本项目没有足端力传感器 → 应优先选 HIM / DreamWaQ 路线**（它们完全不需要接触量，只用编码器+IMU 历史），这与本项目的传感器配置**完全一致**。

### 5.6 Gap 5：域随机化与工程手段

- **DR 不是免费午餐**：Tan 2018 原文 —— 带随机化训练的控制器**均值回报更低（次优）、方差更小（鲁棒）**，"控制器学会在随机环境中保守行事"。
- **"少而准"路线**：Rudin 2021 只随机化**地面摩擦 + 观测噪声（按真机实测标定）+ 随机推力**；Bjelonic 2026 **完全不做动力学随机化**；ERFI 只用 **2 个参数**（RFI 每步力矩扰动 + RAO 每回合偏置）就比全域 DR 泛化 **+53% / +61%**。
  - ⚠️ ERFI 的关键工程细节：**只对旋转关节注入，不要对基座注入**（ANYmal C 上基座力 >5 N 或力矩 >3 N·m 就会训出 pronking 等低效步态）。
- **课程**：Rudin 游戏式课程（楼梯 5→20 cm、坡 0→25°，平地 <4 min、崎岖 <20 min 训完）；Lee 2020 粒子滤波自适应课程（可通行性判据 = 能否以 >0.2 m/s 沿指令前进）。
- **真机在线微调**：Smith 2022 用 REDQ + 自动 reset + 板载速度估计，草坪 **<2 h**、室内 **<2.5 h** 从频繁摔到稳定；Ha 2020 用 cMDP + 拉格朗日松弛做安全约束，平地 1.5 h。**代价高，需要自动 reset 与安全约束。**
- **安全/退化（必须做）**：力矩裁剪（四象限）+ 功率上限；动作二阶差分惩罚 `R_action_curvature = Σ|ä|/(1+ȧ²)^{3/2}`；观测低通（ANYmal-on-wheels 用 5 Hz）；摔倒检测（roll>0.4 rad / pitch>0.2 rad）；**负载均衡** `R_leg_effort_std`（EMA w=0.975 平地 / 0.7 崎岖）—— 这一项对轮足尤其重要（Go2-W 的髋关节热集中）。

---

## 6. 轮足机器人专章

### 6.1 与足式的本质差异

| 维度 | 足式（点足） | 轮足 |
|---|---|---|
| 接触状态 | 二值、离散切换，触地瞬间清晰 | **连续**：滚动时始终接触，"触地事件"失去意义 |
| 约束 | **全向零速** `ṗ_contact = 0` | **非完整滚动约束**：仅垂直/侧向为零，沿滚动方向自由 |
| 相位 | 明确的 swing/stance 交替 | **大部分时间无 swing**；"步态"可以是涌现的 |
| 触地检测 | GRF 上升沿 | **几乎没有上升沿**（滚动接触平滑） |
| ZUPT | 触地期可用 | **不能直接用**（接触点在滚动，姿态一变编码器就读出假旋转）→ 需"有效滚动角补偿" `Δψ_eff = Δψ − Δβ` |

### 6.2 轮足 sim2real 的三个特殊性

1. **轮子的"速度环"是仿真里通常不存在的闭环**（本项目：固件 `Dt = 50 µs` 的 20 kHz 内环，【手册】）。
   - **主流做法：仿真里必须把轮子建成"速度伺服"而不是"力矩源"** —— Wheel-Legged-Gym 用 `wheel: stiffness=0, damping=0.5` + `vel_action_scale=10`；CTBC/Go2-W 都把轮动作定义成速度目标、腿动作定义成位置目标。
   - 共识：**轮速必进观测；轮的动作是速度目标**（只有 ETH 的 WBC/MPC 把轮当力矩源）。
   - 本项目现状：sim2sim 用 `kd·(w_target − qd)`（`WHEEL_KD=2.0`），真机用固件速度环 —— **这是当前最大的建模差异**。好消息：内环 20 kHz 远快于 500 Hz 指令率，所以差异主要在**外环指令路径 + 饱和 + 死区**，而非带宽。
2. **打滑是轮足的头号问题**（比"是否着地"更重要）：`δ = |ω·r| − ‖v_foot,xy‖`，阈值 **0.1 m/s**（CTBC 的 wheel_spin 实测阈值）。
3. **热与功率**：轮式行走把载荷集中到髋关节 → Go2-W 厂商控制器 **34 min 达 84 °C 触发过热关机**；学习策略靠奖励重塑（负载均衡 + 动作曲率惩罚 + 抬脚分摊）稳定到 **60 °C**。**本项目已有轮速急停但没有任何热/负载均衡逻辑。**

### 6.3 主流轮足工作怎么处理接触/相位（汇总）

| 工作 | 接触 | 相位 | 备注 |
|---|---|---|---|
| ANYmal-W（MPC） | 规划侧接触调度 + **轮接触位置最小二乘拟合地形法向** | **驱动 = 全接触退化步态** | 16 关节力矩控制 |
| WB-MPC 2021 | MPC 约束（移动点接触） | **运动学腿效用 `u_i(t)`** 替代相位 | 自动发现非周期步态，CoT −85% |
| Lee 2024（RL） | 只有特权观测有接触状态/力 | **删除 CPG** | 8.3 km 导航 |
| Go2-W | 无（明确因充气胎放弃） | 无（双 EMA 代历史） | 轮位置观测恒 0 |
| CTBC | **轮-障碍接触力 xy + 3 帧滑窗**触发抬腿 | 0.6 s 正弦前馈，训练后退火为 0 | 连续爬 20 cm 台阶 |
| Wheel-Legged-Gym | 仿真 `net_contact_force` | 无 | 摩擦 [0.1,2.0]、延迟 [0,10] ms |

---

## 7. 对本项目的落地建议

### 7.0 先修前提（不做这三步，后面全是白做）

> **✅ 2026-09-29 已落地 2.5/3 项**：解析雅可比（`include/motion/leg_kinematics.h`）、
> 力矩通道校验（`Example58_TorqueChannelCheck`）、质量-质心辨识工具（`Example59_GravityMassIdentify`）。
> 剩下的只有"把 Example59 的结果**真机实测填进** `LINK_DYNAMICS`/`BODY_MASS`"。

| # | 事项 | 状态 | 说明 |
|---|---|---|---|
| **0.1** | **质量/惯量辨识** → 填 `LINK_DYNAMICS[3]` 与 `BODY_MASS` | 🟡 **工具已就绪，数值待实测** | 新示例 `Example59_GravityMassIdentify`：单关节小幅慢速扫描 + 对 `[sinθ, cosθ, 1]` 做 3 参数最小二乘 ⇒ **`G_j = m·g·d` 幅值**（`G_j = √(a²+b²)`，**不需要扫到力臂最大处**）；配合台秤称重反推 `com`，配合 Ex47 的 `J` 经平行轴定理得 `I_c = J − m·d²`。⚠️ 没有这一步，`f = J⁻ᵀτ` 只有相对意义 |
| **0.2** | **腿解析雅可比** | ✅ **已实现并校验** | `leg_kinematics.h` 新增 `leg_jacobian()`（髋系）/ `leg_jacobian_body()`（体系）/ `leg_cond_proxy()`（无量纲条件数代理）/ `leg_foot_force_body()`（`f = (Jᵀ)⁻¹τ`）/ `leg_foot_force_to_torque()`（`τ = Jᵀf`）。已与 `leg_fk`/`leg_fk_all` 中心差分逐元素比对（最大误差 <5e-4），并做 τ→f→τ 往返自检（9.5e-7） |
| **0.3** | **力矩通道校验** | ✅ **已实现** | `Example58_TorqueChannelCheck`：阻抗模式令 `kp=kd=0` ⇒ `τ = τ_ff`，逐电机施加 `0 → +T1 → +T2 → 0 → −T1 → −T2 → 0`，检查零偏/增益/符号/线性度，判定 `|τ_回读 − τ_指令| ≤ max(0.5, 25%·|τ_指令|)`；轮子默认跳过（会转起来），落盘 `log/sysid/torque_check_*.csv` |
| **0.4** | **条件数意识**（新增，由 0.2 的实测得到） | ✅ 已量化 | 实测 FL 腿条件数代理：膝伸直（`q3≈−0.211`）→ **1.8e7**；距伸直 0.02 rad → 103；0.05 rad → 41；DEFAULT_POSE → 15；STAND → 1.8。**力误差 ≈ 力矩误差 × cond** ⇒ 本项目摩擦 1.6~6.2 N·m 在 cond≈15 时不扣掉就是 25~90 N 的力误差。`leg_foot_force_body` 在 `cond > 50` 时直接拒绝返回 |

### 7.1 v0 —— 诊断层（零行为改动，建议立刻做）

| # | 方案 | 判据/公式 | 改动位置 |
|---|---|---|---|
| 0.4 | **轮空载摩擦基线** | 架空后扫 `ω_w ∈ [−5,5]`，拟合 `τ_w0(ω) = b·ω + fc·tanh(ω/ω_s)` | 复用 Ex54 采集链 + `tool/friction_id_offline.py` |
| 0.5 | **力矩因果低通 + 降采样** | `τ_f[k] = τ_f[k−1] + α(τ[k]−τ_f[k−1])`，τ≈15 ms @500 Hz → α≈0.13（参考 `lpf_tauJ_cutoff = 10 Hz`） | 新增/启用 `state_calc` 线程（`robot_app.cpp` 里现被注释） |
| 0.6 | **接触/打滑信号落盘** | 在 `S2RRecorder` 增加列（`f_norm(4)`、`slip(4)`、`contact_p(4)`），先只记录不参与控制 | `src/common/s2r_recorder.cpp` |

### 7.2 v1 —— 纯本体指示器（3–5 天，风险最低）

| # | 方案 | 公式 | 建议阈值 | 风险 |
|---|---|---|---|---|
| 1.1 | **轮打滑指示器**（最省） | `δ_i = \|ω_w,i·r\| − ‖v_foot,i,xy‖`（`v_foot` 由腿 FK 给，需 0.2） | `δ_i > 0.1 m/s` 且持续 3 帧 | 坡道/松软地误报 → 用 `n_support≥3` 且 `\|roll/pitch\|<15°` 门控 |
| 1.2 | **轮驱动扭矩残差**（着地/被顶住） | `r_i = τ_w,i − τ_w0(ω_i) − J_w·ω̇_i` | `\|r_i\| > 2~5 N·m` 连续 3 帧 | `J_w` 未知（可先忽略，轮惯量小） |
| 1.3 | **3 帧滑窗 + 滞环的接触置信度** | `p_i = σ((f̂_z,i − f_th)/Δf)`，`Δf ≈ 0.2·f_th`；再乘滑窗一致性 | `f_th = k·m·g/n_support`，`k≈0.25~0.4` | 需准的 `m·g`（含电池/负载）→ 依赖 0.1 |
| 1.4 | **接触数一致性检查** | `Σ_i f̂_z,i ≈ 1 ± 10%`；`n_support` 用于门控 1.1 | ±10% | 同上 |

**v1 的价值**：**不改训练侧、不改相位、不改控制**，纯真机侧诊断，能立刻验证信号质量，并覆盖"打滑"这个轮足最需要的一路安全信号。

### 7.3 v2 —— 支撑腿筛选 + 状态估计（1–2 周）

| # | 方案 | 公式 | 依赖 |
|---|---|---|---|
| 2.1 | 腿关节力矩 → 轮端竖直力 | `f_i^B = J_i(q)^{-T}τ_i`，`f_i^{W,z} = (R_WB f_i^B)_z` | 0.1（质量）+ 0.2（J）+ **重力前馈 `g_est(q)` + `LEG_FF_FC` 摩擦补偿** |
| 2.2 | **接触锚定里程计**（轮足专用补偿） | 触地记 `c_i^W`；`p̃ = c_i^W − R·p_ee^B`；`ṽ = −R(ω×p_ee^B + ṗ_ee^B)`；`Δψ_eff = Δψ − Δβ`，`β = θ_pitch + q_thigh + q_calf` | 2.1 + IMU 姿态 |
| 2.3 | **冲击抑制** | 触地上升沿 → 该腿 `kd` ×2~3、`kp` 降 20%，持续 50 ms | 2.1 |
| 2.4 | 状态估计门控（照抄） | `Q_leg *= (1+(1−trust)·100)`；`R_leg` 同理；**只在触地时更新足底高度** | 2.1 |
| 参考 | [arXiv 2602.17393](https://ar5iv.labs.arxiv.org/html/2602.17393) + [Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator) 已在**轮足**上验证（700 m 环 7.68 m ≈ 1.1%） | | |

### 7.4 v3 —— 进策略（需重训，2–4 周）

| # | 方案 | 做法 | 代价 |
|---|---|---|---|
| 3.1 | 接触概率进观测 | 仿真用 `net_contact_force` 做伪标签训练回归器（结构可抄 extreme-parkour 的 `Estimator [256,128,64]`）；作为额外观测（64→68 维） | **必须重训** |
| 3.2 | **HIM + phase 观测**（推荐组合） | `actor_input = [obs_history(6帧), sin/cos(φ), v̂(3), ẑ(16)]` —— HIM 的传感器配置与本项目完全一致，而 phase 是无噪声无延迟的确定性锚点，能减少 encoder 负担、加速收敛 | 中（~100 行 + 一次重训） |
| 3.3 | 打滑进奖励 | 照 CTBC：`wheel_spin = Σ max(0, \|r·ω\| − ‖v_foot‖ − 0.1)`，系数 −5.0 | 需重训 |
| 3.4 | 相位从常量改成命令 | 照 walk-these-ways：`commands[:,4] = frequency`（+ 课程分箱）；直接缓解"固定频率→大步幅→滑移" | 需重训 |
| 3.5 | **运动学腿效用 `u_i(t)` 作为新观测** | Bjelonic 的 `u_i = 1 − √((π_∥(r̃)/λ_∥)² + (π_⊥(r̃)/λ_⊥)²)`：**纯运动学、无需接触传感、可微、`u→0` 即物理上必须抬腿** | 中高（最有研究价值） |

### 7.5 sim2real 补偿清单（按 ROI，本项目的具体情况）

| 优先级 | 动作 | 建议参数 | 理由 |
|---|---|---|---|
| **P0** | **执行器/摩擦参数进仿真**：给关节补 `frictionloss / damping / armature`，用 `DCMotor` 四象限裁剪替代理想 PD，加功率上限 `\|τ\|ᵀ\|q̇\| ≤ P_max` | 参照 Barkour `damping=0.024, frictionloss=0.13, armature=0.011`；力矩上限用本项目 `TORQUE_CMD_LIMIT`（120/120/200/52） | 本项目 sim2sim 目前是纯 PD + 无摩擦，**这是最便宜的确定性收益** |
| **P0** | **`action_delay_steps` 落地** | 已实测 ≈24 ms ⇒ 至少 1 步（20 ms）；建议随机化 **[0, 2] 步** | `docs/ACTION_DELAY_MEASURE.md` 已有结论，但训练侧是否启用待确认 |
| **P0** | **观测侧删不可观测量**（本项目已合规） | `base_lin_vel` 已恒 0 ✅ | 保持，不要"顺手"把估出来的线速度塞进 actor |
| **P1** | **"20 秒空中数据"辨识**（含 T_d） | 0.1–2 Hz 位置 chirp + CMA-ES；复用 Ex47 链路 | 一次覆盖 Gap1 + Gap2 的一部分；不需要力矩传感器 |
| **P1** | **摩擦/质量/CoM 随机化 + 地形课程** | 摩擦 [0.3, 1.25] 起步（以真机实测范围为准）；质量 ±(1–5) kg；CoM ±5 cm | 需重训 |
| **P1** | **编码器零偏 + 关节零位偏置随机化** | `±0.015 rad`（mjlab）；`motor_offset ±0.02 rad` 直接加在目标上（WTW） | 真机零位误差是**系统性偏置**，白噪声覆盖不了 |
| **P2** | **HIM/DreamWaQ 式历史隐变量** | 见 7.4 的 3.2 | 需要状态估计与重训基建 |
| **P2** | **热/功率友好奖励**（轮足强烈建议） | `R_leg_effort_std`（EMA w=0.975/0.7）+ `R_action_curvature` + 抬脚分摊 | Go2-W 84 °C→60 °C 的实证 |
| **P3** | 真机在线微调 | 需自动 reset + 板载奖励估计 | 成本高，末期再考虑 |

### 7.6 明确**不建议**做的事

| ❌ 不建议 | 原因 |
|---|---|
| 照搬 Aoi/Sun 的**接触触发相位重置** | 轮足无清晰 GRF 上升沿；有噪时可能**永不重置**（§4.3） |
| 照搬 champ 的 `phase_generator.h` | 它是**纯开环**（`last_touchdown_` 是内部整周期计数），swing 周期硬编码 0.25 s |
| 只做**二值**触地位 | 真机"轻接触"假阳性多，硬阈值下游里程计**不如连续概率**（§0 第 5 条） |
| 把 `feet_air_time`（奖励滞空 0.5 s）直接搬来 | 那是**足式**的长步目标；轮足长时间离地通常是故障 → 应改成"轮子离地惩罚" |
| 靠**轮式里程计硬算位移** | Go2-W 明确因充气胎难判接触而放弃该路线 |
| 用**单帧**力矩/力做阈值比较 | 轮速已有一阶低通（α=0.2），τ 必须同样滤；触地弹跳是最大假正例来源 |
| 在没有 `g_est(q)` 的情况下用 τ 估力 | 本项目腿重力矩 8–12 N·m 与库仑摩擦 1.64–6.17 N·m 同量级，不补则 `f_z` 完全不可用 |
| 把 RMA 原版直接移植 | 它需要 4 维足端接触开关（来自力传感器），本项目没有 → 用 HIM/DreamWaQ |

---

## 8. 待验证 / 存疑清单（**不要当事实传播**）

1. **接触力阈值的绝对值**：ANYmal 的 `f_th` 论文只写 "configurable threshold"，[CTBC] 也只写 "preset threshold" —— **本轮未找到任何"30 N / 40 N"的一手出处**。本报告中的 `k=0.25~0.4`、`F_high/F_low≈1.5~3×`、`T_min=20~50 ms`、`τ_th=2~5 N·m`、打滑 `|s|>0.25 持续 100 ms` 均为**【推断】工程量级**，必须真机标定后才写进代码。
2. **CTBC 的消融数字**：4 组对照（CTBC / w/o feedforward / w/o contact-trigger / w/o both，80000 iter，以地形等级为度量）设置已确认，但**结论表格在抓取到的正文中被截断**，未获数值。
3. **Camurri 2017 逻辑回归的具体系数/阈值**：通过 [arXiv 2606.05501] 的转述确认了方法，**未读到原文系数**。（本项目可参考的是 `mcx-lab` 的 `β0=−20, β1=0.7`。）
4. **AFO / CPG 的自适应律与 Matsuoka 方程**：EPFL Infoscience 的一手 PDF 未能下载，公式按教科书标准形式给出，**未逐字核验**。
5. **`leggedrobotics/legged_control` 与 `leggedrobotics/legged_state_estimator` 仓库 404（未公开）**；可用同源的 `qiayuanl/legged_control` 与 `mcx-lab/legged_state_estimator` 替代。
6. **Quad-SDK 的 `Gait` / `Phase` 类不存在**（main 分支全仓 grep 仅一处注释命中）；功能由 `LocalFootstepPlanner::computeContactSchedule` 承担。
7. **Cheetah-Software 里没有任何"足端力 > 阈值 ⇒ 触地"的逻辑**；TI 板回传的 `force[3]` 是阻抗控制算出的**期望力**。MIT 著名的 Bledt ICRA 2018 接触模型融合**未进入该开源仓库**。
8. **ANYmal 真机的 `measuredContactFlags` 生产者未开源**（`GaitAdaptation` 在开源仓库只在测试中被调用）；ANYmal 的轮子半径、足端力传感器规格未取到一手文档。
9. **"相位 clock 对轮足是帮助还是拖累"的定量消融未找到**；§4.4 的判断是【推断】。
10. **OpenQuadruped / solo-ODRI 的 CPG 实现未验证**（前者官方仓库未定位到，后者只定位到组织页）。
11. **MuJoCo `friction` 三分量（slide/torsion/roll）的官方文档原文未直接抓取**；"`condim=3` 时无滚动摩擦"标为【推断】，落地前请用一手文档复核。
12. **本项目侧**：`JOINT_IMPEDANCE[..][THIGH].tau_ff` 代码 `+5.0f` vs 注释/提交信息 `-5`；`UPPER_LIMIT_THETA1_DEG` 代码 `15.0f` vs 旧注释 `+30°` —— 两处均**未改动数值**，待现场确认。

---

## 9. 主要来源

**RL 训练/部署栈**
[legged_gym](https://github.com/leggedrobotics/legged_gym)｜[rsl_rl](https://github.com/leggedrobotics/rsl_rl)｜[Isaac Lab](https://github.com/isaac-sim/IsaacLab)｜[IsaacGymEnvs](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)｜[unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)｜[unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab)｜[walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)｜[extreme-parkour](https://github.com/chengxuxin/extreme-parkour)｜[HIMLoco](https://github.com/InternRobotics/HIMLoco)｜[DreamWaQ](https://github.com/Manaro-Alpha/DreamWaQ)｜[rl_locomotion (RMA)](https://github.com/antonilo/rl_locomotion)｜[barkour_robot](https://github.com/google-deepmind/barkour_robot)｜[ASE](https://github.com/nv-tlabs/ASE)

**经典 MPC / WBC 栈**
[Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)｜[OCS2](https://github.com/leggedrobotics/ocs2)｜[legged_control](https://github.com/qiayuanl/legged_control)｜[unitree_guide](https://github.com/unitreerobotics/unitree_guide)｜[quad-sdk](https://github.com/robomechanics/quad-sdk)｜[free_gait](https://github.com/leggedrobotics/free_gait)｜[elevation_mapping](https://github.com/ANYbotics/elevation_mapping)

**状态估计 / 接触估计**
[mcx-lab/legged_state_estimator](https://github.com/mcx-lab/legged_state_estimator)｜[invariant-ekf](https://github.com/RossHartley/invariant-ekf)｜[Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator)｜[slip_detection_DOB](https://github.com/UMich-CURLY/slip_detection_DOB)

**轮足**
[Wheel-Legged-Gym](https://github.com/clearlab-sustech/Wheel-Legged-Gym)｜[awesome-wheeled-legged](https://github.com/XinLang2019/awesome-wheeled-legged)｜[Keep Rollin', RA-L 2019](https://arxiv.org/abs/1809.03557)｜[Rolling in the Deep, RA-L 2020](https://arxiv.org/abs/1909.07193)｜[Whole-Body MPC for Wheeled-Legged, ICRA 2021](https://arxiv.org/abs/2010.06322)｜[Ascento, ICRA 2019](https://ar5iv.labs.arxiv.org/html/2005.11435)／[RA-L 2020](https://ar5iv.labs.arxiv.org/html/2005.11431)｜[Lee et al., Science Robotics 2024](https://arxiv.org/abs/2405.01792)｜[Go2-W 导航](https://arxiv.org/html/2606.21387v1)｜[CTBC](https://ar5iv.labs.arxiv.org/html/2509.02986)｜[Contact-Anchored Proprioceptive Odometry](https://ar5iv.labs.arxiv.org/html/2602.17393)

**接触检测 / 相位 / 估计理论**
[Camurri RA-L 2017 概率接触估计](https://www.robots.ox.ac.uk/~mobile/drs/Papers/2017RAL_camurri.pdf)｜[Hwangbo IROS 2016](https://ieeexplore.ieee.org/document/7759570)｜[Contact-CNN, CoRL 2021](https://proceedings.mlr.press/v164/lin22b/lin22b.html)｜[Learning Contact Representation](https://arxiv.org/html/2606.05501v1)｜[ContactNet, RA-L 2022](https://arxiv.org/html/2202.05481)｜[Generalized Momentum Observer](https://ar5iv.labs.arxiv.org/html/2505.03044)｜[Aoi IROS 2011 相位重置](http://space.kuaero.kyoto-u.ac.jp/aoi/pdf/IROS2011_AOI2.pdf)｜[Sun et al. 2021 PR vs PM](https://www.frontiersin.org/articles/10.3389/fncir.2021.706064/full)｜[Margolis & Agrawal 周期奖励组合](https://arxiv.org/abs/2011.01387)

**sim2real**
[Bjelonic IJRR 2026 系统辨识](https://arxiv.org/abs/2509.06342)｜[Hwangbo Science Robotics 2019 ActuatorNet](https://arxiv.org/abs/1901.08652)｜[Rudin 2021 少而准 DR](https://arxiv.org/abs/2109.11978)｜[Tan 2018](https://arxiv.org/abs/1804.10332)｜[RMA](https://arxiv.org/abs/2107.04034)｜[HIM](https://arxiv.org/abs/2312.11460)｜[DreamWaQ](https://arxiv.org/abs/2301.10602)｜[MMDR 多模态延迟随机化](https://arxiv.org/abs/2109.14549)｜[ERFI 最小动力学随机化](https://arxiv.org/abs/2209.12878)｜[UAN](https://arxiv.org/abs/2502.10894)｜[VILENS](https://arxiv.org/abs/2107.07243)｜[Smith 2022 真机微调](https://arxiv.org/abs/2110.05457)｜[Ha 2020 真机从零学](https://arxiv.org/abs/2002.08550)

**模型/参数**
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)（barkour_vb / unitree_go2 / unitree_go1 / anybotics_anymal_c）｜[unitree_ros URDF](https://github.com/unitreerobotics/unitree_ros)

**本项目内部**
`memory/FACT.md`、`docs/SIM2REAL_DEPLOY.md`、`docs/FRICTION_SYSID.md`、`docs/ACTION_DELAY_MEASURE.md`、`src/strategy/rl_controller.cpp`、`src/motor/ele_motor.cpp`、`include/motion/{leg_kinematics,robot_calibration}.h`、**`Doc/集成驱动器使用说明-V1.pdf`**（SEmotor-C：速度环 `Dt = 50 µs`、阻抗/位置环公式）
