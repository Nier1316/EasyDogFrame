# 项目事实

> 时效性标注：✅ 最新 / ⚠️ 过时（值或结论已变） / 🔴 失效（对象已删除） / 🗄️ 历史存档。
> 本文件为工程事实总表，最后逐项对照代码：**2026-09-29**（HEAD `98ef8ea` + 本次审查修复）。

## 工程位置与构建
- 主工程：`/home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify`（四足机器狗电机控制框架，CMake + C++17，分层重组后）。
- 构建：`cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)`，产物 `bin/can_motor_app`；2026-09-29 全量编译通过。
- 分层（代码注释里的 L0~L7）：`include/` 与 `src/` 同名分层 —— `common`（L0 公共类型 / 日志分类 / CSV 与 S2R 落盘）、`transport`（L1 `CanTransport` + 两后端）、`motor`（L2 `EleMotor`/MIT 编解码、L3 `MotorManager` 单例）、`runtime`（L4 `ThreadManager`/`RobotApp`/电机收发线程）、`motion`（L5 `MotionController`/腿运动学/整机参数）、`strategy`（L6 观测+PD+MLP / 真机↔URDF / IMU / 手柄）、`app`（L7 `main.cpp` + 示例）。
- 🔴 已删除（勿再引用）：`bsp/`（BspCan → CanTransport）、`thread/`（thread_manager 已移入 runtime/）、`RoboTasks/`、`motor_drive/`、`src/base/`、`data_types.h`、`RobotDog`（从未实现）。
- 传输层：`CanTransport` 纯虚接口 + `CanetTransport`(TCP) / `Usb2CanTransport`(达妙)，配置结构 `TransportConfig`。**默认后端 = 达妙 Usb2CanTransport**（`src/motor/motor_manager.cpp` 兜底）；CANET **已弃用**，仅 Example27/28 直接使用。达妙链接需新版 libstdc++(GLIBCXX_3.4.32) + libusb(≥1.0.26)，CMake 自动探测 conda lib 路径（`-DCONDA_LIB_DIR` 可覆盖）。

## 硬件拓扑与节拍
- 4 路总线 × 4 电机 = 16 电机：motor_id 1=髋 hip、2=大腿 thigh、3=小腿 calf、4=轮 wheel；tx_id=motor_id，rx_id=50+motor_id。
- 线程：`motor_receive` / `motor_send` 各 **2ms（500Hz）**，SCHED_FIFO 优先级 80（`src/runtime/motor_io.cpp`）；`robot_calibration.h` `CONTROL_HZ=500`。
- 电机环：轮子走**固件 SPEED 速度环**（`SendSpeed(vel, kvp, ki)`，固件内部 1kHz 闭环）；腿关节走固件**阻抗环**（kp/kd/tau_ff）。
- 标定单一真值来源：`MOTOR_CALIBRATION[4][4]`（`include/motor/motor_calibration.h`）；`robot_calibration.h` 直接引用其 `pos_offset`，勿手抄字面量。

## RL 部署（dogurdf 轮足策略）
### 权重链路（✅ 2026-09-22 起 = iteration_9754）
- 真机：`weights/iteration_9754.pkl` → `tool/export_policy.py`（默认 `CKPT`，`--ckpt` 可覆盖）→ `include/strategy/policy_weights.h` + `include/strategy/policy_test_ref.h`；三者 mtime 2026-09-22，与 HEAD `98ef8ea`（权重切换 9754，降低抬腿幅度）一致。
- 网络：`64 → 512 → 256 → 128 → 16`，中间层 ELU 激活、输出层无激活（`include/strategy/mlp.h`）。
- sim2sim：`dogurdf_sim2sim_deploy/run_sim2sim.sh` 默认 checkpoint = `../weights/iteration_9754.pkl`（**与真机同一份**），可用 `SIM2SIM_CKPT=<path.pkl>` 覆盖；此前硬编码 `iteration_3000`，2026-09-29 修正。
- 🗄️ 历史存档（**不再与真机同步**）：`dogurdf_sim2sim_deploy/checkpoints/dogurdf_velocity/{iteration_450,iteration_3000}.pkl`；`weights/` 现有 `iteration_9754.pkl`（当前）+ `iteration_2100.pkl`（历史），`iteration_3500/4350/5350.pkl` 已于 2026-09-29 删除。
- ⚠️ 9754 / 2100 / 3000 三个 checkpoint 的 actor 形状相同（64-512-256-128-16），可互换加载。

### 关键常量（`include/strategy/rl_controller.h`）
- `NUM_JOINTS=16`、`NUM_LEG_JOINTS=12`、`NUM_WHEELS=4`、`OBS_DIM=64`、`ACTION_DIM=16`
- `ACTION_SCALE=0.25`、`WHEEL_VEL_SCALE=12.5`
- `LEG_KP=250`、`LEG_KD=4`（对齐 `dogurdf_sim2sim_deploy/src/sim2sim.py` 的 `LEG_KP/LEG_KD` 默认值；⚠️ 300/10 是 V30 参数勿混淆，历史曾用 250/40。注：当前部署权重 `iteration_9754` 的训练侧 PD 参数本仓库未记录，如需追证请查 `RL_Train/code` 训练配置）
- `WHEEL_KD=1.0`（**仅 RL 阻抗诊断路径用**；轮子实际走固件 SPEED 环，此量不参与真实控制）
- `CONTROL_DT=0.02`（50Hz）、`GAIT_CYCLE=0.6`、`GAIT_OFFSET={0.0, 0.5, 0.5, 0.0}`
- 轮子 SPEED 环：`WHEEL_KVP=3.0`、`WHEEL_KVI=0.05`（⚠️ 历史 0.3 在 RL 上积分过强致疯转）、`WHEEL_SOFT_KVP=0.1`（起立/回位软启动）、`WHEEL_CMD_ALPHA=0.2`（轮速目标低通 @50Hz）、`WHEEL_CMD_MOVE_THR=0.1`（移动/静止门控，站立锁轮）
- ⚠️ `WHEEL_CMD_DEADZONE=0.5` **已弃用**（会削减转向差速 action，改由门控接管），仅留历史值
- 腿摩擦前馈：`LEG_FF_ENABLE=true`、`LEG_FF_TANH_K=0.5`；`LEG_FF_FC[12]`（POLICY 序，Nm）= FL 2.40/3.36/4.82、FR 1.64/1.89/4.54、RL 2.59/2.13/6.17、RR 1.84/2.56/5.38（hip/thigh/calf）；`LEG_FF_FV[12]` 全 0（Ex54 回归 b 不可靠）
- 🔴 **扭矩限幅不在本文件**：`LEG_TORQUE_LIMIT` 已删除。真机限幅由 `include/motor/ele_motor_def.h` 的 `MOTOR_LIMITS`（协议量程）+ `TORQUE_CMD_LIMIT`（编码前命令 clamp）决定 = **Hip 120 / Thigh 120 / Calf 200 / Wheel 52 Nm**（2026-08-30 固件改限幅，2026-09-04 Hip/Thigh 110→120）。

### 观测布局（`src/strategy/rl_controller.cpp`，共 64）
`base_lin_vel(3)=0 | base_ang_vel(3) | projected_gravity(3) | joint_pos_rel(12) | joint_vel(16) | last_action(16) | command(3) | gait_phase(8)`
- `gait_phase` 为**分组**布局：`obs[56..59]=sin(2πφ)×4脚`、`obs[60..63]=cos(2πφ)×4脚`（**不是**交错）。φ = `fmod(step*0.02/0.6 + GAIT_OFFSET[foot], 1)`；`0ee431f`（2026-09-07）由交错改为分组以对齐训练。
- `world2self(q,v)` 等价 `brax rotate(v, quat_inv(q))`；`projected_gravity = world2self(quat, [0,0,-1])`。

### 关节序与真机↔URDF 转换
- **顺序置换（2026-09-30 更名）**：`POLICY_TO_CAN = {0,1,2,4,5,6,8,9,10,12,13,14,3,7,11,15}`（policy→can）；
  `CAN_TO_POLICY = {0,1,2,12, 3,4,5,13, 6,7,8,14, 9,10,11,15}`（can→policy）。POLICY 序 = 12 腿 + 4 轮；CAN/MJX 序 = 每腿 hip/thigh/calf/wheel。
  ⚠ **旧名 `POLICY_TO_MJX`/`MJX_TO_POLICY` 的字面含义与实际语义正好相反**（数组值没错、名字骗人；旧 `POLICY_TO_MJX` 实为 can→policy）。
  已按语义重命名，旧名保留为**引用别名**（`const int (&OLD)[N] = NEW;`）故既有调用点行为不变；新代码一律用新名。
  自检：`/tmp/perm_test`（9 项：腿/轮范围、双向互逆、双射、抽查、别名等价）全部通过。
- `DEFAULT_POSE`（POLICY 序）：hip 0、thigh +0.20、calf −0.35、wheel 0（四腿相同），与 dogurdf `NOMINAL_*` 一致。
- `CONV_A`：每腿 hip/thigh/calf = `+1, −1, +1`，四轮 `+1`；`CONV_B`：每腿 `+0.0297, −0.9624, −1.2832`，四轮 `0`。
- 关系：`URDF = CONV_A*GetStatus + CONV_B`；`GetStatus = (URDF − CONV_B)/CONV_A`（`src/strategy/sim2real_conv.cpp`）。

### 标定表（`include/motor/motor_calibration.h`）
- `MOTOR_CALIBRATION[4][4]`（`{pos_scale, vel_scale, pos_offset}`，**vel_scale 必须 = pos_scale**）：

| CAN | Hip | Thigh | Calf | Wheel |
|---|---|---|---|---|
| 0 (FL) | −1, −1, 0.611 | +1, +1, 0.441 | −1, −1, 0.211 | −1, −1, 0 |
| 1 (FR) | +1, +1, 0.611 | −1, −1, 0.441 | +1, +1, 0.211 | +1, +1, 0 |
| 2 (RL) | +1, +1, 0.611 | +1, +1, 0.441 | −1, −1, 0.211 | −1, −1, 0 |
| 3 (RR) | −1, −1, 0.611 | −1, −1, 0.441 | +1, +1, 0.211 | +1, +1, 0 |

- ⚠️ FR hip `pos_offset` 代码 = **0.611**；曾评估 0.78（实测 FR hip 下发位置偏 RL +0.18），但**未落地**，勿按 0.78 写。
- 方向约定：反馈侧 `pos*scale+offset`、`vel*scale`、`torque*pos_scale`；发送侧 `pos=(pos−offset)*scale`、`vel*scale`、`torque*pos_scale`。按字段 helper 为 `ApplyMotorCalibrationPos/Vel/Torque`，`ApplyMotorCalibration` 是三者的组合。

### 关节阻抗与整机参数（`include/motion/robot_calibration.h`）
- `JOINT_IMPEDANCE[4][3]`（kp/kd/tau_ff）：hip 300/10/**−10**；thigh 250/10/**代码 +5.0f**；calf 250/10/**+12（CAN0/1）/ +20（CAN2/3）**。
- ⚠️ **待现场确认**：thigh `tau_ff` **代码 = +5.0f**；历史行内注释与提交 `6546688` 曾写 `−5`，现注释已标注为 +5（代码值）+ 待核实。**保持代码值不变**，上真机前人工确认符号。
- `CONTROL_HZ=500`；连杆 `LEG_L1=0.1308`、`LEG_L2=0.34`、`LEG_L3=0.343`；机身 `BODY_LENGTH=0.653`、`BODY_WIDTH=0.16`、`BODY_SIZE_HEIGHT=0.06`。
- 关节限位（deg）：θ1 `[−60, +15]`、θ2 `[−70, +90]`、θ3 `[+60, +180]`。
- ⚠️ **待现场确认**：`UPPER_LIMIT_THETA1_DEG` **代码 = 15.0f**，但旧注释写"放宽到 +30°"。**保持代码值不变**，勿按 +30 写。
- 站立姿态（真机标定角）：`STAND_HIP/THIGH/CALF = 0 / −60 / +60`；趴下姿态：`LIE_DOWN = +11.4 / −55.2 / +12.6`（Example50 标定）。
- `WHEEL_KVP=3.0`（与 `rl::WHEEL_KVP` 同值）。
- ⚠️ `LINK_DYNAMICS` / `BODY_MASS` / `MOTOR_DRIVE` **仍是 TODO 占位未实测**（全 0 / 占位 1.0），动力学解算勿依赖。

### 轮控安全（🔴 旧策略侧软限位已删除）
- 轮速保护由 `MotorManager`（`src/motor/motor_manager.cpp`）承担：`WHEEL_ESTOP_KVP=3.0`、`WHEEL_ESTOP_VEL_CMD=0.02 rad/s`（避 v=0 固件歧义）、`WHEEL_ESTOP_VEL=15.0 rad/s` 自动触发、`WHEEL_ESTOP_GRACE_TICKS=1000`（2s 静默窗口）。自动超速为**瞬态**（只当次制动）；手动 `WheelEmergencyStop()` 才保持。
- 固件模式同步 `MODE_SETTLE_TICKS=20`（写模式后等 ~40ms 再发控制帧）；轮速一阶低通 α=0.2（仅轮，`src/motor/ele_motor.cpp`）。
- 🔴 **2026-09-29 删除**（迁移 SPEED 后无调用者的死代码；历史数值留档，**勿再依赖**）：
  - `rl::wheel_torque()` = `kd·(WHEEL_VEL_SCALE·action − vel) + WHEEL_FF 前馈 + 速度软限位`
  - `WHEEL_FF[4][2]` = FL{+0.6,−0.6} FR{+0.5,−0.4} RL{+0.8,−0.8} RR{+0.5,−0.4}（Ex35 实测，CAN2 阻力最大）；`WHEEL_FF_ENABLE=false`
  - `WHEEL_SOFT_LIMIT_ENABLE`（曾 true）、`WHEEL_VEL_SOFT_LIMIT=5.0 rad/s`、`WHEEL_SOFT_LIMIT_TORQUE=30.0 Nm`（2026-08-30 由 10→30）
  - `WHEEL_TORQUE_LIMIT=52.0`（曾 53）、`LEG_TORQUE_LIMIT=250`（历史 150，从未参与 clamp）

### sim2sim 对齐与已知差异（`dogurdf_sim2sim_deploy/src/sim2sim.py`）
- 与真机一致的常量：`ACTION_SCALE=0.25`、`WHEEL_VEL_SCALE=12.5`、`LEG_KP/LEG_KD=250/4`、`GAIT_CYCLE_TIME=0.6`、`GAIT_PHASE_OFFSETS=(0,0.5,0.5,0)`、gait_phase 分组布局、`CONTROL_DT=0.02`。
- 积分参数：`SIM_DT=0.005`、`DECIMATION=4`（→ 控制 50Hz）。⚠️ 旧文档写的 `SIM_DT=0.002/DECIMATION=10` **已过时**；本仓库无 `MOTOR_DECIMATION`。
- `--real_actuator` 把仿真扭矩上限对齐真机（hip/thigh 120、calf 200、wheel 52）并默认 1 步动作延迟；`--wheel_gate` 复现真机站立锁轮门控（阈值 0.1）；不带 `--real_actuator` 时仿真仍用 `LEG_TORQUE_LIMIT=250 / WHEEL_TORQUE_LIMIT=53`。
- ⚠️ 仍存在的建模差异：sim2sim 轮子走 `kd·(w_target − qd)`（`WHEEL_KD=2.0`），真机轮子走固件 SPEED 环（`kvp=3.0, ki=0.05`）。
- 时延：真机纯传输延迟实测 ≈24ms（可信区间 18~30ms），建议 `action_delay_steps=1`（见 `docs/ACTION_DELAY_MEASURE.md`）；**训练是否含 action delay 的旧文档说法互斥，以 `RL_Train/code` 训练配置为唯一真源**。
- 固件速度环：驱动器手册（`Doc/集成驱动器使用说明-V1.pdf`）明确 `τ_des = Kvp(ω_des−ω_act) + Kvi·Dt·Σ(ω_des−ω_act)`，**`Dt = 50 µs`**（固件内 20 kHz）——远快于上位机 500 Hz 指令率，故 sim2sim 的差异主要在**指令路径/饱和/死区**而非内环带宽。

## 触地检测 / 相位 / sim2real（专题）
- 📌 **专题文档：`docs/CONTACT_PHASE_SIM2REAL.md`**（2026-09-29）—— 开源框架（legged_gym / Isaac Lab / unitree_rl_{gym,mjlab} / walk-these-ways / HIM / DreamWaQ / RMA / Cheetah-Software / OCS2 / Quad-SDK / ANYmal-W / Go2-W / CTBC / Wheel-Legged-Gym）在**触地接触检测、相位检测、sim2real gap 补偿**上的代码级做法，以及本项目分阶段落地建议。
- 现状：**本项目没有任何触地/接触检测代码**（全仓 grep 仅命中"腿悬空不触地"之类注释）。相位为**开环** `gait_phase`（8 维分组 sin/cos）。
- 可用的接触信号条件：**16 路关节力矩反馈（含 4 轮，500 Hz）** + 腿 FK + 轮子连续接地。**缺**：足端/轮端力传感器、质量与惯量模型（`LINK_DYNAMICS` 全 0、`BODY_MASS=0`）、`leg_kinematics.h` 的**雅可比**（需新增）。
- 落地顺序（详见专题文档 §7）：**v0 质量/惯量辨识 + 解析雅可比 + 力矩通道校验** → v1 纯本体指示器（打滑 `δ>0.1 m/s`、轮驱动扭矩残差 2~5 N·m、3 帧滑窗连续接触置信度）→ v2 支撑腿筛选 + 接触锚定里程计 → v3 才考虑接触概率进策略（需重训）。
- ⚠️ 不要做**接触触发的相位重置**：轮足无清晰 GRF 上升沿，有噪时可能永不重置（ANYmal 的 `GaitAdaptation` 也只做"提前触地"一种）。

## 示例（demo）与运行方式
- 示例总数 **43**，编号 **17~60**（编号不连续；1~16 已清理，**Example55 从未实现，2026-09-29 删除其声明与注释调用**；**58/59/60 为 2026-09-29 新增**）。
- 分发机制：改 `src/app/main.cpp` 的注释 + 重新编译，**无命令行参数、无注册表**。
- **当前激活 = `Example37_RLTeleopControl`**（`main.cpp` 结尾唯一未注释的调用）。

### 触地检测前提件（2026-09-29 新增，见 `docs/CONTACT_PHASE_SIM2REAL.md` §7.0）
- **解析雅可比**（`include/motion/leg_kinematics.h`）：`leg_jacobian`（髋系）/ `leg_jacobian_body`（体系，= R·J_hip）/ `leg_cond_proxy`（无量纲条件数代理）/ `leg_foot_force_body`（`f = (Jᵀ)⁻¹τ`，`cond > 50` 拒绝）/ `leg_foot_force_to_torque`（`τ = Jᵀf`）。已与 `leg_fk`/`leg_fk_all` 中心差分逐元素比对（<5e-4）+ τ→f→τ 往返自检（9.5e-7）。
- **条件数实测**（FL，体系）：膝伸直（`q3 ≈ −0.211`，即 `t3_int=0`）→ **1.8e7**；距伸直 0.02 rad → 103；0.05 rad → 41；DEFAULT_POSE → **15**；STAND → **1.8**。⇒ **力误差 ≈ 力矩误差 × cond**：本项目腿摩擦 1.6~6.2 N·m、重力矩 8~20 N·m，不扣掉就是几十牛的力误差。
- **角度坐标**：`leg_fk`/`leg_jacobian` 的输入 = `GetStatus().position` = `SendImpedance` 的位置参数（**同一坐标**，Ex18 把 `leg_ik` 输出直接下发）；不要再减 `THETA*_OFFSET`。
- **新增示例**（`src/app/examples/ex_sysid.cpp` / `include/app/examples/ex_sysid.h`）：
  - **Example58 `TorqueChannelCheck`**：力矩通道校验（零偏/增益/符号/线性度）。`kp=kd=0 ⇒ τ=τ_ff`，逐电机 `0→+T1→+T2→0→−T1→−T2→0`；轮子默认跳过（会转起来）。落盘 `log/sysid/torque_check_*.csv`。
  - **Example59 `GravityMassIdentify`**：重力矩系数 `G_j = m·g·d` 与质量-质心。单关节小幅慢扫 14 点 → 对 `[sinθ, cosθ, 1]` 最小二乘 ⇒ `G_j = √(a²+b²)`（与角度零点约定无关）；配合称重反推 `d`、配合 Ex47 的 `J` 得 `I_c = J − m·d²`；打印建议的 `LINK_DYNAMICS`/`BODY_MASS`。落盘 `log/sysid/gravity_summary_*.csv`。
- ⚠️ 分工：`J/B/f_c/K_g` 由 **Example47** 辨识、腿摩擦由 **Example54** 辨识 —— 58/59 **不重复**，只补 Ex47 给不出的"大范围重力矩幅值"与"质量/质心"。
- 文件分工：`ex_basic.cpp`（17~23，7 个）、`ex_diag.cpp`（24, 26~29, 33, 34, 39~50, 54, 57，21 个）、`ex_rl.cpp`（25, 30~32, 35~38, 51~53, 56，12 个）、**`ex_sysid.cpp`（58~59，2 个）**、**`ex_probe.cpp`（60，1 个）** = 共 **43 个**。
- 关键示例：Ex25 完整 RL + 手柄；Ex30 离线链路回归（不碰 CAN）；Ex34 轮子方向核对；Ex35 轮摩擦前馈标定（历史）；Ex36 RL 站立循环；**Ex37 RL 遥操作（当前激活）**；Ex38 动作延迟辨识；Ex47 整狗 chirp 辨识；Ex49 轮 SPEED 环 kvp 扫描；Ex51 站立→趴下；**Ex58 力矩通道校验**；**Ex59 重力矩/质量-质心辨识**；Ex54 吊装摩擦辨识；Ex56 固定 yaw 遥测落盘；Ex57 单腿零位对照（验证 CONV_A/B）。
- `examples_common` 只有 4 个 helper：`RawTerminal`、`poll_key`、`g_rl_stop`+`rl_signal_handler`、`EnableRlFrictionFF/DisableRlFrictionFF`。
- ⚠️ 安全现状（2026-09-29 脚本复核）：42 个示例中 **29 个会使能电机**（脚本按函数体内直接调 `EnableMotor`/`PreEnableZeroTorque` 统计为 27 个；Ex58/59 经公共 helper `init_zero_torque()` 使能），其中 **13 个装了 `SIGINT` 急停** —— Ex25/34/35/36/37/38/51/52/53/54/56/**58/59**；其余 **16 个**（Ex18/19/20/21/22/23/29/32/41/44/45/46/47/48/49/57）会使能但没有软急停，运行前须留安全距离、可随时断电。

### sim2real 数据回馈（专题，见 `docs/SIM2REAL_DATA_FEEDBACK.md`）
- 原则：**数据不进训练，进训练的是仿真里的 `p(s'|s,a)` 与观测模型**。录音必须凑齐"**我命令了什么 + 实际发生了什么 + 共用一个单调时钟**"，才能把控制延迟与执行器动态分开。
- **记录现状**：`recv_*.csv`（raw+cal、500 Hz、默认开）、`rl_*.csv` / `rlrun_*/trace.csv`（50 Hz、含 quat/gyro/τ）已有；
  **缺口**：`LogFileSwitch::SEND`（下发的 pos/vel/kp/kd/τ_ff）**默认关** → 做 sim2real 必须打开；
  **IMU 无独立落盘**（只有 50 Hz 进 trace）；**Vbus 从未读**（手册有 `MOTOR_OR_Vbus 0x07`，`ReadParam` 可读）；温度同理。
- **最大的结构性 gap = 轮子**：真机是固件速度环（手册 `Dt = 50 µs`），sim2sim 是 `kd·(w_target − qd)`（`WHEEL_KD=2.0`）—— **模型结构不同，调 DR 补不上**。要么把仿真轮子建成速度伺服（主流做法），要么真机改成力矩控制，二者不可混。
- **四条消费路径**：① 参数辨识→改仿真模型（最快，不需重训）；② 实测离散度→定 DR 区间（"少而准"）；③ **轨迹回放拟合**（真机录 30~60s 500Hz 指令+反馈 → `sim2sim --replay` → CMA-ES 拟合参数，残差谱还能分诊）；④ 真机微调/历史隐变量（最后做，且只治感知误差）。
- 已有资产：`tool/compare_sim2real.py`（**缺 τ 对比与互相关时移**，建议先补）、`run_dual_compare.sh`、`sim2sim.py --record`。

### 统一 sim2real 数据集录制（2026-09-29 新增，`include/common/s2r_dataset.h`）
- **一行 = 一个 500 Hz 收发节拍**：`wall_ms,t_ms` + 16 电机 `(c_mode + c_pos/c_vel/c_kp/c_kd/c_tau + m_pos/m_vel/m_tau + m_temp + m_vbus)` + `gyro(3)/quat(4)` + `cmd(3)` = **188 列**。
  产物 `log/dataset_<ts>.csv` + `.meta.txt`。索引是 **CAN 顺序**（`i = can*4 + motor_id-1`），`c_*` 语义随 `c_mode_i` 变化。
- **实时性**：生产者（`MotorManager::SendOnce`）只做一次 ~0.6 KB 拷贝进 **8192 槽无锁环形缓冲**，队列满则丢弃并计数；格式化与写盘在独立写线程（1 MB 全缓冲，1 s flush 一次）。**绝不阻塞 500 Hz 控制环。**
- 采集时自动以 **1 Hz** 轮询 `Vbus(0x07)`（每路 1 号电机，4 个）与 `temperature(0x0D)`（16 个）→ 共 20 帧/s（稳态 8000 帧/s 的 0.4%）。
  ⚠ `MOTOR_OR_Vbus` 在 Ex24 参数表里**从未读过**，属首次启用；回读恒 0 就在分析里忽略该列。为此 `EleMotor` 新增 `current_vbus` 字段。
- 离线工具（`tool/`，每个都有 `--selftest`）：`dataset_health.py`（体检：频率/丢帧/范围/"能用来做什么"）、`delay_fit.py`（延迟 T_d + 一阶 τ）、`wheel_servo_fit.py`（轮速伺服：T_d/τ/K/死区/饱和 + `kd_equiv`）；`compare_sim2real.py` 增补 τ 对比与互相关时移。
  - 已验证：`delay_fit` 用**独立生成**的 188 列数据集（真值 T_d=26.0 ms / τ=30.0 ms）→ 输出 T_d **26.0 ms**、τ 29.0 ms、K 1.000；`wheel_servo_fit` 自测 → T_d 18.0/18.0、τ 40.0/40.0、死区 0.40/0.40、ω_max 精确。
  - ⚠ 已知不一致：轮子 `kd` 在 `dogurdf_sim2sim_deploy/src/sim2sim.py` 是 **2.0**、在 `include/strategy/rl_controller.h`（`rl::WHEEL_KD`）是 **1.0**（注释记为"历史 2.0 → 1.0"）。改仿真前先确认用哪个。
- 🚩 **真机操作手册：`docs/REAL_ROBOT_HANDOVER.md`**（自足：安全铁律 / T0~T8 逐步清单 / 判据 / 异常处理 / 回传报告模板）。
- 自测：`/tmp/test_dataset`（合成 3000 行）→ 188 列、行数一致、`t_ms` 单调、0 丢弃，全部通过。
- 详见 `docs/SIM2REAL_DATA_FEEDBACK.md`（§7 是**真机测试清单 T0~T8**）。

## 其他事实
- 手柄：左摇杆上推=+vx 前进、右摇杆左推=+wz 左转(CCW)，vy 恒 0，量程 vx±1.0 m/s、wz±1.0 rad/s；B 键急停；Ctrl+C 急停。
- 站立前冲补偿 `CMD_BIAS_VX = −0.05`（Ex36/Ex37/Ex51/Ex53 + `MotionController::cmd_bias_vx`）：抵消策略 wheel action 正向偏置。⚠️ 其绝对值必须 < 训练 `turn_lin_threshold`(0.1)，否则退出纯 yaw gate 变扭腿。
- IMU（维特 HWT606，`/dev/ttyUSB0`，安装 `Z_DOWN_X`，115200）：`base_ang_vel`=gyro（机体系 rad/s）；`projected_gravity`=world2self(quat,[0,0,−1])。放平判读 pgr≈(0,0,−1)；右倾→pgr.y 变负；开机需机身水平（水平校准基准）。
- ⚠️ IMU 串口：`/dev/ttyUSB0` 曾被 Ubuntu **brltty 盲文服务抢占**（`PRODUCT==1a86/7523`）。已 `systemctl stop/disable/mask brltty` 与 `brltty-udev` 修复，勿再启用；另加 CH340 udev 规则（MODE=0666）。
- ⚠️ 代码教训：`rl::world2self`/`build_observation` 要求 quat 是**连续 float[4]**。写 IMU 读取时统一用数组 + `GetQuat(quat[0],...)`，勿用独立局部变量地址。
- 控制台日志分类：`include/common/log_control.h` 的 `LogCat`（SYSTEM/MOTOR/RL/IMU/WHEEL/CAN/DIAG）+ `LOG_SWITCH[]`；CSV 落盘开关 `include/common/motor_logger.h` 的 `LogFileSwitch`；S2R 遥测 `include/common/s2r_recorder.h`。
- Python 工具（`tool/`）：`export_policy.py`（ckpt → `policy_weights.h`/`policy_test_ref.h`）、`compare_sim2real.py`（sim `--record` CSV vs 真机 `log/rl_*.csv` 按 `wall_ms` 对齐）、`friction_id_offline.py`（Ex54 落盘 → fc/fv 回归，含 `--demo` 自测）、`verify_friction_ff.py`、`plot_rlrun.py`、`plot_motor_torque.py`、`plot_joint_torque.py`。
- 🗄️ 轮子乱转根因（2026-08-21 Example36 诊断锁定，已修复存档）：起立目标 `STAND_*` 与 RL 目标 `DEFAULT_POSE` 跳变 → 腿猛动带轮子 → 轮速冲高 → 制动饱和 → 污染策略观测 → 发散。修复：起立目标改用 `urdf_to_status(DEFAULT_POSE)` 消除跳变。

## 2026-09 变更要点
- 09-01 `463d230` 腿摩擦前馈 `fc·tanh(τ_pd/2)+fv·dq` + 电机扭矩命令限幅（当时 Hip/Thigh 110、Calf 200、Wheel 52）；`72200b0` Example54 吊装摩擦辨识；`a723bb3` 注释与文档对齐代码现状。
- 09-03 `ada57ff` 权重同步 iteration_2100。
- 09-05 `f10a2f5` 稳定版存档（标定与扭矩修正 / S2R 遥测记录）；`5c4b30a` 摩擦前馈高频化 500Hz（`SetLegTauFFOverride`，Ex56 A/B 腿跟踪误差 −46%）。
- 09-07 `0544f6c` Ex37 遥操作行走稳定（设为激活示例）；`0ee431f` gait_phase 交错→分组（对齐训练）。
- 09-11 `3039927` iteration_3500；09-12 `33b672e` iteration_4350；09-15 `b61216e` 回退到 iteration_2100；09-17 `0d0257c` iteration_5350；09-22 `98ef8ea` iteration_9754（当前）。
- 09-13 `bfadc76` 新增 Ex57 单腿零位对照；`6546688` 真机参数调整（腿重力前馈清零 / 轮软限位 30Nm / Ex37 量程 0.7）；`88a64e1` sim2sim `--wheel_gate`。

## 本次审查修复（2026-09-29）
1. `dogurdf_sim2sim_deploy/run_sim2sim.sh`：默认 checkpoint → `../weights/iteration_9754.pkl`（与真机同权重），支持 `SIM2SIM_CKPT` 覆盖。
2. `src/motor/ele_motor.cpp`：修复**参数回帧二次标定** —— 单寄存器回帧原先对 `position/velocity/torque` 三者整体再标定一次（符号被翻回 / offset 重复叠加），现只标定被更新的那个字段；`MOTOR_OR_torque` 原先**完全没标定**，现补上；日志 raw 值改为在标定前捕获。
3. `include/motor/motor_calibration.h`：按字段标定 helper `ApplyMotorCalibrationPos/Vel/Torque` 启用，`ApplyMotorCalibration` 改为其组合（配合第 2 条）。
4. `include/motor/ele_motor.h`：删除死成员 `state_mutex`（全仓无任何使用，实际加锁为 `MotorManager::m_motor_mutex`）。
5. `include/strategy/rl_controller.h` + `src/strategy/rl_controller.cpp`：删除死代码簇 `wheel_torque()`、`WHEEL_FF[4][2]`、`WHEEL_FF_ENABLE`、`WHEEL_SOFT_LIMIT_*`、`WHEEL_TORQUE_LIMIT`、`LEG_TORQUE_LIMIT`（历史数值见上文「轮控安全」节）。
6. `include/app/examples/ex_diag.h` + `src/app/main.cpp`：删除 `Example55_SingleLegLimitMeasure` 的声明与被注释调用（该示例从未实现，取消注释即链接失败）。
7. `src/app/main.cpp`：删除未使用的全局 `static RobotApp g_app;`（其析构会访问已销毁的 `MotorManager` 单例，属静态析构顺序 UB）及随之无用的 `#include "runtime/robot_app.h"`。
8. bug 修复：`can_device.cpp` 的 `DWORD` 用 `%d` 打印（改 `%lu`）；`log_control.h` 注释里的 `/*` 触发 `-Wcomment`；`ex_rl.cpp` 未使用 typedef、`ex_diag.cpp` 未使用变量 `POS_LIMIT`、`ex_basic.cpp` 未使用变量 `dt`；`MotionController` 三个插值函数在 `mm_==nullptr` 或 `total<=0` 时会崩 / 产生 NaN（加保护）；`Usb2CanTransport::recv` 用 `operator[]` 会给未打开通道凭空建条目（改 `find`）；`examples_common.cpp` 的 `poll_key` 读方向键时字节分次到达会丢键（改为带缓冲的状态解析）；`types.h` 的 `#endif` 注释与宏名不符。
9. `include/motion/robot_calibration.h`：修正与代码不符的注释（θ1 上限 15°、"仍压在 0 边界"等），以及引用已删除 Example55 的措辞。
10. 仍**保持原值不变、仅标注**的两处冲突（安全相关，⚠️ **待现场确认**）—— 见上文 `JOINT_IMPEDANCE[..][THIGH].tau_ff`（代码 `+5.0f` / 注释与 `6546688` 写 `−5`）与 `UPPER_LIMIT_THETA1_DEG`（代码 `15.0f` / 旧注释 `+30°`）。

## 注意
- `PLAN.md`（12 电机）与 `TODO.md` 滞后于代码（实际 16 电机）；`TODO.md` 列的 Bug 1~5 多已在代码中修复。
- 零位偏移唯一真值来源是 `MOTOR_CALIBRATION[].pos_offset`；`robot_calibration.h` 直接引用它，勿再手抄字面量。
- 训练权威 = `RL_Train/code`（非 `dogurdf_sim2sim_deploy`，后者是历史快照）。
- `RobotDog` 类仍未实现；`include/runtime` 的 `state_calc`/`monitor` 预留线程至今未实现；`RobotApp` 当前无调用者（示例各自建局部 `ThreadManager`）。
