# 四足机器狗 运控 & RL 对接 —— 项目知识库

> 本文件是供**运控 / 强化学习对接 Agent** 检索的事实性参考（「查什么」）。
> 行为准则、工作方式、安全边界见系统提示词（「怎么做」）。
> 数值若有出入，一律以对应源码头文件为准（下文每处都标注了唯一真值来源）。
> 2026-08-30 整理：对齐分层重构 + USB2CAN + traj_v28 + 轮控 SPEED 迁移。
>
> ✅ **2026-09-29 更新**：权重 `weights/iteration_9754.pkl`（sim2sim 同步同一份）；`main.cpp` 激活 **Example37_RLTeleopControl**；
> 示例总数 40（编号 17~57，`Example55` 从未实现已移除）；`gait_phase` 观测为**分组**布局；真机扭矩命令限幅 120/120/200/52；
> 轮速软限位已失效，现由 `MotorManager` 的 `WHEEL_ESTOP_*` 承担。
>
> 📌 **相关专题**：`docs/CONTACT_PHASE_SIM2REAL.md` —— 触地/接触检测、相位检测、sim2real gap 补偿的开源框架调研
> 与本项目落地路线（含"本项目当前没有任何接触检测代码""无质量模型 ⇒ 绝对 GRF 不可用"等前提的说明）。
>
> 🔧 **2026-09-29 触地检测前提件已落地**：`leg_kinematics.h` 新增**解析雅可比**（`leg_jacobian`/`leg_jacobian_body`/
> `leg_cond_proxy`/`leg_foot_force_body`/`leg_foot_force_to_torque`，已与 `leg_fk` 有限差分逐元素校验）；
> 新增示例 **Example58 力矩通道校验**、**Example59 重力矩系数与质量-质心辨识**（见 `src/app/examples/ex_sysid.cpp`）。

---

## 1. 工程与构建

- 根目录：`/home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify`
- 语言/构建：C++17 + CMake；产物 `bin/can_motor_app`
- 编译：`cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)`
- 分层结构：`src/app/`（main + examples/ex_basic/ex_diag/ex_rl）、`src/runtime/`（thread_manager/robot_app/motor_io）、`src/strategy/`（rl_controller/sim2real_conv/imu_device）、`src/motion/`（motion_controller）、`src/motor/`（motor_manager/ele_motor）、`src/transport/`（canet_transport/can_device/usb2can_transport）
- 传输层：CANET TCP（`lib/CANET.h`、`lib/linux_x64/{Debug,Release}/libCANET_TCP.{a,so}`）+ **达妙 USB2CAN**（`lib/damiao_sdk/linux/x86_64/libdm_device.so`）
- 可选依赖：SDL2（手柄示例）；达妙链接需 conda 新版 libstdc++(GLIBCXX_3.4.32) + libusb(≥1.0.26)，CMakeLists 自动探测 `CONDA_LIB_DIR`（bruce/sysu 两机）
- 权威事实：`memory/FACT.md`；部署交接：`docs/SIM2REAL_DEPLOY.md`；训练侧：`/home/sysu/Desktop/Project/Bruce/RL_Train/code`

---

## 2. 硬件拓扑

| 项 | 值 |
|---|---|
| CAN 路数 | 4（CAN0~3） |
| 通信 | CANET TCP（IP 192.168.0.178，端口 4001~4004）+ 达妙 USB2CAN（主控，默认后端） |
| 电机总数 | **16** = 4 CAN × 4 电机/路 |
| motor_id 语义 | `1=髋(hip)`、`2=大腿(thigh)`、`3=小腿(calf)`、`4=轮(wheel)` |
| 帧 ID | 发送 `tx_id = motor_id`(1~4)；接收 `rx_id = 50 + motor_id`(51~54) |
| 控制模式 | `IMPEDANCE=0`（阻抗）、`SPEED=1`（速度）、`POSITION=2`（位置） |

> 🔴 **轮子控制（2026-08-29 SPEED 迁移）**：轮速走**固件 SPEED 速度环**（`SendSpeed(vel, kvp, ki)`，固件内部 1kHz 闭环），不再走阻抗前馈扭矩。阻抗模式忽略 `vel_des`。相关常量见 `include/strategy/rl_controller.h`（WHEEL_KVP=3.0/KVI=0.05/SOFT_KVP=0.1/CMD_ALPHA=0.2/MOVE_THR=0.1）。

---

## 3. 模块与 API 速查

### 3.1 电机管理 `include/motor/motor_manager.h`（单例）

```cpp
MotorManager::GetInstance();                       // 单例
bool Initialize(ThreadManager& thread_mgr);        // 初始化 4 路设备 + 注册收发线程
void Stop();                                       // 关闭设备（线程由外部 ThreadManager 停）
void SetControlMode(can_port, motor_id, mode);     // 使能前写固件控制模式
void ReadParam(can_port, motor_id, type);          // 读固件参数寄存器（异步，回帧打印 [PARAM]）
void EnableMotor(can_port, motor_id);              // 使能
void DisableMotor(can_port, motor_id);             // 失能
void SetZero(can_port, motor_id);                  // 归零
void ClearError(can_port, motor_id);               // 清错
void SendImpedance(can_port, motor_id, pos, vel, kp, kd, torque);   // 阻抗
void SendSpeed(can_port, motor_id, vel, kp, ki);                    // 速度
void SendPosition(can_port, motor_id, pos, kvp, kp, kd, kvi);       // 位置
MotorStatus GetStatus(can_port, motor_id) const;   // 读状态
```

- 索引约定：`can_port` ∈ 0~3，`motor_id` ∈ 1~4（内部数组下标 `motor_id-1`）。
- `Send*` 只更新目标字段，真正发帧由 `SendThreadFunc`（2ms）对 enabled 电机统一驱动。
- 收发线程注册到外部 `ThreadManager`：`motor_receive` / `motor_send`，`LOOP, 2ms, 优先级 80`（500Hz，见 `include/runtime/motor_io.h`）。
- 传输后端默认 **USB2CAN**（`motor_manager.cpp`），可用 `SetChannelTransport(can, &transport)` 覆盖。

### 3.2 单电机编解码 `include/motor/ele_motor.h`

- 结构体字段：`device_idx`、`motor_id`、`current_speed/current_torque/current_position/current_temp`、`target_*`、`control_mode`、`hw_control_mode`、`mode_settle_ticks`、`kp/kd/ki/kvp`、`error_code`、`enabled`。
- 自由函数：
  - `float2bag(motor, param, RW, type)` — 参数读写帧（`RW=0`读 / `1`写，`type` 见 `ele_motor_def.h` 的 `MOTOR_OR_*`/`MOTOR_WR_*`）。
  - `set_motor_para_bt(motor, p1..p5, model)` — 控制帧编码下发（三种 mode 布局不同）。
  - `unpack_frame(motor, data, dlc)` — 直接解包接收帧。
  - `uint_to_float` / `float_to_uint` — 协议量程编解码。

### 3.3 传输层抽象 `include/transport/`（BspCan 已删）

- `CanTransport` 抽象基类 + 两个实现：`CanetTransport`（TCP）、`Usb2CanTransport`（达妙 USB2CAN）。
- `MotorManager::SetChannelTransport(can_port, &transport)` 指定某路后端；默认 USB2CAN。
- USB2CAN 需 udev 0666 + 波特率 1M 匹配电机总线。

### 3.4 线程管理 `include/runtime/thread_manager.h`

```cpp
void register_thread(name, func, ThreadMode::LOOP, interval_ms, priority);
void start_thread(name); void stop_thread(name);
SharedData& get_shared_data();          // 跨线程共享数据（std::any）
```

- `ThreadMode`：`ONCE` / `LOOP`；`priority` 1~99 = SCHED_FIFO（需 root）。

### 3.5 顶层 `include/runtime/robot_app.h`

- `RobotApp::init()/start()/stop()`，统一持有 `ThreadManager`，按优先级启停线程。

### 3.6 日志 `include/common/motor_logger.h`

- `Init()` 创建 `log/` 下 CSV：`send_*.csv`、`recv_*.csv`、`sendcan_*.csv`。
- `LogSend`（目标值→逆标定值）、`LogSendCan`（上线 8 字节）、`LogRecv`（原始→标定）。

### 3.7 仿真同步 `include/motion/SimSync.h`

```cpp
SimSync sim("127.0.0.1", 12345);   // MATLAB quadruped_realtime 是 TCP 服务器
sim.send_deg(float[12]);           // 12 关节角，单位「度」
sim.send_rad(float[12]);           // 12 关节角，单位「弧度」（内部转度）
bool connected();
```

- 协议：48 字节 = 12 × float32，顺序 `[FLθ1,FLθ2,FLθ3, FR…, RL…, RR…]`，单位**度**。
- 详见 `tool/API_SIMULATION.md`。

### 3.8 手柄 `include/strategy/xbox_controller.h`

- `XboxController::Initialize/Poll/GetState`；`XboxState` 含归一化摇杆/扳机（已去死区）与按键。

### 3.9 轮位置环 `include/motion/wheel_position_loop.h`（⚠ 已废弃）

- 曾用于「阻抗前馈扭矩控轮」，**2026-08-29 SPEED 迁移后废弃**；轮速走固件 SPEED 环。仅 ex_basic 老示例引用。

---

## 4. 标定与整机参数

### 4.1 电机标定（唯一真值来源：`include/motor/motor_calibration.h`）

- 常量：`CAN_PORTS=4`、`MOTORS_PER_CAN=4`。
- 矩阵 `MOTOR_CALIBRATION[can_port][motor_id-1]` = `{pos_scale, vel_scale, pos_offset}`：

| CAN | 髋(1) | 大腿(2) | 小腿(3) | 轮(4) |
|---|---|---|---|---|
| CAN0 FL | -1,1,0.611 | 1,1,0.441 | -1,1,0.211 | -1,-1,0 |
| CAN1 FR | 1,1,0.611 | -1,1,0.441 | 1,1,0.211 | 1,1,0 |
| CAN2 RL | 1,1,0.611 | 1,1,0.441 | -1,1,0.211 | -1,-1,0 |
| CAN3 RR | -1,1,0.611 | -1,1,0.441 | 1,1,0.211 | 1,1,0 |

- `ApplyMotorCalibration(can,id, pos, vel, torque)` — 接收方向（原始→标定）。
- `ApplyMotorCalibrationInverse(can,id, pos, vel, torque*)` — 发送方向（统一坐标→原始）。
- `JOINT_IMPEDANCE[can][motor_id-1]`（仅关节 1~3）= `{kp, kd, tau_ff}`：hip 300/10/tau_ff=-10、thigh 250/10/**tau_ff=+5（代码值；行内注释与提交信息曾写 -5，属未解冲突，待现场确认）**、calf 250/10/+12(前)/+20(后)；RL 闭环只取此表的 `tau_ff`（kp/kd 由 `rl::LEG_KP/KD` 下发）。

### 4.2 整机参数（唯一真值来源：`include/motion/robot_calibration.h`）

| 参数 | 值 |
|---|---|
| 连杆 | `LEG_L1=0.1308`、`LEG_L2=0.34`、`LEG_L3=0.343`（m） |
| 机身 | `BODY_LENGTH=0.653`、`BODY_WIDTH=0.16`（m） |
| 零位偏移 | 髋 `0.611`、大腿 `0.441`、小腿 `0.211`（rad）——直接引用 `MOTOR_CALIBRATION` |
| 关节限位（度） | θ₁ `[-60, +15]`（代码 `UPPER_LIMIT_THETA1_DEG=15.0f`；`robot_calibration.h` 注释曾写"放宽到 +30°"，以代码 **+15** 为准）、θ₂ `[-70, 90]`、θ₃ `[60, 180]` |
| 站立姿态 | `STAND_HIP_DEG=0`、`STAND_THIGH_DEG=-60`、`STAND_CALF_DEG=60` |
| 趴下姿态 | `LIE_DOWN_HIP_DEG=+11.4`、`LIE_DOWN_THIGH_DEG=-55.2`、`LIE_DOWN_CALF_DEG=+12.6` |
| 控制周期 | `CONTROL_HZ=500` |
| 轮参数（`robot_calibration.h`） | `WHEEL_MAX_SPEED`、`WHEEL_KVP=3.0`、`WHEEL_KVI=0.3`、`WHEEL_MAX_TURN`、`WHEEL_SPEED_CAP` |
| 轮参数（RL 实际） | `include/strategy/rl_controller.h`：`WHEEL_KVP=3.0`、**`WHEEL_KVI=0.05`**（⚠ 整机表里的 0.3 是键盘控轮历史值，RL 上会振荡疯转，勿混用）、`WHEEL_SOFT_KVP=0.1`、`WHEEL_CMD_ALPHA=0.2`、`WHEEL_CMD_MOVE_THR=0.1` |
| 腿编号 | `LegIndex{FL=0,FR=1,RL=2,RR=3}`；`JointIndex{HIP=0,THIGH=1,CALF=2}` |

---

## 5. 协议编码量程（实测，勿臆测）

> 来源：`include/motor/ele_motor_def.h` 的 `MOTOR_LIMITS[]`，2026-08-07 用 `Example24_ReadMotorParams` 从固件寄存器实测回读。

| 参数 | 关节 (hip/thigh) | 关节 (calf) | 轮 |
|---|---|---|---|
| 位置 p（rad） | ±12.5 | ±12.5 | ±12.5 |
| 速度 v（rad/s） | **±3**（不是 65） | **±3** | **±48** |
| 扭矩 t（Nm） | **±120** | **±200** | **±52** |
| kp | 0~500 | 0~500 | 0~500 |
| kd | **0~100**（不是 500） | 0~100 | 0~100 |
| ki | 0~500（待厂商确认） | 0~500 | 0~500 |

> 扭矩量程 2026-08-30 由固件层改、2026-09-04 hip/thigh 110→120；`ele_motor_def.h` 的 `MOTOR_LIMITS` 量程
> 与命令限幅 `TORQUE_CMD_LIMIT`（120/120/200/52）现为同一组值。⚠ 旧文写的「关节 ±150」已作废。

- 编解码两侧量程不一致会导致收发数值全错（曾出现速度差 21.7 倍）。
- 控制命令宏：`MOTOR_STRAT=0xFC`、`MOTOR_STOP=0xFD`、`MOTOR_ANGLE_ZERO=0xFE`、`MOTOR_CLEAR_ERROR=0xF4`。

---

## 6. 坐标与运动学

### 6.1 坐标系与角度约定（`include/motion/leg_kinematics.h` + `include/motion/robot_calibration.h`）

- 身体系：`X+前 / Y+左 / Z+上`；髋系：`X+后 / Y+外翻 / Z+上`。
- 关节正方向：θ₁ 外翻为正、θ₂ 后摆为正、θ₃ 后弯为正。
- `物理角 = ZERO_OFFSET + 指令角`；指令角 ∈ `[LOWER_LIMIT, UPPER_LIMIT]`。
- 与 MATLAB `leg_kinematics.m` 完全一致。

### 6.2 运动学接口（`include/motion/leg_kinematics.h`，纯头文件）

```cpp
void leg_fk(const float q_cmd[3], L1,L2,L3, off1,off2,off3, float p[3]); // 单腿正解
void leg_ik(const float p[3], L1,L2,L3, off1,off2,off3, float q_cmd[3]);  // 单腿逆解
void hip_rotation_matrix(LegIndex leg, float R[3][3]);
void leg_fk_all(const float q_all[12], float foot_body[4][3]);             // 12 关节→4 足端
// —— 2026-09-29 新增：解析雅可比与接触力映射（触地检测/GRF 估计的基础）——
void leg_jacobian(q_cmd[3], L1,L2,L3, off1,off2,off3, float J[3][3]);      // 髋系：q̇ → 轮心速度
void leg_jacobian_body(LegIndex leg, q_cmd[3], float Jb[3][3]);            // 体系：= R·J_hip
float leg_cond_proxy(const float J[3][3]);                                 // 条件数代理（1=好，∞=奇异）
bool leg_foot_force_body(LegIndex leg, q_cmd[3], tau[3], f[3], float* cond=nullptr); // f=(Jᵀ)⁻¹τ
void leg_foot_force_to_torque(LegIndex leg, q_cmd[3], f[3], tau[3]);       // τ=Jᵀf
```

- `q_all[12]` 顺序 `[FLθ1,FLθ2,FLθ3, FR…, RL…, RR…]`（rad）。
- **角度坐标**：`leg_fk`/`leg_jacobian` 的输入 = `GetStatus().position` = `SendImpedance` 的位置参数，**同一坐标**（Ex18 把 `leg_ik` 输出直接下发给 `SendImpedance`）。无需再减 `THETA*_OFFSET`——那是 `leg_fk` 内部的几何约定。
- **雅可比末端点**是"轮心"（L3 末端），不是轮底接触点；要轮底点沿轮半径平移即可。
- ⚠️ `leg_foot_force_body` 要求传入的 `tau` **已扣掉重力项与摩擦项**；且腿接近伸直时条件数爆炸会被拒绝（`cond > 50`）。实测条件数：膝伸直 1.8e7、距伸直 0.02 rad → 103、DEFAULT_POSE → 15、STAND → 1.8。**力误差 ≈ 力矩误差 × cond**。

---

## 7. 关键约定与陷阱（务必内化）

1. **零位/标定唯一真值来源**是 `MOTOR_CALIBRATION[][].pos_offset`；`robot_calibration.h` 直接引用，**禁止手抄字面量**。
2. **使能前先 `SetControlMode`**，否则电机在固件默认模式下被使能会瞬时误动（轮电机实测会转）。
3. **模式切换有生效窗口**：`hw_control_mode` 同步 + `mode_settle_ticks`≈20 周期（发送 2ms → 约 40ms）。
4. **标定对称**：接收 `ApplyMotorCalibration`，下发 `ApplyMotorCalibrationInverse`；扭矩随 `pos_scale` 一起翻转，否则前馈正反馈发散。
5. **轮子走固件 SPEED 速度环**（SendSpeed），不用阻抗前馈扭矩通道（阻抗忽略 vel_des）。
6. **RL 闭环 kp/kd 来自 `rl::LEG_KP/KD`（rl_controller.h）**；`JOINT_IMPEDANCE` 只供非 RL 示例 + RL 的 tau_ff。
7. 例程是活文档：`src/app/examples/ex_{basic,diag,rl}.cpp` 编码了 IK、读站立、物理零位、手柄、轮式、USB2CAN、RL 部署等实战教训。

---

## 8. RL 对接要点（dogurdf 轮足，traj_v28）

- **观测（64 维）**：base_lin_vel(3)=0 | base_ang_vel(3) | projected_gravity(3) | joint_pos_rel(12) | joint_vel(16) | last_action(16) | command(3) | gait_phase(8)。IMU（HWT606）提供 gyro + 四元数算 ang_vel/projected_gravity。
  - `gait_phase` 是**分组**布局：`obs[56..59]=sin(2πφ)×4脚`、`obs[60..63]=cos(2πφ)×4脚`（**不是** `[sin,cos]` 交错）。φ = `fmod(step·0.02/0.6 + GAIT_OFFSET[foot], 1)`，offset=`{0,0.5,0.5,0}`。2026-09-19 提交 `0ee431f` 从交错改为分组以对齐训练（`src/strategy/rl_controller.cpp` 的 `build_observation`）。
- **动作（16 维）**：12 腿位置偏移 + 4 轮速目标；`ACTION_SCALE=0.25`、`WHEEL_VEL_SCALE=12.5`。
- **控制律**：腿 `τ = LEG_KP(q_t−q) + LEG_KD(0−qd)`（LEG_KP/KD=250/4，经 `urdf_to_status` 下发）；轮 `SendSpeed(vel, WHEEL_KVP, WHEEL_KVI)`。
- **关节顺序**：policy order（12 腿 + 4 轮）↔ CAN order 经 `POLICY_TO_CAN`（policy→can）/ `CAN_TO_POLICY`（can→policy）。
  ⚠ 2026-09-30 更名：旧名 `POLICY_TO_MJX`/`MJX_TO_POLICY` 的**字面含义与实际语义正好相反**（历史上的坑），现已按语义重命名并保留旧名作为引用别名（`&` 绑定）。
- **零位转换**：`sim2real_conv` 的 `CONV_A/CONV_B`（真机 GetStatus ↔ URDF）。
- **sim-real**：真机↔MATLAB 仿真走 `SimSync`（12 关节，度，轮不在环内）；训练权威 `RL_Train/code`（其 sim2sim 用 SIM_DT=0.002/DECIMATION=10/MOTOR_DECIMATION=1，500Hz PD 子环）。
  **本仓库自带**的 `dogurdf_sim2sim_deploy/src/sim2sim.py` 是另一套原生 MuJoCo 验证器：`SIM_DT=0.005 / DECIMATION=4`（同样 50Hz 控制），默认已与真机同权重 `weights/iteration_9754.pkl`。
- **部署入口**：`src/app/examples/ex_rl.cpp`（当前激活 **Example37_RLTeleopControl**，见 `src/app/main.cpp`）；RL 决策 50Hz，电机 500Hz。
- **安全兜底**：动作 clamp 到限位、`TORQUE_CMD_LIMIT` 扭矩 clamp（120/120/200/52）、急停、`WHEEL_CMD_MOVE_THR` 移动门控锁轮。
  ⚠ 旧文写的"轮速软限位"（`WHEEL_SOFT_LIMIT_*` / `rl::wheel_torque()`）已失效、无调用者；轮速保护现由 `MotorManager` 的 `WHEEL_ESTOP_*`（15 rad/s 自动急停）承担。
- **action latency**：本仓库不记录训练侧是否含时延——**以 `RL_Train/code` 的配置为唯一真源**；本仓库实测结论是真机纯传输延迟 ≈24ms（18~30ms），建议 `action_delay_steps=1`。

---

## 9. 示例索引（`src/app/examples/`，示例 17~61，共 44 个）

> 分发机制：改 `src/app/main.cpp` 里各示例调用的注释 + 重新编译，**无命令行参数、无注册表**。
> ⚠️ `Example55_SingleLegLimitMeasure` 在本仓库**从未实现**（声明与被注释调用已在本次清理中删除），不要再当作可用示例。
> ⚠️ 安全现状（2026-09-29 脚本复核）：42 个示例中 **29 个会使能电机**（脚本按"函数体内直接调 `EnableMotor`/`PreEnableZeroTorque`"统计为 27 个；Ex58/59 经公共 helper `init_zero_torque()` 使能，故实际 29 个），
> 其中 **13 个装了 `SIGINT` 急停**（Ex25/34/**35**/36/37/38/51/52/53/54/56/**58/59**）；
> 其余 **16 个会发使能帧但无 `SIGINT` 保护**（Ex18/19/20/21/22/23/29/32/41/44/45/46/47/48/49/57）——跑这些示例务必人在现场、可随时断电。
> 🆕 **系统辨识/通道校验（2026-09-29 新增，触地检测前提件）**：见 §9.4 与 `docs/CONTACT_PHASE_SIM2REAL.md` §7.0。
> 🆕 **真机激励探针 Example60 + 统一 500 Hz 数据集录制（`common/s2r_dataset.h`，188 列指令+反馈同帧）**：见 §9.5 与 `docs/SIM2REAL_DATA_FEEDBACK.md` §7（真机测试清单）。

### ex_basic.cpp（17~23）
| 示例 | 功能 |
|---|---|
| Example17 | SimSync 仿真集成 |
| Example18 | 腿部 IK 控制 |
| Example19 | 读取当前姿态并缓慢移动到站立 |
| Example20 | 选择电机移动到物理零位 |
| Example21 | Xbox 手柄控制 |
| Example22 | 起立 + 轮子测试 |
| Example23 | 单路 CAN 键盘控制 |

### ex_diag.cpp（24/26-29/33/34/39-50/54/57）
`Example24` 只读固件参数；`Example26` 键盘输入测试；`Example27/28` CANET 频率/批量探针（CANET 已弃用，仅这两个示例仍直接用）；`Example29` 控制环频率；`Example33` IMU 检查；`Example34` 轮子方向；
`Example39-43` USB2CAN 系列（探针/读状态/500Hz站立/速率/CAN顺序标定）；`Example44` USB2CAN 手柄控制；`Example45` 移到零位；`Example46` 单电机阶跃；`Example47` 整狗 chirp 参数辨识；`Example48` 轮子方向验证；`Example49` 轮 SPEED 环 kvp 扫描；`Example50` 趴下角度记录（标定 LIE_DOWN_*）；`Example54` 吊装摩擦辨识；`Example57` 单腿零位对照（验证 `CONV_A/B`）。

### ex_rl.cpp（25/30-32/35-38/51-53/56）
`Example25` 完整 RL 循环 + 手柄；`Example30` 离线链路回归（**不碰 CAN**）；`Example31` 零位对齐；`Example32` 默认姿态验证；`Example35` 轮摩擦前馈标定；`Example36` RL 站立循环；`Example37` RL 遥操作（手柄前进/后退/转向）；`Example38` 动作延迟辨识；`Example51` 站立后趴下；`Example52` 固定 yaw；`Example53` 重力前馈测量；`Example56` 固定 yaw 遥测落盘。

### 9.4 ex_sysid.cpp（58~59）—— 触地检测的前提件，2026-09-29 新增

| 示例 | 功能 | 前提/落盘 |
|---|---|---|
| **Example58** `TorqueChannelCheck` | 16 路**力矩通道校验**：阻抗模式 `kp=kd=0 ⇒ τ=τ_ff`，逐电机施加 `0→+T1→+T2→0→−T1→−T2→0`，检查零偏（<0.5 N·m）/增益（误差 ≤ max(0.5, 25%)）/符号/线性度。轮子默认跳过（会转起来） | 狗必须**吊起**；人工轻扶被测肢体；→ `log/sysid/torque_check_*.csv` |
| **Example59** `GravityMassIdentify` | **重力矩系数 `G_j = m·g·d`** 与质量-质心：单关节在参考姿态附近双向慢扫 7 点（共 14 点），准静态采样 (θ, τ)，对 `[sinθ, cosθ, 1]` 做 3 参数最小二乘 → `G_j = √(a²+b²)`（**不需要扫到力臂最大处**）；配合台秤称重反推 `d`，配合 Ex47 的 `J` 得 `I_c = J − m·d²`；打印可直接填入 `LINK_DYNAMICS`/`BODY_MASS` 的建议值 | 狗必须**吊起 + 机身水平**（可选 IMU 检查倾角）；→ `log/sysid/gravity_summary_*.csv` |

### 9.6 ex_rl.cpp（61）—— 站立/原地转向 专精策略遥操作，2026-10-02 新增

`Example61_RLStandTurnTeleop`：针对 **standstep_s4 / iteration_10000** 策略（运行时会强制切到该变体）的手柄遥操作。
该策略训练时**只有两类命令**：`[0,0,0]`（站立）与 `[0,0,wz]`（原地迈步转向），vx 恒 0、不支持前进。
- 手柄：右摇杆水平=原地转向（死区内 = 精确 `[0,0,0]`）、A=强制站立、B/START=优雅趴下、q=退出（`Ctrl+C` 硬急停）。
- **手柄 Y=标准对比序列**（站立 5s → 左转 8s → 站立 3s → 右转 8s → 站立 3s，共 27s；上升沿触发，推摇杆即中止回手动）：给真机与仿真一条
  **完全相同的命令串**，命令写进数据集 `cmd_wz` 列，离线按命令分段即可严格对齐 —— 这是做 sim/real gap
  对比时排除"人手不一致"的关键。
- 默认开 500 Hz 统一数据集录制；每秒打印 `cmd` vs `gyro_z`（现场就能看出跟不跟得上）。
- 两套权重同时编译进程序，运行时选择见 `include/strategy/policy_set.h`；`policy_variant.h` 的 `POLICY_VARIANT` 只是默认值。

### 9.5 ex_probe.cpp（60）—— 真机激励探针，2026-09-29 新增

`Example60_SysIdProbe`：悬空下 5 种激励（1 力矩脉冲 / 2 位置阶跃 / 3 单关节位置 chirp / 4 轮速阶跃+扫频 / 5 全部 12 关节 chirp），
全程用 `S2RDataset` 录 500 Hz 数据集（指令与反馈同一行、共时间基准、含 IMU/Vbus/温度）。
产物 `log/dataset_*.csv` → `tool/dataset_health.py` / `delay_fit.py` / `wheel_servo_fit.py`。
与 `Example38` 的分工：Ex38 用相位法**在线**估延迟（只打印）；Ex60 把原始数据落盘，延迟/伺服模型**离线**拟合，可复算可回放。

> **为什么需要它们**：`f = (Jᵀ)⁻¹·τ` 的力误差 ≈ 力矩误差 × 条件数。实测腿的条件数在 DEFAULT_POSE ≈ 15、
> 膝伸直 → 1.8e7；而本项目腿摩擦 1.6~6.2 N·m、重力矩 8~20 N·m 与信号同量级。
> **不先校验 τ 通道、不先标定重力项，接触力估计就是噪声。**
> 分工：`J/B/f_c` 由 Example47 辨识、腿摩擦由 Example54 辨识，`58/59` 不重复。

> 当前 `main.cpp` 激活：**Example37_RLTeleopControl**。
