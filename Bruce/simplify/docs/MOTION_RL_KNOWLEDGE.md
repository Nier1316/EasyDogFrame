# 四足机器狗 运控 & RL 对接 —— 项目知识库

> 本文件是供**运控 / 强化学习对接 Agent** 检索的事实性参考（「查什么」）。
> 行为准则、工作方式、安全边界见系统提示词（「怎么做」）。
> 数值若有出入，一律以对应源码头文件为准（下文每处都标注了唯一真值来源）。
> 2026-08-30 整理：对齐分层重构 + USB2CAN + traj_v28 + 轮控 SPEED 迁移。

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
- `JOINT_IMPEDANCE[can][motor_id-1]`（仅关节 1~3）= `{kp, kd, tau_ff}`：**hip 300/10/tau_ff=-10、thigh 250/10/-5、calf 250/10/+12(前)/+20(后)**；RL 闭环只取此表的 `tau_ff`（kp/kd 由 `rl::LEG_KP/KD` 下发）。

### 4.2 整机参数（唯一真值来源：`include/motion/robot_calibration.h`）

| 参数 | 值 |
|---|---|
| 连杆 | `LEG_L1=0.1308`、`LEG_L2=0.34`、`LEG_L3=0.343`（m） |
| 机身 | `BODY_LENGTH=0.653`、`BODY_WIDTH=0.16`（m） |
| 零位偏移 | 髋 `0.611`、大腿 `0.441`、小腿 `0.211`（rad）——直接引用 `MOTOR_CALIBRATION` |
| 关节限位（度） | θ₁ `[-60, 0]`、θ₂ `[-70, 90]`、θ₃ `[60, 180]` |
| 站立姿态 | `STAND_HIP_DEG=0`、`STAND_THIGH_DEG=-60`、`STAND_CALF_DEG=60` |
| 控制周期 | `CONTROL_HZ=500` |
| 轮参数 | `WHEEL_MAX_SPEED`、`WHEEL_KVP=3.0`、`WHEEL_KVI=0.3`、`WHEEL_MAX_TURN`、`WHEEL_SPEED_CAP` |
| 腿编号 | `LegIndex{FL=0,FR=1,RL=2,RR=3}`；`JointIndex{HIP=0,THIGH=1,CALF=2}` |

---

## 5. 协议编码量程（实测，勿臆测）

> 来源：`include/motor/ele_motor_def.h` 的 `MOTOR_LIMITS[]`，2026-08-07 用 `Example24_ReadMotorParams` 从固件寄存器实测回读。

| 参数 | 关节 | 轮 |
|---|---|---|
| 位置 p（rad） | ±12.5 | ±12.5 |
| 速度 v（rad/s） | **±3**（不是 65） | **±48** |
| 扭矩 t（Nm） | ±150 | **±52** |
| kp | 0~500 | 0~500 |
| kd | **0~100**（不是 500） | 0~100 |
| ki | 0~500（待厂商确认） | 0~500 |

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
```

- `q_all[12]` 顺序 `[FLθ1,FLθ2,FLθ3, FR…, RL…, RR…]`（rad）。

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
- **动作（16 维）**：12 腿位置偏移 + 4 轮速目标；`ACTION_SCALE=0.25`、`WHEEL_VEL_SCALE=12.5`。
- **控制律**：腿 `τ = LEG_KP(q_t−q) + LEG_KD(0−qd)`（LEG_KP/KD=250/4，经 `urdf_to_status` 下发）；轮 `SendSpeed(vel, WHEEL_KVP, WHEEL_KVI)`。
- **关节顺序**：policy order（12 腿 + 4 轮）↔ CAN order 经 `POLICY_TO_MJX/MJX_TO_POLICY`。
- **零位转换**：`sim2real_conv` 的 `CONV_A/CONV_B`（真机 GetStatus ↔ URDF）。
- **sim-real**：仿真走 `SimSync`（12 关节，度）；训练权威 `RL_Train/code`（sim2sim 用 SIM_DT=0.002/DECIMATION=10/MOTOR_DECIMATION=1，500Hz PD 子环）。
- **部署入口**：`src/app/examples/ex_rl.cpp`；RL 决策 50Hz，电机 500Hz。
- **安全兜底**：动作 clamp 到限位、扭矩 clamp、跌倒检测、急停、轮速软限位 + 移动门控锁轮。

---

## 9. 示例索引（`src/app/examples/`，示例 17~53）

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

### ex_diag.cpp（24/26-29/33/34/39-50）
`Example24` 只读固件参数；`Example26` 键盘输入测试；`Example27/28` CANET 频率/批量探针；`Example29` 控制环频率；`Example33` IMU 检查；`Example34` 轮子方向；`Example39-46` USB2CAN 系列（探针/读状态/500Hz站立/速率/CAN顺序标定/手柄/移零位/单步）；`Example47` 整狗 chirp 参数辨识；`Example48` 轮子方向验证；`Example49` 轮速环测试；`Example50` 趴下角度记录。

### ex_rl.cpp（25/30-32/35-38/51-53）
`Example25` RL 循环；`Example30` 离线回归；`Example31` 零位对齐；`Example32` 默认姿态验证；`Example35` 轮摩擦前馈标定；`Example36` USB2CAN RL 站立循环；`Example37` 手柄遥操作；`Example38` 动作延迟测量；`Example51` 站立后趴下；`Example52` 固定 yaw；`Example53` 重力前馈测量。

> 当前 `main.cpp` 激活：**Example44_USB2CanXboxControl**。
