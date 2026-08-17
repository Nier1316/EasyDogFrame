# 四足机器狗 运控 & RL 对接 —— 项目知识库

> 本文件是供**运控 / 强化学习对接 Agent** 检索的事实性参考（「查什么」）。
> 行为准则、工作方式、安全边界见系统提示词（「怎么做」）。
> 数值若有出入，一律以对应源码头文件为准（下文每处都标注了唯一真值来源）。

---

## 1. 工程与构建

- 根目录：`/home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify`
- 语言/构建：C++17 + CMake；产物 `bin/can_motor_app`
- 编译：`cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)`
- CANET SDK 位于 `lib/`：`CANET.h`、`ControlCAN.h`、`lib/linux_x64/{Debug,Release}/libCANET_TCP.{a,so}`
- 可选依赖：SDL2（Xbox 手柄示例 Example21）
- 权威事实：`memory/FACT.md`；框架总览：`FRAMEWORK_GUIDE.md`

---

## 2. 硬件拓扑

| 项 | 值 |
|---|---|
| CAN 路数 | 4（CAN0~3） |
| 通信 | CANET TCP，IP `192.168.0.178`，端口 `4001`~`4004` |
| 电机总数 | **16** = 4 CAN × 4 电机/路 |
| motor_id 语义 | `1=髋(hip)`、`2=大腿(thigh)`、`3=小腿(calf)`、`4=轮(wheel)` |
| 帧 ID | 发送 `tx_id = motor_id`(1~4)；接收 `rx_id = 50 + motor_id`(51~54) |
| 控制模式 | `IMPEDANCE=0`（阻抗）、`SPEED=1`（速度）、`POSITION=2`（位置） |

> 轮电机固件自带速度/位置环不可靠：**轮子走阻抗模式的前馈扭矩通道**（`kp=kd=0`，上位机闭环）。参见 `include/math/wheel_position_loop.h`。

---

## 3. 模块与 API 速查

### 3.1 电机管理 `include/motor_manager.h`（单例）

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
- `Send*` 只更新目标字段，真正发帧由 `SendThreadFunc`（1ms）对 enabled 电机统一驱动。
- 收发线程注册到外部 `ThreadManager`：`motor_receive` / `motor_send`，`LOOP, 1ms, 优先级 80`。

### 3.2 单电机编解码 `include/motor_drive/ele_motor.h`

- 结构体字段：`device_idx`、`motor_id`、`current_speed/current_torque/current_position/current_temp`、`target_*`、`control_mode`、`hw_control_mode`、`mode_settle_ticks`、`kp/kd/ki/kvp`、`error_code`、`enabled`。
- 自由函数：
  - `float2bag(motor, param, RW, type)` — 参数读写帧（`RW=0`读 / `1`写，`type` 见 `ele_motor_def.h` 的 `MOTOR_OR_*`/`MOTOR_WR_*`）。
  - `set_motor_para_bt(motor, p1..p5, model)` — 控制帧编码下发（三种 mode 布局不同）。
  - `unpack_frame(motor, data, dlc)` — 直接解包接收帧。
  - `uint_to_float` / `float_to_uint` — 协议量程编解码。

### 3.3 CAN 抽象 `include/bsp/bsp_can.h` + `include/can_device.h`

```cpp
BspCan::GetInstance();
bool InitDevice(device_idx, const CanDeviceConfig&);
bool StartDevice(device_idx); bool StopDevice(device_idx); bool CloseDevice(device_idx);
bool Can_Tx(device_idx, can_id, data, dlc=8);
bool SendFrame(device_idx, const BspCanFrame&);
bool ReceiveFrames(device_idx, std::vector<BspCanFrame>&, timeout_ms=100);
void ShutdownAll();
```

- `CanDeviceConfig`（`data_types.h`）：`device_idx`、`port`、`server_ip`、`work_mode`（`TCP_CLIENT=0`/`TCP_SERVER=1`）。
- `BspCanFrame`：`id`、`dlc`、`data[8]`、`is_extended`。

### 3.4 线程管理 `include/thread/thread_manager.h`

```cpp
void register_thread(name, func, ThreadMode::LOOP, interval_ms, priority);
void start_thread(name); void stop_thread(name);
SharedData& get_shared_data();          // 跨线程共享数据（std::any）
```

- `ThreadMode`：`ONCE` / `LOOP`；`priority` 1~99 = SCHED_FIFO（需 root）。

### 3.5 顶层 `include/RoboTasks/robot_app.h`

- `RobotApp::init()/start()/stop()`，统一持有 `ThreadManager`，按优先级启停线程。

### 3.6 日志 `include/motor_logger.h`

- `Init()` 创建 `log/` 下 CSV：`send_*.csv`、`recv_*.csv`、`sendcan_*.csv`。
- `LogSend`（目标值→逆标定值）、`LogSendCan`（上线 8 字节）、`LogRecv`（原始→标定）。

### 3.7 仿真同步 `include/SimSync.h`

```cpp
SimSync sim("127.0.0.1", 12345);   // MATLAB quadruped_realtime 是 TCP 服务器
sim.send_deg(float[12]);           // 12 关节角，单位「度」
sim.send_rad(float[12]);           // 12 关节角，单位「弧度」（内部转度）
bool connected();
```

- 协议：48 字节 = 12 × float32，顺序 `[FLθ1,FLθ2,FLθ3, FR…, RL…, RR…]`，单位**度**。
- 详见 `tool/API_SIMULATION.md`。

### 3.8 手柄 `include/xbox_controller.h`

- `XboxController::Initialize/Poll/GetState`；`XboxState` 含归一化摇杆/扳机（已去死区）与按键。

### 3.9 轮位置环 `include/math/wheel_position_loop.h`

```cpp
WheelPositionLoop w; w.configure(kp, kd, rate, max_torque, full_span=25.0f);
float tau = w.update(feedback_pos, feedback_vel, stick_norm, dt);  // 返回应下发的前馈扭矩
```

- 默认 `kp=4, kd=0.2, rate=3 rad/s, max_torque=3 Nm`。
- 输出经 `SendImpedance(cp, 4, 0,0,0,0, tau)` 下发（`kp=kd=0` 退化纯扭矩执行器）。

---

## 4. 标定与整机参数

### 4.1 电机标定（唯一真值来源：`include/motor_calibration.h`）

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
- `JOINT_IMPEDANCE[can][motor_id-1]`（仅关节 1~3）= `{kp, kd, tau_ff}`，当前全 `200/20/0`；取用 `GetJointImpedance(can,id)`。

### 4.2 整机参数（唯一真值来源：`include/robot_calibration.h`）

| 参数 | 值 |
|---|---|
| 连杆 | `LEG_L1=0.12`、`LEG_L2=0.35`、`LEG_L3=0.35`（m） |
| 机身 | `BODY_LENGTH=0.30`、`BODY_WIDTH=0.12`（m） |
| 零位偏移 | 髋 `0.611`、大腿 `0.441`、小腿 `0.211`（rad）——直接引用 `MOTOR_CALIBRATION` |
| 关节限位（度） | θ₁ `[-60, 0]`、θ₂ `[-70, 90]`、θ₃ `[60, 180]` |
| 站立姿态 | `STAND_HIP_DEG=0`、`STAND_THIGH_DEG=-60`、`STAND_CALF_DEG=60` |
| 控制周期 | `CONTROL_HZ=100` |
| 起立插值 | `STAND_INTERP_FRAMES=1000`（10s） |
| 轮参数 | `WHEEL_MAX_SPEED=3`、`WHEEL_KVP=1`、`WHEEL_KVI=0`、`WHEEL_MAX_TURN=2`、`WHEEL_SPEED_CAP=4` |
| 腿编号 | `LegIndex{FL=0,FR=1,RL=2,RR=3}`；`JointIndex{HIP=0,THIGH=1,CALF=2}` |

---

## 5. 协议编码量程（实测，勿臆测）

> 来源：`include/motor_drive/ele_motor_def.h` 的 `MOTOR_LIMITS[]`，2026-08-07 用 `Example24_ReadMotorParams` 从固件寄存器实测回读。

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

### 6.1 坐标系与角度约定（`include/leg_kinematics.h` + `robot_calibration.h`）

- 身体系：`X+前 / Y+左 / Z+上`；髋系：`X+后 / Y+外翻 / Z+上`。
- 关节正方向：θ₁ 外翻为正、θ₂ 后摆为正、θ₃ 后弯为正。
- `物理角 = ZERO_OFFSET + 指令角`；指令角 ∈ `[LOWER_LIMIT, UPPER_LIMIT]`。
- 与 MATLAB `leg_kinematics.m` 完全一致。

### 6.2 运动学接口（`include/leg_kinematics.h`，纯头文件）

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
3. **模式切换有生效窗口**：`hw_control_mode` 同步 + `mode_settle_ticks`≈20ms。
4. **标定对称**：接收 `ApplyMotorCalibration`，下发 `ApplyMotorCalibrationInverse`；扭矩随 `pos_scale` 一起翻转，否则前馈正反馈发散。
5. **轮子走阻抗前馈扭矩**，不用固件速度/位置环。
6. 站立姿态 θ₁/θ₃ 压在限位边界（已知待改项），新姿态优先把角挪进区间内。
7. 例程是活文档：`src/example.cpp` 的 Example17~24 编码了 IK、读站立、物理零位、手柄、轮式、只读诊断等实战教训。

---

## 8. RL 对接要点

- **观察量（obs）**：12 关节角（标定后 rad）+ 12 速度 + 12 扭矩 + 4 轮速度；可选 `leg_fk_all` 的 4 足端位置、关节角 sin/cos 编码。当前无 IMU/足底触地传感器。
- **动作量（act）**：优先输出关节位置增量，经 `SendImpedance(pos, 0, kp, kd, tau_ff)` 下发；`kp/kd` 取 `GetJointImpedance()`（默认 200/20），`tau_ff` 作额外力矩输出。
- **奖励候选**：前进速度追踪、能耗 `Σtorque²`、足端滑移/触地、机身姿态、限位/力矩超限、对称项、存活奖励。
- **sim-real**：仿真走 `SimSync`（12 关节，度），`leg_kinematics.h` 与 `leg_kinematics.m` 同约定；两侧共享 `robot_calibration.h` 的连杆/限位/`kp/kd`。
- **部署**：实现 `Policy` 抽象（obs→act），100Hz 线程 `obs→归一化→推理→反归一化/clamp→SendImpedance`；后端可选 ONNX Runtime / LibTorch。
- **安全兜底**：动作 clamp 到限位、力矩 clamp 到 `MOTOR_LIMITS`、急停键、跌倒检测、上电软启动（`WheelPositionLoop::reset` 对齐首帧反馈）。

---

## 9. 示例索引（`src/example.cpp`）

| 示例 | 功能 |
|---|---|
| Example17 | SimSync 仿真集成 |
| Example18 | 腿部 IK 控制 |
| Example19 | 读取当前姿态并缓慢移动到站立 |
| Example20 | 选择电机移动到物理零位 |
| Example21 | Xbox 手柄控制（当前 `main.cpp` 激活项） |
| Example22 | 起立 + 轮子阻抗模式测试 |
| Example23 | 单路 CAN 键盘控制（选路 + 站立 + ↑↓高度 + ←→轮子） |
| Example24 | 只读固件参数诊断（不使能，安全） |
