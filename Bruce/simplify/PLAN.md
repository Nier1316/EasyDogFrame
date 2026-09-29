# 四足机器狗12关节控制系统 — 实现计划

> ⚠️ **本文档为历史实现计划（横幅更新于 2026-09-29），已过时，正文保留作历史存档，请勿据此施工。**
>
> **与现状的差异（逐条核对代码后确认）**：
> 1. **`RobotDog` 四足语义层从未实现**：全仓不存在 `include/robot_dog.h` / `src/robot_dog.cpp`，
>    也没有 `LegId` / `JointId` 枚举。四足语义现由 `motion` 层（`MotionController`、腿部运动学、
>    整机标定）与各示例直接调用 `MotorManager` 承担。
> 2. **电机数 12 → 16**：计划目标是 4 路 × 3 关节 = 12 电机；实际已演进为 **16 电机（4 路 × 4）**，
>    每路多一个**轮电机**（`motor_id = 4`），`CAN_PORTS = 4`、`MOTORS_PER_CAN = 4`。
> 3. **示例编号 2/3/4 已被取代**：计划里的 `Example2_EnableAllMotors` / `Example3_ZeroAllJoints` /
>    `Example4_StandingPosture` 与 `main.cpp` 的 `case 2/3/4` 均**不存在**；现行为
>    **`Example17~Example57`（共 40 个）**示例体系，切换方式是在 `src/app/main.cpp` 改注释 + 重新编译，
>    **无命令行参数**（`./bin/can_motor_app 2` 这类用法不支持）。当前启用 `Example37_RLTeleopControl`。
> 4. **接口与节拍变化**：收发线程现由 `MotorManager::Initialize()` 经 `RegisterMotorIoThreads()`
>    注册到外部 `ThreadManager`，`motor_receive` / `motor_send` 各 **2ms（500Hz）**、优先级 80
>    （计划中的"每 1ms 轮询"已过时）；传输层改为 `CanTransport` 抽象，默认**达妙 USB2CAN**
>    （计划中的 CANET TCP 已弃用，仅 Example27/28 直连）。
> 5. **文件路径全部迁移**：`include/{motor_manager,motor_calibration,data_types}.h`、
>    `include/{bsp,motor_drive,thread}/`、`src/{main,example}.cpp`、`src/base/` 均已移动或删除，
>    见 `FRAMEWORK_GUIDE.md` 文首的「已失效名称 → 当前实现」对照表。
>
> **现状以 `docs/SIM2REAL_DEPLOY.md`、`memory/FACT.md` 为准**（`docs/MOTION_RL_KNOWLEDGE.md`、
> `FRAMEWORK_GUIDE.md` 可作补充）。

## 目标

在 `simplify` 目录现有 CAN 通信基础设施上，新增 `MotorManager`（12电机批量管理）和 `RobotDog`（四足语义接口），实现关节层控制。

---

## 硬件拓扑

```
CAN0 → 左前腿 FL: motor_id=1(髋), 2(大腿), 3(小腿)
CAN1 → 右前腿 FR: motor_id=1(髋), 2(大腿), 3(小腿)
CAN2 → 左后腿 RL: motor_id=1(髋), 2(大腿), 3(小腿)
CAN3 → 右后腿 RR: motor_id=1(髋), 2(大腿), 3(小腿)

tx_id = motor_id (1/2/3)        上位机 → 电机
rx_id = 50 + motor_id (51/52/53) 电机 → 上位机
TCP: 192.168.0.178, 端口 4001~4004
```

---

## 现有代码基础

| 文件 | 作用 |
|------|------|
| `include/motor_drive/ele_motor.h` | 单电机数据结构 + 控制/解包函数声明 |
| `src/motor_drive/ele_motor.cpp` | `set_motor_para_bt()` 编码发送、`unpack_frame()` 解包 |
| `include/bsp/bsp_can.h` | CAN 收发抽象层（单例） |
| `src/bsp/bsp_can.cpp` | `Can_Tx()` / `ReceiveFrames()` 实现 |
| `include/data_types.h` | `MotorStatus` / `CanDeviceConfig` |
| `include/motor_drive/ele_motor_def.h` | 参数范围宏、控制命令宏 |

> 🗄️ 上表是**计划编写时**的文件布局，全部已迁移（历史路径 → 当前路径）：
> `include/motor_drive/ele_motor.h` → `include/motor/ele_motor.h`；
> `src/motor_drive/ele_motor.cpp` → `src/motor/ele_motor.cpp`；
> `include/bsp/bsp_can.h` 🔴（已删）→ `include/transport/can_transport.h` +
> `canet_transport.h` / `usb2can_transport.h`；`src/bsp/bsp_can.cpp` 🔴（已删）→
> `src/transport/{canet,usb2can}_transport.cpp`；
> `include/data_types.h` → `include/common/types.h`；
> `include/motor_drive/ele_motor_def.h` → `include/motor/ele_motor_def.h`。

---

## 实现步骤

### Step 1 — 新增 `MotorManager`

**文件：** `include/motor_manager.h` + `src/motor_manager.cpp`

**职责：**
- 单例，管理 4×3=12 个 `EleMotor` 实例
- `Initialize()` — 初始化4路 CANET TCP 连接，创建12个电机对象
- `Stop()` — 停止后台线程，关闭设备
- 后台接收线程：每1ms 轮询4个 CAN 口，按 `frame.id - 50 = motor_id` 路由帧，更新电机状态
- 线程安全：每个电机一把 `std::mutex`，状态读写加锁

**对外接口：**
```cpp
void EnableMotor(uint8_t can_port, uint8_t motor_id);
void DisableMotor(uint8_t can_port, uint8_t motor_id);
void SetZero(uint8_t can_port, uint8_t motor_id);
void ClearError(uint8_t can_port, uint8_t motor_id);
void SendImpedance(uint8_t can_port, uint8_t motor_id,
                   float pos, float vel, float kp, float kd, float torque);
void SendSpeed(uint8_t can_port, uint8_t motor_id,
               float vel, float kp, float ki);
void SendPosition(uint8_t can_port, uint8_t motor_id,
                  float pos, float kvp, float kp, float kd, float kvi);
MotorStatus GetStatus(uint8_t can_port, uint8_t motor_id) const;
```

---

### Step 2 — 新增 `RobotDog`

**文件：** `include/robot_dog.h` + `src/robot_dog.cpp`

**职责：**
- 包装 `MotorManager`，提供四足语义接口
- 枚举定义：`LegId {FL=0, FR=1, RL=2, RR=3}`，`JointId {HIP=0, THIGH=1, CALF=2}`
- 映射：`can_port = LegId`，`motor_id = JointId + 1`

**对外接口：**
```cpp
bool Initialize();
void Shutdown();
void EnableAll();
void DisableAll();
void EnableLeg(LegId leg);
void DisableLeg(LegId leg);
void SetJointImpedance(LegId leg, JointId joint,
                       float pos, float vel, float kp, float kd, float torque);
void SetJointPosition(LegId leg, JointId joint,
                      float pos, float kp, float kd);
void SetJointSpeed(LegId leg, JointId joint,
                   float vel, float kp, float ki);
void SetLegImpedance(LegId leg,
                     const float pos[3], const float vel[3],
                     const float kp[3],  const float kd[3],
                     const float torque[3]);
MotorStatus GetJointStatus(LegId leg, JointId joint) const;
```

---

### Step 3 — 更新 `CMakeLists.txt`

在 `SOURCES` 列表中添加：
```cmake
src/motor_manager.cpp
src/robot_dog.cpp
```

---

### Step 4 — 更新示例程序

**`include/example.h`** — 新增声明：
```cpp
void Example2_EnableAllMotors();
void Example3_ZeroAllJoints();
void Example4_StandingPosture();
```

**`src/example.cpp`** — 实现三个示例：
- Example2：使能全部12电机，循环打印状态
- Example3：全关节阻抗模式归零（pos=0, kp=10, kd=1）
- Example4：基础站立姿态（各关节设定固定角度）

**`src/main.cpp`** — 新增 case 2/3/4

---

## 控制参数范围

| 参数 | 范围 | 来源 |
|------|------|------|
| 位置 pos | ±12.5 rad | `ele_motor_def.h` |
| 速度 vel | ±65 rad/s | `ele_motor_def.h` |
| 扭矩 torque | ±18 Nm | `ele_motor_def.h` |
| Kp | 0~500 | `ele_motor_def.h` |
| Kd | 0~5 | `ele_motor_def.h` |
| Ki | 0~500 | `ele_motor_def.h` |

> ⚠️ 上表是**计划期**的估值（尤其速度 ±65、扭矩 ±18 均与实现不符）。
> 当前代码（`include/motor/ele_motor_def.h`，按电机类型分行）实测/实配值为：
> 位置 ±12.5 rad；速度 髋/大腿/小腿 ±3.0、轮 ±48.0 rad/s；
> 编解码扭矩量程 髋/大腿 ±120、小腿 ±200、轮 ±52 Nm（`TORQUE_CMD_LIMIT` 同值：
> 120 / 120 / 200 / 52）；Kp 0~500；Kd 0~100；Ki 0~500。

---

## 验证步骤

> 🔴 **以下命令从未可用、现在也不存在**：工程一直是「改 `src/app/main.cpp` 注释 + 重新编译」，
> **不接受命令行参数**，没有 `case 2/3/4`。保留原文仅为对照历史计划。
> 现在的对应做法：把 `main.cpp` 里目标示例的 3 行取消注释后 `cmake --build build -j$(nproc)` 再运行。

```bash
# 1. 编译
cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)

# 2. 使能全部电机（需连接硬件）    🔴 历史计划，当前无此入口
# ./bin/can_motor_app 2

# 3. 全关节归零                    🔴 历史计划，当前无此入口
# ./bin/can_motor_app 3

# 4. 站立姿态                      🔴 历史计划，当前无此入口
# ./bin/can_motor_app 4
```

**当前可用的等价示例**（需在 `src/app/main.cpp` 内取消注释后重编译）：
整狗站立循环见 `Example36_RLStandLoop`，起立→站立→趴下完整流程见 `Example51_StandRLThenLieDown`，
当前激活的遥操作见 `Example37_RLTeleopControl`。

---

## 文件变更汇总

> 🗄️ 下表的真实落地情况（2026-09-29 核对）：
> - `include/motor_manager.h` / `src/motor_manager.cpp` → ✅ 已实现，但现位于
>   `include/motor/motor_manager.h` / `src/motor/motor_manager.cpp`（管理 16 电机）。
> - `include/robot_dog.h` / `src/robot_dog.cpp` → 🔴 **从未创建**。
> - `include/example.h` / `src/example.cpp` → 已演进/改名为
>   `include/app/examples.h` 与 `src/app/examples/*.cpp`。
> - `src/main.cpp` → 现为 `src/app/main.cpp`。

| 操作 | 文件（历史计划） |
|------|------|
| 新增 | `include/motor_manager.h` → 现 `include/motor/motor_manager.h` |
| 新增 | `src/motor_manager.cpp` → 现 `src/motor/motor_manager.cpp` |
| 新增 | `include/robot_dog.h` 🔴 从未实现 |
| 新增 | `src/robot_dog.cpp` 🔴 从未实现 |
| 修改 | `CMakeLists.txt` |
| 修改 | `include/example.h` → 现 `include/app/examples.h` |
| 修改 | `src/example.cpp` → 现 `src/app/examples/*.cpp` |
| 修改 | `src/main.cpp` → 现 `src/app/main.cpp` |
