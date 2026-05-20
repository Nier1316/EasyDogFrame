# 四足机器狗12关节控制系统 — 实现计划

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
| `src/motor_drive/ele_motor.cpp` | `set_motor_para_bt()` 编码发送、`unpack_cmd()` 解包 |
| `include/bsp/bsp_can.h` | CAN 收发抽象层（单例） |
| `src/bsp/bsp_can.cpp` | `Can_Tx()` / `ReceiveFrames()` 实现 |
| `include/data_types.h` | `MotorStatus` / `MotorCommand` / `CanDeviceConfig` |
| `include/motor_drive/ele_motor_def.h` | 参数范围宏、控制命令宏 |

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

---

## 验证步骤

```bash
# 1. 编译
cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)

# 2. 使能全部电机（需连接硬件）
./bin/can_motor_app 2

# 3. 全关节归零
./bin/can_motor_app 3

# 4. 站立姿态
./bin/can_motor_app 4
```

---

## 文件变更汇总

| 操作 | 文件 |
|------|------|
| 新增 | `include/motor_manager.h` |
| 新增 | `src/motor_manager.cpp` |
| 新增 | `include/robot_dog.h` |
| 新增 | `src/robot_dog.cpp` |
| 修改 | `CMakeLists.txt` |
| 修改 | `include/example.h` |
| 修改 | `src/example.cpp` |
| 修改 | `src/main.cpp` |
