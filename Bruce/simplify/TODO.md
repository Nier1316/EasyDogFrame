# simplify 框架待完善清单

## 优先级说明
- 🔴 阻塞性 Bug — 不修复则电机完全无法控制
- 🟡 功能缺失 — 框架不完整
- 🟢 扩展功能 — 框架完善后再做

---

## 🔴 Bug 1：BspCan 与 CanDevice 接口不统一（最优先）

**文件：** `src/motor_manager.cpp:30-54` / `src/motor_drive/ele_motor.cpp:43-48`

**问题：**
`MotorManager::Initialize()` 通过独立的 `CanDevice` 实例打开 4 路 CAN 设备，
但 `float2bag()` 和 `set_motor_para_bt()` 调用的是 `BspCan::GetInstance().Can_Tx()`。
两套接口操作的是不同的设备句柄，`BspCan` 单例从未被初始化，所有发帧操作都会静默失败。

**修复方案：**
将 `MotorManager::Initialize()` 改为通过 `BspCan::GetInstance()` 初始化设备：
```cpp
// 替换 CanDevice 初始化逻辑，改为：
BspCan::GetInstance().InitDevice(i, config);
BspCan::GetInstance().StartDevice(i);
// 同时移除 m_can_devices 成员，ReceiveThreadFunc 改用 BspCan::GetInstance().ReceiveFrames()
```

---

## 🔴 Bug 2：EnableMotor / DisableMotor 没有发送 CAN 命令

**文件：** `src/motor_manager.cpp:126-146`

**问题：**
`EnableMotor` 只修改了 `motor.enabled = true`，没有向电机发送启动命令。
`DisableMotor` 同理，只改了标志位，电机实际不会响应。

**修复方案：**
```cpp
void MotorManager::EnableMotor(uint8_t can_port, uint8_t motor_id) {
    // ... 参数校验 ...
    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.enabled = true;
    float2bag(motor, 0.0f, 1, MOTOR_STRAT);  // 发送启动命令 0xFC
}

void MotorManager::DisableMotor(uint8_t can_port, uint8_t motor_id) {
    // ...
    motor.enabled = false;
    float2bag(motor, 0.0f, 1, MOTOR_STOP);   // 发送停止命令 0xFD
}
```

---

## 🔴 Bug 3：SetZero 没有发送归零命令

**文件：** `src/motor_manager.cpp:148-157`

**问题：**
`SetZero` 只清零了 `motor.target_position`，没有向电机发送角度归零命令帧。

**修复方案：**
```cpp
void MotorManager::SetZero(uint8_t can_port, uint8_t motor_id) {
    // ...
    motor.target_position = 0;
    float2bag(motor, 0.0f, 1, MOTOR_ANGLE_ZERO);  // 发送归零命令 0xFE
}
```

---

## 🔴 Bug 4：ClearError 没有发送清错命令

**文件：** `src/motor_manager.cpp:159-168`

**问题：**
`ClearError` 只清零了本地 `motor.error_code`，没有向电机发送清除错误命令帧。

**修复方案：**
```cpp
void MotorManager::ClearError(uint8_t can_port, uint8_t motor_id) {
    // ...
    motor.error_code = 0;
    float2bag(motor, 0.0f, 1, MOTOR_CLEAR_ERROR);  // 发送清错命令 0xF4
}
```

---

## 🔴 Bug 5：SendImpedance / SendSpeed / SendPosition 没有发送 CAN 帧

**文件：** `src/motor_manager.cpp:170-205`

**问题：**
三个发送函数只更新了 target 字段，注释写着"这里需要调用 set_motor_para_bt"但未实现。
调用这三个函数后电机不会收到任何指令。

**修复方案（推荐：异步发送，由 SendThreadFunc 统一驱动）：**

`SendImpedance/SendSpeed/SendPosition` 只负责更新 target 字段（已有），
在 `EleMotor` 中增加 `control_mode` 字段记录当前模式，
`SendThreadFunc` 每 1ms 遍历所有 enabled 电机，按 `control_mode` 调用 `set_motor_para_bt` 发帧：

```cpp
void MotorManager::SendThreadFunc() {
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
            EleMotor& motor = m_motors[can_port][motor_id - 1];
            if (!motor.enabled) continue;

            switch (motor.control_mode) {
                case IMPEDANCE:
                    set_motor_para_bt(motor,
                        motor.target_position, motor.target_speed,
                        motor.kp, motor.kd, motor.target_torque, IMPEDANCE);
                    break;
                case SPEED:
                    set_motor_para_bt(motor,
                        motor.target_speed, motor.kp, 0, 0, motor.ki, SPEED);
                    break;
                case POSITION:
                    set_motor_para_bt(motor,
                        motor.target_position, motor.kvp, motor.kp,
                        motor.kd, motor.kvi, POSITION);
                    break;
            }
        }
    }
}
```

**需要同步修改 `EleMotor` 结构体，增加以下字段：**
```cpp
ControlMode control_mode;  // 当前控制模式
float kp, kd, ki;          // 阻抗/速度控制增益
float kvp, kvi;            // 位置控制增益
```

---

## 🟡 功能缺失 1：RobotDog 四足语义层未实现

**计划文件：** `include/robot_dog.h` + `src/robot_dog.cpp`（PLAN.md Step 2）

**需要实现：**
- 枚举 `LegId {FL=0, FR=1, RL=2, RR=3}` 和 `JointId {HIP=0, THIGH=1, CALF=2}`
- 映射关系：`can_port = LegId`，`motor_id = JointId + 1`
- 包装 `MotorManager` 的接口，提供四足语义 API：
  - `EnableAll()` / `DisableAll()`
  - `EnableLeg(LegId)` / `DisableLeg(LegId)`
  - `SetJointImpedance(LegId, JointId, pos, vel, kp, kd, torque)`
  - `SetJointPosition(LegId, JointId, pos, kp, kd)`
  - `SetJointSpeed(LegId, JointId, vel, kp, ki)`
  - `SetLegImpedance(LegId, pos[3], vel[3], kp[3], kd[3], torque[3])`
  - `GetJointStatus(LegId, JointId) -> MotorStatus`

---

## 🟡 功能缺失 2：示例程序 2/3/4 未实现

**文件：** `include/example.h` + `src/example.cpp` + `src/main.cpp`（PLAN.md Step 4）

**需要实现：**
- `Example2_EnableAllMotors()` — 使能全部 12 电机，循环打印状态
- `Example3_ZeroAllJoints()` — 全关节阻抗模式归零（pos=0, kp=10, kd=1）
- `Example4_StandingPosture()` — 基础站立姿态（各关节设定固定角度）
- `main.cpp` 新增 case 2/3/4

---

## 🟢 扩展功能（后续）

- **状态监控线程**：`robot_app.h` 中预留的 `monitor` 线程（100ms，优先级0），定期打印所有电机状态
- **状态解算线程**：`state_calc` 线程（5ms，优先级50），预留给正运动学/逆运动学计算
- **错误自动恢复**：接收线程检测到 `error_code != 0` 时自动触发 `ClearError` 并重新使能
- **参数持久化**：通过 `MOTOR_OW_Save_Patemeter (0x2A)` 将调好的增益写入电机 FLASH

---

## 修复顺序建议

```
Bug 1（统一接口）→ Bug 2/3/4（命令发送）→ Bug 5（SendThreadFunc）
→ 功能缺失 1（RobotDog）→ 功能缺失 2（示例程序）
```
