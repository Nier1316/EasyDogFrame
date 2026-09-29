# simplify 框架待完善清单

> ⚠️ **本文档已基本过时（横幅更新于 2026-09-29）。**
>
> - ✅ **Bug 1~5 在当前代码中已全部修复**（接口统一、使能/归零/清错、控制帧下发均已实现）；
>   逐条现行代码位置见各 Bug 小节内的「✅ 现行实现」。
> - ℹ️ 实际电机数为 **16**（4 路 × 4，含轮电机）而非 12。
> - 🔴 `RobotDog` 层与「示例 2/3/4」**始终未实现**；现示例体系为
>   **17~57 共 40 个**，当前启用 `Example37_RLTeleopControl`。
> - 🔴 **仍未实现**：`RobotDog` 层、`state_calc` / `monitor` 两个预留线程
>   （`src/runtime/robot_app.cpp` 中仍被注释）、错误自动恢复、参数持久化。
>
> 当前架构见 `docs/SIM2REAL_DEPLOY.md`、`memory/FACT.md`、`FRAMEWORK_GUIDE.md`。

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

**✅ 现行实现（2026-09-29 核对，Bug 已修复）：**
最终采用的是**反向方案**——把 `BspCan` 整个删掉，全部统一到 `CanTransport` 抽象，
而不是本节建议的"`MotorManager` 改用 `BspCan` 单例"：

- 纯虚接口：`include/transport/can_transport.h`（`open/send/sendBatch/recv/close/shutdown`）
- `MotorManager` 按通道持有后端指针：`include/motor/motor_manager.h:142`
  `CanTransport* m_transport[CAN_PORTS]`；注入接口 `SetTransport()` / `SetChannelTransport()`
  （声明 `include/motor/motor_manager.h:42,50`；实现 `src/motor/motor_manager.cpp:51,55`）
- 未注入时在 `Initialize()` 兜底为默认后端 **`Usb2CanTransport`（达妙 USB2CAN）**
  —— `src/motor/motor_manager.cpp:62-67`
- 单电机发帧统一走 `motor.transport->send(device_idx, f)` —— `src/motor/ele_motor.cpp:38`
  （收发同一后端对象，已不存在"两套句柄、单例从未初始化"的问题）
- 后端实现：`src/transport/canet_transport.cpp`（CANET TCP，仅 Example27/28 直连）、
  `src/transport/usb2can_transport.cpp`（达妙 USB2CAN，默认）

> 注：`BspCanFrame` 类型也随之改名为 `CanFrame`（`include/common/types.h`）。

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

**✅ 现行实现（2026-09-29 核对，Bug 已修复）：**
使能/失能现在**确实会发 CAN 帧**，逻辑已下沉到 `EleMotor`：

- 启动帧（`0xFC`，帧体 `80 FF FF FF FF FF FF FC`）：`EleMotor::enable()`
  — **`src/motor/ele_motor.cpp:29`**（发帧在 `ele_motor.cpp:38`，经 `transport->send`）
- 停止帧（`0xFD`，帧体 `80 FF FF FF FF FF FF FD`）：`EleMotor::disable()`
  — **`src/motor/ele_motor.cpp:41`**
- `MotorManager` 侧调用点：`MotorManager::EnableMotor()` —— **`src/motor/motor_manager.cpp:215`**
  （轮子额外置 `estop_grace_ticks` 急停静默窗口）；`MotorManager::DisableMotor()`
  —— **`src/motor/motor_manager.cpp:230`**
- 命令宏：`MOTOR_STRAT=0xFC` / `MOTOR_STOP=0xFD`（`include/motor/ele_motor_def.h:153-154`）

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

**✅ 现行实现（2026-09-29 核对，Bug 已修复）：**

- `MotorManager::SetZero()` —— **`src/motor/motor_manager.cpp:254`**：清零 `motor.target_position`
  后调用 `float2bag(motor, 0.0f, 1, MOTOR_ANGLE_ZERO)` 真正下发归零帧
  —— 发帧语句在 **`src/motor/motor_manager.cpp:263`**
- 归零命令宏 `MOTOR_ANGLE_ZERO=0xFE`（`include/motor/ele_motor_def.h:155`）

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

**✅ 现行实现（2026-09-29 核对，Bug 已修复）：**

- `MotorManager::ClearError()` —— **`src/motor/motor_manager.cpp:267`**：清零本地 `motor.error_code`
  后调用 `float2bag(motor, 0.0f, 1, MOTOR_CLEAR_ERROR)` 真正下发清错帧
  —— 发帧语句在 **`src/motor/motor_manager.cpp:276`**
- 清错命令宏 `MOTOR_CLEAR_ERROR=0xF4`（`include/motor/ele_motor_def.h:159`）

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

**✅ 现行实现（2026-09-29 核对，Bug 已修复）：**
本节推荐的"异步发送、由发送线程统一驱动"已落地，只是线程名从 `SendThreadFunc`
改为 `MotorManager::SendOnce()`、节拍从 1ms 改为 **2ms（500Hz）**：

- 三个设置函数只更新 target / 模式（不直接发帧）：
  `MotorManager::SendImpedance()` —— **`src/motor/motor_manager.cpp:280`**
  `MotorManager::SendSpeed()` —— **`src/motor/motor_manager.cpp:296`**
  `MotorManager::SendPosition()` —— **`src/motor/motor_manager.cpp:310`**
- 统一发送循环：`MotorManager::SendOnce()` —— **`src/motor/motor_manager.cpp:370`**
  遍历 16 电机，按 `motor.control_mode` 调 `set_motor_para_bt()` 下发
  IMPEDANCE / SPEED / POSITION 三种帧；循环前部还含：
  轮子急停与超速保护（`WHEEL_ESTOP_*`）、固件模式同步（`MODE_SETTLE_TICKS`）、
  腿摩擦前馈每 2ms 重算叠加
- 线程注册：`RegisterMotorIoThreads()` —— **`src/runtime/motor_io.cpp:9-23`**
  `motor_receive` / `motor_send` 各 **2ms（500Hz）、SCHED_FIFO 优先级 80**
  （**不是**本节写的 1ms）
- `EleMotor` 结构体已含所需字段（`control_mode` / `kp` / `kd` / `ki` / `kvp` / `kvi`），
  见 `include/motor/ele_motor.h`

---

## 🟡 功能缺失 1：RobotDog 四足语义层未实现

> 🔴 **2026-09-29 复核：仍未实现。** 全仓不存在 `include/robot_dog.h` / `src/robot_dog.cpp`
> 之类的文件（`grep -rn "RobotDog" src include` 无任何命中），也没有 `LegId` / `JointId` 枚举。
> 四足语义现由 `motion` 层（`MotionController`、`include/motion/leg_kinematics.h`、
> `include/motion/robot_calibration.h`）与各示例直接调用 `MotorManager` 承担。

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

> 🔴 **2026-09-29 复核：本节的编号体系已被彻底取代，不再适用。**
> 工程从未实现 `Example2/3/4`，也**没有** `main.cpp` 的 `case 2/3/4` 分支
> （`main()` 不接受命令行参数）。现行示例体系为 **`Example17~Example57` 共 40 个**，
> 切换方式是改 `src/app/main.cpp` 注释 + 重新编译。等价能力由
> `Example36_RLStandLoop`（站立循环）、`Example51_StandRLThenLieDown`（站立→趴下）等承担。
> 计划期文件 `include/example.h` / `src/example.cpp` 现为
> `include/app/examples.h` / `src/app/examples/*.cpp`。

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

> 🔴 **以上四项到 2026-09-29 为止仍未实现**（逐项核对）：
> - `state_calc` / `monitor`：**仍是注释**，见 `src/runtime/robot_app.cpp:26-27`（注册）、
>   `:37-38`（启动）、`:52-53`（停止）；全仓无这两个线程的注册调用。
> - 错误自动恢复：`ReceiveOnce()`（`src/motor/motor_manager.cpp:145`）只更新状态，
>   检测到 `error_code != 0` 时**不会**自动 `ClearError` + 重新使能，需上层手动处理。
> - 参数持久化：全仓无 `MOTOR_OW_Save_Patemeter` / `0x2A` 的调用点。
>
> 另：`RobotApp` 现在**无人使用**——各示例多自建局部 `ThreadManager`；
> `src/app/main.cpp` 中此前的 `static RobotApp g_app`（静态析构顺序 UB）已于 2026-09-29 删除。

---

## 修复顺序建议

```
Bug 1（统一接口）→ Bug 2/3/4（命令发送）→ Bug 5（SendThreadFunc）
→ 功能缺失 1（RobotDog）→ 功能缺失 2（示例程序）
```

> 🗄️ 上表为历史建议。**Bug 1~5 已全部修复**（现行位置见各小节「✅ 现行实现」）；
> 「功能缺失 1（RobotDog）」至今**仍未实现**（全仓无 `robot_dog.h/.cpp`）；
> 「功能缺失 2（示例 2/3/4）」**已被 17~57 的 40 个示例体系取代**，
> 等价能力由 `Example36_RLStandLoop`（站立）、`Example51_StandRLThenLieDown`（站立→趴下）等承担。
