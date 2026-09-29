# MotorManager 使用指南

## 📋 目录
1. [概述](#概述)
2. [初始化](#初始化)
3. [API 参考](#api-参考)
4. [工作流程](#工作流程)
5. [使用示例](#使用示例)
6. [常见问题](#常见问题)

---

## 概述

**MotorManager** 是四足机器狗 16 电机（12 关节 + 4 轮）控制系统的核心管理器，负责：
- 管理 4 路 CAN 传输（默认达妙 USB2CAN，可注入 CANET TCP，CAN0~CAN3）
- 管理 16 个电机对象（4 路 × 4 电机：3 关节 + 1 轮）
- 后台接收电机状态（2ms 周期，500Hz）
- 后台发送控制命令（2ms 周期，500Hz）
- 提供线程安全的控制接口

### 硬件拓扑

```
CAN0 (左前腿FL)    → 电机1(髋), 2(大腿), 3(小腿), 4(轮)
CAN1 (右前腿FR)    → 电机1(髋), 2(大腿), 3(小腿), 4(轮)
CAN2 (左后腿RL)    → 电机1(髋), 2(大腿), 3(小腿), 4(轮)
CAN3 (右后腿RR)    → 电机1(髋), 2(大腿), 3(小腿), 4(轮)
```

传输后端由 `CanTransport` 抽象（见 `include/transport/CAN_TRANSPORT_GUIDE.md`）：
- **默认：达妙 USB2FDCAN（`Usb2CanTransport`）**，逻辑路 idx → 物理 `(idx/2, idx%2)`；
- CANET TCP（`CanetTransport`，`192.168.0.178`，端口 `4001~4004`）**已弃用**，仅在显式注入时使用。

### 关键特性

| 特性 | 说明 |
|------|------|
| **单例模式** | 全局唯一实例 |
| **线程安全** | 每个电机独立互斥锁 |
| **异步驱动** | 由外部 ThreadManager 驱动 |
| **三种控制模式** | 阻抗/速度/位置 |
| **实时性** | 2ms 周期接收/发送（500Hz） |

---

## 初始化

### 步骤 0（可选）：注入传输后端

默认后端就是达妙 USB2FDCAN，不需要注入；要换后端（如老 CANET）时，**必须在 `Initialize()` 之前**调用
`SetTransport()`（4 路全换）或 `SetChannelTransport(can_port, ...)`（只换某一路）：

```cpp
#include "transport/usb2can_transport.h"

// 例：只把 CAN1 换成达妙 USB2CAN（其余路走默认）
motor_mgr.SetChannelTransport(1, &Usb2CanTransport::GetInstance());
```

### 步骤 1：获取单例

```cpp
#include "motor/motor_manager.h"

MotorManager& motor_mgr = MotorManager::GetInstance();
```

### 步骤 2：初始化（需要 ThreadManager）

```cpp
#include "runtime/thread_manager.h"

ThreadManager thread_mgr;

// 初始化 MotorManager（会向 thread_mgr 注册 motor_receive 和 motor_send 线程）
if (!motor_mgr.Initialize(thread_mgr)) {
    printf("[ERROR] Failed to initialize MotorManager\n");
    return false;
}

// 启动已注册的线程（各 2ms 周期 = 500Hz，优先级 80）
thread_mgr.start_thread("motor_receive");  // 后台接收线程 → MotorManager::ReceiveOnce()
thread_mgr.start_thread("motor_send");     // 后台发送线程 → MotorManager::SendOnce()
```

### 步骤 3：关闭

```cpp
// 停止线程
thread_mgr.stop_thread("motor_receive");
thread_mgr.stop_thread("motor_send");

// 关闭 MotorManager（关闭 4 路 CAN 设备）
motor_mgr.Stop();
```

---

## API 参考

### 使能前的准备（顺序敏感）

#### `SetControlMode(can_port, motor_id, mode)`
预写固件控制模式（`IMPEDANCE=0` / `SPEED=1` / `POSITION=2`）。
**必须在 `EnableMotor()` 之前调用**：该函数在调用线程**直接发 `MOTOR_WR_CONTROL_MODE`(0x5B) 帧**，
不受"未使能则发送线程跳过"的限制；若先使能再设模式，电机会先在固件默认模式（阻抗）下被使能而意外运动
（实测 CAN1 轮在使能瞬间就转起来）。

```cpp
// 腿走阻抗、轮走速度环（固件阻抗环忽略 vel_des）
for (int cp = 0; cp < 4; cp++) {
    for (int mi = 1; mi <= 3; mi++) motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    motor_mgr.SetControlMode(cp, 4, SPEED);
}
```

---

#### `PreEnableZeroTorque(can_port, motor_id)`
使能前预置零扭矩阻抗控制帧（**直发**，不受发送线程 `enabled` 检查限制），
覆盖固件残留目标（上一次命令/上电默认），使能瞬间零扭矩不冲。
推荐顺序：`SetControlMode → PreEnableZeroTorque → EnableMotor`；使能后由后续正常增益的控制帧接管。

```cpp
motor_mgr.PreEnableZeroTorque(0, 1);
```

---

#### `ReadParam(can_port, motor_id, type)`
读固件参数寄存器（`type` 见 `ele_motor_def.h` 的 `MOTOR_OR_*` / `MOTOR_WR_*`）。
**异步**：回帧由接收线程打印 `[PARAM] CANx motory type=0xNN value=...`。使能前也可调用。

```cpp
motor_mgr.ReadParam(0, 1, MOTOR_WR_CONTROL_MODE);   // 核对固件当前控制模式
motor_mgr.ReadParam(0, 4, MOTOR_OR_velocity);       // 读轮速
```

---

### 电机使能/禁用

#### `EnableMotor(can_port, motor_id)`
使能指定电机，发送启动命令。

```cpp
// 使能 CAN0 上的电机 1（左前腿髋关节）
motor_mgr.EnableMotor(0, 1);

// 使能 CAN1 上的电机 2（右前腿大腿）
motor_mgr.EnableMotor(1, 2);
```

**参数：**
- `can_port` (uint8_t): CAN 口索引 [0, 3]
- `motor_id` (uint8_t): 电机 ID **[1, 4]**（1=髋 2=大腿 3=小腿 4=轮）

**发送命令：** `MOTOR_STRAT` (启动)

---

#### `DisableMotor(can_port, motor_id)`
禁用指定电机，发送停止命令。

```cpp
motor_mgr.DisableMotor(0, 1);
```

**发送命令：** `MOTOR_STOP` (停止)

---

### 电机特殊操作

#### `SetZero(can_port, motor_id)`
将指定电机归零，发送归零命令。

```cpp
// 将所有电机归零
for (uint8_t can_port = 0; can_port < 4; can_port++) {
    for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
        motor_mgr.SetZero(can_port, motor_id);
    }
}
```

**发送命令：** `MOTOR_ANGLE_ZERO` (归零)

---

#### `ClearError(can_port, motor_id)`
清除指定电机的错误状态。

```cpp
motor_mgr.ClearError(0, 1);
```

**发送命令：** `MOTOR_CLEAR_ERROR` (清错)

---

### 电机控制命令

#### `SendImpedance(can_port, motor_id, pos, vel, kp, kd, torque)`
发送阻抗控制命令（位置 + 速度 + 扭矩前馈）。

```cpp
// 阻抗控制：目标位置 0.5rad，目标速度 0，刚度 10，阻尼 1，扭矩前馈 0
motor_mgr.SendImpedance(0, 1, 0.5f, 0.0f, 10.0f, 1.0f, 0.0f);
```

**参数范围：**
- `pos`: ±12.5 rad
- `vel`: 关节 ±3 / 轮 ±48 rad/s
- `kp`: 0~500
- `kd`: 0~100
- `torque`: Hip/Thigh ±120、Calf ±200、Wheel ±52 Nm（编码前由 `TORQUE_CMD_LIMIT` clamp）

**发送命令：** `set_motor_para_bt(..., IMPEDANCE)`

---

#### `SendSpeed(can_port, motor_id, vel, kp, ki)`
发送速度控制命令。

```cpp
// 速度控制：目标速度 1.0 rad/s，速度环 Kp=10，Ki=0.5
motor_mgr.SendSpeed(0, 1, 1.0f, 10.0f, 0.5f);
```

**参数范围：**
- `vel`: 关节 ±3 / 轮 ±48 rad/s
- `kp`: 0~500
- `ki`: 0~500

**发送命令：** `set_motor_para_bt(..., SPEED)`

---

#### `SendPosition(can_port, motor_id, pos, kvp, kp, kd, kvi)`
发送位置控制命令。

```cpp
// 位置控制：目标位置 0.5rad，位置环 Kvp=5，Kp=10，Kd=1，速度环 Ki=0.5
motor_mgr.SendPosition(0, 1, 0.5f, 5.0f, 10.0f, 1.0f, 0.5f);
```

**参数范围：**
- `pos`: ±12.5 rad
- `kvp`: 0~500
- `kp`: 0~500
- `kd`: 0~100
- `kvi`: 0~500

**发送命令：** `set_motor_para_bt(..., POSITION)`

---

### 状态查询

#### `GetStatus(can_port, motor_id)`
查询指定电机的当前状态。

```cpp
MotorStatus status = motor_mgr.GetStatus(0, 1);

printf("Position: %.2f rad\n", status.position);
printf("Velocity: %.2f rad/s\n", status.velocity);
printf("Torque: %.2f Nm\n", status.torque);
printf("Enabled: %d\n", status.enable);
printf("Error: 0x%02x\n", status.error_code);
```

**返回值：** `MotorStatus` 结构体（定义在 `include/common/types.h`；`ack`/`fault` 字段早已移除）
```cpp
struct MotorStatus {
    uint8_t motor_id;       // CAN_ID（data[0] bit3-0）
    bool enable;            // 使能状态（应用层命令态）
    float position;         // 当前位置 (rad)
    float velocity;         // 当前速度 (rad/s)
    float torque;           // 当前扭矩 (Nm)
    uint8_t error_code;     // 电机错误码（固件原始错误寄存器值，非 0 即故障）
};
```

---

### 腿阻抗前馈覆盖（500Hz）

#### `SetLegTauFFOverride(fn)` / `ClearLegTauFFOverride()`
注册/清除 500Hz 腿摩擦前馈回调。`SendOnce`（2ms）用最新 `q/q̇` 重算并把扭矩增量叠加到 `target_torque`
（随 target 一起逆标定翻转）。回调返回 `status`/目标坐标下的扭矩增量；**空回调 = 关闭**（`target_torque` 原样）。
分层上 motor 层只提供钩子，不依赖 strategy；实现方（`rl::leg_friction_ff`）在 strategy 层，
由 `examples_common` 的 `EnableRlFrictionFF/DisableRlFrictionFF` 统一开关。

```cpp
using LegTauFFOverrideFn = std::function<float(uint8_t can_port, uint8_t motor_id,
        float pos_status, float vel_status, float qdes_status, float kp, float kd)>;
void SetLegTauFFOverride(LegTauFFOverrideFn fn);
void ClearLegTauFFOverride();
```

---

### 轮子急停（安全）

#### `WheelEmergencyStop()` / `ClearWheelEmergency()` / `wheelEmergency()`
触发轮子急停：置急停标志 + 直发 SPEED 制动帧 + 制动增益帧（`WHEEL_ESTOP_KVP = 3.0`）。
触发后 `SendOnce` 每周期对 4 个轮子**强制发制动帧（忽略上层目标）**，
直到 `ClearWheelEmergency()` 解除。轮速超 **15 rad/s**（`WHEEL_ESTOP_VEL`）也会**自动**置位
（自动超速为瞬态，只当次制动；手动 `WheelEmergencyStop()` 才保持）。
制动目标速度用 `WHEEL_ESTOP_VEL_CMD = 0.02 rad/s`（避开固件对 `v=0` 的特殊语义），
速度反馈正常时固件速度环闭环到 0 速附近（主动制动，非自由滑行）。
使能后有 `WHEEL_ESTOP_GRACE_TICKS = 1000`（2s @500Hz）静默窗口，避免使能瞬间假速度偏移误触发。

> ⚠️ 轮子上位机扭矩软限位（`rl_controller.h` 的 `WHEEL_SOFT_LIMIT_*` / `wheel_torque()`）**已删除**：
> SPEED 迁移后轮子走固件速度环，不再经过该路径。**轮速保护现由 `WHEEL_ESTOP_*` 承担**
> （15 rad/s 自动急停 + 固件速度环制动）。

```cpp
motor_mgr.WheelEmergencyStop();     // 急停
motor_mgr.ClearWheelEmergency();    // 解除
bool estop = motor_mgr.wheelEmergency();
```

---

### 单次 IO（线程入口）

#### `ReceiveOnce()` / `SendOnce()`
`motor_receive` / `motor_send` 两个 500Hz 线程实际调用的单次轮询函数。
**MotorManager 不拥有线程生命周期**，线程由 `runtime/motor_io.h` 的
`RegisterMotorIoThreads(thread_mgr, mm)` 在 `Initialize()` 内注册。
- `ReceiveOnce()`：轮询 4 路 CAN 并解包状态帧，同时更新接收心跳 `GetReceiveHeartbeatMs()`；
- `SendOnce()`：遍历 enabled 电机，按 `control_mode` 编帧发送（轮子上层闭环与摩擦前馈覆盖也在这一步）。

诊断：心跳长时间不前进 + `Usb2CanTransport::RxCount()` 仍在涨 → 接收线程卡死；两者都停 → SDK 回调停。

---

## 工作流程

### 完整的控制流程

```
┌─────────────────────────────────────────────────────────┐
│ 应用层                                                  │
│ motor_mgr.SetControlMode(0, 1, IMPEDANCE)   // 使能前   │
│ motor_mgr.PreEnableZeroTorque(0, 1)         // 使能前   │
│ motor_mgr.EnableMotor(0, 1)                             │
│ motor_mgr.SendImpedance(0, 1, 0.5, 0, 10, 1, 0)        │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────────────────────┐
│ MotorManager 状态更新                                   │
│ - 更新 motor.target_position = 0.5                      │
│ - 更新 motor.kp = 10, motor.kd = 1                      │
│ - 设置 motor.control_mode = IMPEDANCE                   │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓ (2ms 周期，motor_send 线程)
┌─────────────────────────────────────────────────────────┐
│ MotorManager::SendOnce()                                │
│ - 遍历所有 enabled 电机                                 │
│ - 按 control_mode 调用 set_motor_para_bt()              │
│ - 编码 CAN 帧并发送                                     │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────────────────────┐
│ CanTransport::send()                                    │
│ - 默认 Usb2CanTransport（达妙 USB2FDCAN）发到电机       │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────────────────────┐
│ 电机执行命令                                            │
│ - 按已同步的固件模式控制（IMPEDANCE/SPEED/POSITION）    │
│ - 返回状态帧                                            │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓ (2ms 周期，motor_receive 线程)
┌─────────────────────────────────────────────────────────┐
│ MotorManager::ReceiveOnce()                             │
│ - 轮询 4 路 CAN 口                                      │
│ - 接收电机状态帧 (ID: 51-54)                            │
│ - 调用 unpack_frame() 解包（含按字段标定）              │
│ - 更新 motor.current_position/velocity/torque           │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────────────────────┐
│ 应用层                                                  │
│ status = motor_mgr.GetStatus(0, 1)                      │
│ printf("Position: %.2f\n", status.position)             │
└─────────────────────────────────────────────────────────┘
```

---

## 使用示例

### 示例 1：基础使能和控制

```cpp
#include "motor/motor_manager.h"
#include "runtime/thread_manager.h"

int main() {
    // 初始化
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize\n");
        return -1;
    }
    
    // 启动线程（各 2ms = 500Hz）
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    
    // 使能前先写固件模式 + 预置零扭矩
    motor_mgr.SetControlMode(0, 1, IMPEDANCE);
    motor_mgr.PreEnableZeroTorque(0, 1);

    // 使能电机
    motor_mgr.EnableMotor(0, 1);
    sleep(1);  // 等待电机启动
    
    // 发送阻抗控制命令
    motor_mgr.SendImpedance(0, 1, 0.5f, 0.0f, 10.0f, 1.0f, 0.0f);
    
    // 循环读取状态
    for (int i = 0; i < 100; i++) {
        MotorStatus status = motor_mgr.GetStatus(0, 1);
        printf("Pos: %.2f, Vel: %.2f, Torque: %.2f\n",
               status.position, status.velocity, status.torque);
        sleep(0.01);  // 10ms
    }
    
    // 禁用电机
    motor_mgr.DisableMotor(0, 1);
    
    // 关闭
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    
    return 0;
}
```

---

### 示例 2：全电机初始化和归零

```cpp
void InitializeAllMotors() {
    MotorManager& motor_mgr = MotorManager::GetInstance();
    
    // 使能所有 16 个电机
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
            motor_mgr.EnableMotor(can_port, motor_id);
        }
    }
    
    sleep(1);  // 等待所有电机启动
    
    // 全部归零
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
            motor_mgr.SetZero(can_port, motor_id);
        }
    }
    
    printf("[INFO] All motors initialized and zeroed\n");
}
```

---

### 示例 3：三种控制模式切换

```cpp
void DemoControlModes() {
    MotorManager& motor_mgr = MotorManager::GetInstance();
    uint8_t can_port = 0, motor_id = 1;
    
    motor_mgr.EnableMotor(can_port, motor_id);
    sleep(1);
    
    // 模式 1：阻抗控制（位置 + 速度 + 扭矩）
    printf("[INFO] Impedance mode\n");
    motor_mgr.SendImpedance(can_port, motor_id, 0.5f, 0.0f, 10.0f, 1.0f, 0.0f);
    sleep(2);
    
    // 模式 2：速度控制
    printf("[INFO] Speed mode\n");
    motor_mgr.SendSpeed(can_port, motor_id, 1.0f, 10.0f, 0.5f);
    sleep(2);
    
    // 模式 3：位置控制
    printf("[INFO] Position mode\n");
    motor_mgr.SendPosition(can_port, motor_id, 0.5f, 5.0f, 10.0f, 1.0f, 0.5f);
    sleep(2);
    
    motor_mgr.DisableMotor(can_port, motor_id);
}
```

---

### 示例 4：错误处理

```cpp
void HandleMotorError() {
    MotorManager& motor_mgr = MotorManager::GetInstance();
    uint8_t can_port = 0, motor_id = 1;
    
    MotorStatus status = motor_mgr.GetStatus(can_port, motor_id);
    
    if (status.error_code != 0) {
        printf("[WARNING] Motor error detected: 0x%02x\n", status.error_code);
        
        // 清除错误
        motor_mgr.ClearError(can_port, motor_id);
        sleep(0.5);
        
        // 重新使能
        motor_mgr.EnableMotor(can_port, motor_id);
        sleep(1);
        
        printf("[INFO] Motor recovered\n");
    }
}
```

---

## 常见问题

### Q1：如何同时控制多个电机？

**A：** MotorManager 支持并发控制，每个电机独立互斥锁。

```cpp
// 并发控制多个电机
motor_mgr.SendImpedance(0, 1, 0.5f, 0.0f, 10.0f, 1.0f, 0.0f);
motor_mgr.SendImpedance(0, 2, 0.3f, 0.0f, 10.0f, 1.0f, 0.0f);
motor_mgr.SendImpedance(0, 3, 0.2f, 0.0f, 10.0f, 1.0f, 0.0f);
```

---

### Q2：SendOnce / ReceiveOnce 何时被调用？

**A：** 两者分别由 `motor_send` / `motor_receive` 线程以 2ms（500Hz）周期驱动，不需要手动调用。
注册发生在 `MotorManager::Initialize()` 内的 `RegisterMotorIoThreads()`（`src/runtime/motor_io.cpp`）：

```cpp
// Initialize() 内已注册（等价代码）
thread_mgr.register_thread(
    "motor_send",
    [&mm]() { mm.SendOnce(); },
    ThreadMode::LOOP, 2, 80  // 2ms 间隔，SCHED_FIFO 优先级 80
);
thread_mgr.register_thread(
    "motor_receive",
    [&mm]() { mm.ReceiveOnce(); },
    ThreadMode::LOOP, 2, 80
);
```

---

### Q3：如何监控电机状态？

**A：** 使用 GetStatus() 定期查询。

```cpp
// 后台监控线程
while (running) {
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
            MotorStatus status = motor_mgr.GetStatus(can_port, motor_id);
            if (status.error_code != 0) {
                printf("[ERROR] Motor %d:%d error: 0x%02x\n",
                       can_port, motor_id, status.error_code);
            }
        }
    }
    sleep(0.1);  // 100ms
}
```

---

### Q4：参数范围是多少？

**A：** 见下表：

| 参数 | 范围 | 单位 |
|------|------|------|
| 位置 (pos) | ±12.5 | rad |
| 速度 (vel) | 关节 ±3 / 轮 ±48 | rad/s |
| 扭矩 (torque) | Hip/Thigh ±120、Calf ±200、Wheel ±52 | Nm |
| Kp | 0~500 | — |
| Kd | 0~100 | — |
| Ki | 0~500 | — |

> 上表 = `include/motor/ele_motor_def.h` 的 `MOTOR_LIMITS`（固件 `CAN_REPLY_*` 编解码量程），
> 命令扭矩另由 `TORQUE_CMD_LIMIT`（同值）在编码前 clamp。
> 轮速保护由 `WHEEL_ESTOP_*`（15 rad/s 自动急停 + 固件速度环制动）承担，不是上位机扭矩软限位。

---

### Q5：如何调试解包过程？

**A：** 解包发生在 `MotorManager::ReceiveOnce()` 调用的 `unpack_frame()` 中，可在那里加日志。

```cpp
// 在 unpack_frame() 后添加
printf("[DEBUG] Motor %d:%d - Pos: %.2f, Vel: %.2f, Torque: %.2f\n",
       can_port, motor_id,
       motor.current_position, motor.current_speed, motor.current_torque);
```

---

### Q6：线程安全性如何保证？

**A：** 每个电机一把独立互斥锁（`MotorManager::m_motor_mutex[CAN_PORTS][MOTORS_PER_CAN]`），
接收线程和应用线程互斥访问。`EleMotor` 自身**不带锁**（原 `state_mutex` 已删除，全仓无使用处）。

```cpp
// 接收线程
std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
unpack_frame(motor, frame.data, frame.dlc);

// 应用线程
std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
MotorStatus status = GetStatus(...);
```

---

## 总结

| 操作 | 函数 | 周期 |
|------|------|------|
| 初始化 | `Initialize()` | 一次 |
| 传输注入 | `SetTransport/SetChannelTransport` | 一次（Initialize 前） |
| 模式预写 | `SetControlMode` | 按需（EnableMotor 前） |
| 使能前零扭矩 | `PreEnableZeroTorque` | 按需（EnableMotor 前） |
| 使能/禁用 | `EnableMotor/DisableMotor` | 按需 |
| 控制 | `SendImpedance/Speed/Position` | 按需 |
| 腿摩擦前馈 | `SetLegTauFFOverride/ClearLegTauFFOverride` | 一次开关 |
| 特殊操作 | `SetZero/ClearError/ReadParam` | 按需 |
| 轮子急停 | `WheelEmergencyStop/ClearWheelEmergency` | 按需 |
| 状态查询 | `GetStatus()` | 按需 |
| 接收状态 | `ReceiveOnce()`（motor_receive 线程） | 2ms（500Hz） |
| 发送命令 | `SendOnce()`（motor_send 线程） | 2ms（500Hz） |
| 关闭 | `Stop()` | 一次 |

---

**更新时间：** 2026-09-29  
**版本：** 1.1
