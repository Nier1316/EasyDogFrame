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
- 管理 4 路 CAN 传输（CANET TCP / 达妙 USB2CAN，CAN0~CAN3）
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

TCP: 192.168.0.178, 端口 4001~4004
```

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

### 步骤 1：获取单例

```cpp
#include "motor_manager.h"

MotorManager& motor_mgr = MotorManager::GetInstance();
```

### 步骤 2：初始化（需要 ThreadManager）

```cpp
#include "thread/thread_manager.h"

ThreadManager thread_mgr;

// 初始化 MotorManager（会向 thread_mgr 注册 motor_receive 和 motor_send 线程）
if (!motor_mgr.Initialize(thread_mgr)) {
    printf("[ERROR] Failed to initialize MotorManager\n");
    return false;
}

// 启动已注册的线程
thread_mgr.start_thread("motor_receive");  // 后台接收线程
thread_mgr.start_thread("motor_send");     // 后台发送线程
```

### 步骤 3：关闭

```cpp
// 停止线程
thread_mgr.stop_thread("motor_receive");
thread_mgr.stop_thread("motor_send");

// 关闭 MotorManager
motor_mgr.Stop();
```

---

## API 参考

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
- `motor_id` (uint8_t): 电机 ID [1, 3]

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
- `torque`: 腿 ±150 / 轮 ±52 Nm

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

**返回值：** `MotorStatus` 结构体
```cpp
struct MotorStatus {
    uint8_t motor_id;       // 电机 ID
    bool ack;               // 收到指令
    bool fault;             // 驱动错误
    bool enable;            // 使能状态
    float position;         // 当前位置 (rad)
    float velocity;         // 当前速度 (rad/s)
    float torque;           // 当前扭矩 (Nm)
    uint8_t error_code;     // 错误码
};
```

---

## 工作流程

### 完整的控制流程

```
┌─────────────────────────────────────────────────────────┐
│ 应用层                                                  │
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
                 ↓ (2ms 周期)
┌─────────────────────────────────────────────────────────┐
│ SendThreadFunc()                                        │
│ - 遍历所有 enabled 电机                                 │
│ - 按 control_mode 调用 set_motor_para_bt()              │
│ - 编码 CAN 帧并发送                                     │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────────────────────┐
│ BspCan::Can_Tx()                                        │
│ - 通过 CANET 发送到电机                                 │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────────────────────┐
│ 电机执行命令                                            │
│ - 按阻抗模式控制                                        │
│ - 返回状态帧                                            │
└────────────────┬────────────────────────────────────────┘
                 │
                 ↓ (2ms 周期)
┌─────────────────────────────────────────────────────────┐
│ ReceiveThreadFunc()                                     │
│ - 轮询 4 路 CAN 口                                      │
│ - 接收电机状态帧 (ID: 51-53)                            │
│ - 调用 unpack_frame() 解包                              │
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
#include "motor_manager.h"
#include "thread/thread_manager.h"

int main() {
    // 初始化
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize\n");
        return -1;
    }
    
    // 启动线程
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    
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

### Q2：SendThreadFunc 何时被调用？

**A：** SendThreadFunc 由外部 ThreadManager 以 2ms 周期驱动，不需要手动调用。

```cpp
// Initialize() 中已注册
thread_mgr.register_thread(
    "motor_send",
    [this]() { SendThreadFunc(); },
    ThreadMode::LOOP, 2, 80  // 2ms 间隔
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
| 扭矩 (torque) | ±18 | Nm |
| Kp | 0~500 | — |
| Kd | 0~5 | — |
| Ki | 0~500 | — |

---

### Q5：如何调试解包过程？

**A：** 在 ReceiveThreadFunc() 中添加日志。

```cpp
// 在 unpack_frame() 后添加
printf("[DEBUG] Motor %d:%d - Pos: %.2f, Vel: %.2f, Torque: %.2f\n",
       can_port, motor_id,
       motor.current_position, motor.current_speed, motor.current_torque);
```

---

### Q6：线程安全性如何保证？

**A：** 每个电机独立互斥锁，接收线程和应用线程互斥访问。

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
| 使能/禁用 | `EnableMotor/DisableMotor` | 按需 |
| 控制 | `SendImpedance/Speed/Position` | 按需 |
| 特殊操作 | `SetZero/ClearError` | 按需 |
| 状态查询 | `GetStatus()` | 按需 |
| 接收状态 | `ReceiveThreadFunc()` | 1ms |
| 发送命令 | `SendThreadFunc()` | 1ms |
| 关闭 | `Stop()` | 一次 |

---

**更新时间：** 2026-05-24  
**版本：** 1.0
