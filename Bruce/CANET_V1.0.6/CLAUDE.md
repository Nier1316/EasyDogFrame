# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

Outputs:
- `bin/motor_app` — demo executable
- `lib/libmotor_framework.a` — reusable static library

Debug build: replace `Release` with `Debug` (links `lib/linux_x64/Debug/libCANET_TCP.a` instead of Release variant).

## Run

```bash
./bin/motor_app <1-6>
```

| 编号 | 内容 |
|------|------|
| 1 | 单电机基本控制（三种模式：阻抗/速度/位置） |
| 2 | 同口多电机协同 |
| 3 | 跨 CAN 口控制 |
| 4 | 电机健康检查与故障处理 |
| 5 | 长时间运行（全 12 电机） |
| 6 | 原始 CAN 帧测试（仅依赖 BSP） |

## Architecture

Four-layer stack, each layer only depends on the one below:

```
MotorController   ← 应用门面，唯一推荐的应用层入口
      ↓
MotorManager      ← 单例，协调所有设备和电机；后台线程轮询接收帧
      ↓
Motor / CanDevice ← Motor 管理单个电机（编解码）；CanDevice 管理单个 CANET 设备（TCP 收发）
      ↓
CANET 库          ← 预编译静态库 lib/linux_x64/{Debug,Release}/libCANET_TCP.a
```

### Hardware topology

4 个 CANET 设备（can0~can3），每个设备挂 3 个电机（motor_id 1~3）：
- 发送 CAN ID = motor_id（1/2/3）
- 接收 CAN ID = 50 + motor_id（51/52/53）
- TCP 端口 = 4001 + device_idx（4001~4004），TCP_SERVER 模式

修改电机数量/CAN 口数量 → `src/motor_manager.cpp` 的 `CreateMotors()`  
修改 CAN ID 映射 → 同上，`rx_id` / `tx_id` 计算公式  
修改 TCP 端口 → `src/motor_controller.cpp` 的 `Initialize()`

### Key design decisions

- **MotorManager 是单例**：`MotorManager::GetInstance()`，整个进程只有一份电机拓扑视图。MotorController 持有其引用，不拥有其生命周期。
- **后台接收线程**：`MotorManager::ProcessCanData()` 在独立线程中轮询所有 CanDevice，收到帧后按 rx_id 分发到对应 Motor，调用 `Motor::DecodeStatus()` 更新状态。
- **线程安全**：Motor、CanDevice、MotorManager 各自持有 `std::mutex`，状态读写均加锁。
- **硬件协议完整支持**：仅支持硬件协议的功能集，不保留旧的简化 API：
  - **特殊指令**：使能/失能/置零/清错误/角度矫正
  - **参数指令**：读写电机参数
  - **控制指令**：三种模式（阻抗/速度/位置）的位域打包
- **CAN 帧编解码**：集中在 `Motor::EncodeCommand()` 和 `Motor::DecodeStatus()`，支持硬件协议位域打包：
  - **特殊指令**：`{0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, special_byte}`
  - **参数指令**：`{0x80, val[0], val[1], val[2], val[3], RW, param_type, 0xEC}`
  - **阻抗控制**：5参数位域打包（pos/vel/kp/kd/torque）
  - **速度控制**：vel/kp/ki 位域打包
  - **位置控制**：pos/kvp/kp/kd/kvi 位域打包
  - **状态解码**：按硬件实际 6 字节返回帧格式解析
- **CANET 库 API**：底层使用 `VCI_OpenDevice` / `VCI_InitCAN` / `VCI_StartCAN` / `VCI_Transmit` / `VCI_Receive`，封装在 `CanDevice` 内，上层不直接调用。

### Data flow

**发送流程**（支持硬件协议的完整功能）：
```
MotorController::EnableMotor/DisableMotor/SetMotorZero/ClearMotorError
MotorController::ImpedanceControl/SpeedControl/PositionControl
  ↓
MotorManager::SendMotorCommand()
  ↓
Motor::EncodeCommand()  ← 按 cmd_type 选择编码格式
  ├─ 特殊指令：{0x80, 0xFF, ..., special_byte}
  ├─ 参数指令：{0x80, val[0-3], RW, param_type, 0xEC}
  ├─ 阻抗控制：5参数位域打包
  ├─ 速度控制：vel/kp/ki 位域打包
  └─ 位置控制：pos/kvp/kp/kd/kvi 位域打包
  ↓
CanDevice::SendFrame()
  ↓
CANET 库 → CAN 总线 → 电机
```

**接收流程**（6字节硬件返回帧）：
```
电机 → CAN 总线
  ↓
CANET 库
  ↓
CanDevice::ReceiveFrames()
  ↓
MotorManager::ProcessCanData()
  ↓
Motor::DecodeStatus()  ← 按 6 字节硬件格式解析
  ├─ data[0]：ACK/FAULT/ENABLE/CAN_ID
  ├─ data[1-2]：position（16位）
  ├─ data[3-5]：velocity/torque（12位各）
  ↓
Motor::UpdateStatus()
  ↓
应用层读取 Motor::GetStatus()
```

### All shared data types

`include/data_types.h` 是唯一的数据定义文件，所有层都依赖它：
- `MotorCommand` — 支持硬件协议的完整功能（特殊指令、参数指令、三种控制模式）
- `MotorStatus` — 按硬件实际 6 字节返回帧格式
- `CanDeviceConfig`、`MotorConfig`
- `MotorCommandType` — 仅包含硬件协议支持的命令类型
- `ControlMode` — 三种控制模式（IMPEDANCE/SPEED/POSITION）
- `ErrorCode` — 错误码枚举

### API 接口

**MotorController 应用层接口**（推荐使用）：
- 特殊指令：`EnableMotor()` / `DisableMotor()` / `SetMotorZero()` / `ClearMotorError()`
- 控制指令：`ImpedanceControl()` / `SpeedControl()` / `PositionControl()`
- 状态查询：`GetMotorStatus()` / `PrintMotorStatus()` / `PrintAllMotorStatus()`
- 错误处理：`CheckMotorHealth()` / `HandleMotorError()`

**Motor 类接口**（内部使用）：
- 特殊指令：`Enable()` / `Disable()` / `SetZero()` / `ClearError()`
- 控制指令：`ImpedanceControl()` / `SpeedControl()` / `PositionControl()`
- 参数读写：`ReadParam()` / `WriteParam()`
- 编解码：`EncodeCommand()` / `DecodeStatus()`

