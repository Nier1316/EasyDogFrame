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
| 1 | 单电机基本控制 |
| 2 | 同口多电机协同 |
| 3 | 跨 CAN 口控制 |
| 4 | 健康检查与故障处理 |
| 5 | 长时间运行（全 12 电机） |
| 6 | 原始 CAN 帧测试 |

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
- **CAN 帧编解码**：集中在 `Motor::EncodeCommand()` 和 `Motor::DecodeStatus()`，上层只操作 `MotorCommand` / `MotorStatus` 结构体。
- **CANET 库 API**：底层使用 `VCI_OpenDevice` / `VCI_InitCAN` / `VCI_StartCAN` / `VCI_Transmit` / `VCI_Receive`，封装在 `CanDevice` 内，上层不直接调用。

### Data flow

发送：`MotorController::MoveMotor()` → `MotorManager::SendMotorCommand()` → `Motor::EncodeCommand()` → `CanDevice::SendFrame()` → CANET 库  
接收：CANET 库 → `CanDevice::ReceiveFrames()` → `MotorManager::ProcessCanData()` → `Motor::DecodeStatus()` → `Motor::UpdateStatus()`

### All shared data types

`include/data_types.h` 是唯一的数据定义文件，所有层都依赖它：`MotorCommand`、`MotorStatus`、`CanDeviceConfig`、`MotorConfig`、`MotorCommandType`（枚举）、`ErrorCode`（枚举）。
