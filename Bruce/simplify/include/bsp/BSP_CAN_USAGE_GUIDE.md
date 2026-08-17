# BSP CAN 硬件抽象层使用指南

## 📋 目录

1. [概述](#概述)
2. [核心概念](#核心概念)
3. [API 参考](#api-参考)
4. [使用示例](#使用示例)
5. [常见问题](#常见问题)
6. [最佳实践](#最佳实践)

---

## 概述

**BspCan** 是 CAN 硬件抽象层（Board Support Package），为上层应用提供统一的 CAN 收发接口，隐藏 CANET 库的底层细节。

### 主要特性

- ✅ **单例模式** - 全局唯一实例，简化使用
- ✅ **线程安全** - 内部使用 mutex 保护，支持多线程并发访问
- ✅ **多设备支持** - 支持 0-3 号 CAN 设备
- ✅ **TCP 模式** - 支持客户端/服务器两种工作模式
- ✅ **格式转换** - 自动处理应用层和硬件层的格式转换
- ✅ **状态管理** - 完整的设备生命周期管理

### 架构图

```
┌─────────────────────────────────────┐
│      应用层（Application）           │
│  使用 BspCanFrame 和 BspCan API     │
└────────────────┬────────────────────┘
                 │
┌────────────────▼────────────────────┐
│    BSP 层（BspCan）                 │
│  - 设备管理                          │
│  - 格式转换                          │
│  - 线程安全                          │
└────────────────┬────────────────────┘
                 │
┌────────────────▼────────────────────┐
│   设备层（CanDevice）                │
│  - CANET 库封装                      │
│  - TCP 配置                          │
└────────────────┬────────────────────┘
                 │
┌────────────────▼────────────────────┐
│   硬件层（CANET 库）                 │
│  - CAN 硬件通信                      │
└─────────────────────────────────────┘
```

---

## 核心概念

### BspCanFrame 结构体

应用层统一的 CAN 帧表示，隐藏 VCI_CAN_OBJ 的复杂性。

```cpp
struct BspCanFrame {
    uint32_t id;           // CAN ID（0x000-0x7FF 标准帧，0x00000000-0x1FFFFFFF 扩展帧）
    uint8_t  dlc;          // 数据长度（0-8 字节）
    uint8_t  data[8];      // 数据区（最多 8 字节）
    uint8_t  is_extended;  // 帧类型（0=标准帧，1=扩展帧）
};
```

### CanDeviceConfig 结构体

CAN 设备配置参数。

```cpp
struct CanDeviceConfig {
    uint8_t     device_idx;    // 设备索引（0-3）
    uint16_t    port;          // TCP 端口
    const char* server_ip;     // 远端服务器 IP（客户端模式需要）
    uint8_t     work_mode;     // 工作模式（TCP_SERVER=1 / TCP_CLIENT=0）
};
```

### 设备生命周期

```
未初始化 → 初始化 → 启动 → 运行 → 停止 → 关闭 → 未初始化
```

---

## API 参考

### 1. 单例获取

#### `BspCan& BspCan::GetInstance()`

获取 BspCan 单例实例。

**返回值**：BspCan 单例引用

**示例**：
```cpp
BspCan& bsp = BspCan::GetInstance();
```

---

### 2. 设备生命周期管理

#### `bool InitDevice(uint8_t device_idx, const CanDeviceConfig& config)`

初始化指定 CAN 设备。

**参数**：
- `device_idx` - 设备索引（0-3）
- `config` - 设备配置

**返回值**：成功返回 true，失败返回 false

**示例**：
```cpp
CanDeviceConfig config;
config.device_idx = 0;
config.port = 4001;
config.server_ip = "192.168.0.178";
config.work_mode = TCP_CLIENT;

if (!bsp.InitDevice(0, config)) {
    printf("[ERROR] Failed to initialize device 0\n");
}
```

---

#### `bool StartDevice(uint8_t device_idx)`

启动 CAN 设备（使其能收发）。

**参数**：
- `device_idx` - 设备索引

**返回值**：成功返回 true，失败返回 false

**注意**：必须在 InitDevice 之后调用

**示例**：
```cpp
if (!bsp.StartDevice(0)) {
    printf("[ERROR] Failed to start device 0\n");
}
```

---

#### `bool StopDevice(uint8_t device_idx)`

停止 CAN 设备。

**参数**：
- `device_idx` - 设备索引

**返回值**：成功返回 true，失败返回 false

**示例**：
```cpp
bsp.StopDevice(0);
```

---

#### `bool CloseDevice(uint8_t device_idx)`

关闭 CAN 设备并释放资源。

**参数**：
- `device_idx` - 设备索引

**返回值**：成功返回 true，失败返回 false

**示例**：
```cpp
bsp.CloseDevice(0);
```

---

### 3. 数据收发

#### `bool SendFrame(uint8_t device_idx, const BspCanFrame& frame)`

发送单个 CAN 帧。

**参数**：
- `device_idx` - 目标设备索引
- `frame` - 帧数据

**返回值**：成功返回 true，失败返回 false

**示例**：
```cpp
BspCanFrame frame;
frame.id = 0x123;
frame.dlc = 8;
frame.is_extended = 0;
for (int i = 0; i < 8; i++) {
    frame.data[i] = i;
}

if (!bsp.SendFrame(0, frame)) {
    printf("[ERROR] Failed to send frame\n");
}
```

---

#### `bool Can_Tx(uint8_t device_idx, uint32_t can_id, const uint8_t* data, uint8_t dlc = 8)`

便捷发送函数：直接发送数据。

**参数**：
- `device_idx` - 目标设备索引
- `can_id` - CAN ID
- `data` - 数据指针（8 字节）
- `dlc` - 数据长度（默认 8）

**返回值**：成功返回 true，失败返回 false

**示例**：
```cpp
uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
bsp.Can_Tx(0, 0x123, data, 8);
```

---

#### `bool ReceiveFrames(uint8_t device_idx, std::vector<BspCanFrame>& frames, int timeout_ms = 100)`

批量接收 CAN 帧。

**参数**：
- `device_idx` - 源设备索引
- `frames` - 输出容器（调用者负责清空）
- `timeout_ms` - 阻塞等待时间（毫秒，默认 100）

**返回值**：有帧返回 true，无帧返回 false

**示例**：
```cpp
std::vector<BspCanFrame> frames;
if (bsp.ReceiveFrames(0, frames, 100)) {
    for (const auto& frame : frames) {
        printf("Received frame: ID=0x%03x, DLC=%d\n", frame.id, frame.dlc);
    }
}
```

---

### 4. 全局控制

#### `void ShutdownAll()`

关闭所有设备并释放资源。

**示例**：
```cpp
bsp.ShutdownAll();
```

---

## 使用示例

### 示例 1：基本的发送和接收

```cpp
#include "bsp/bsp_can.h"
#include <stdio.h>
#include <unistd.h>

int main() {
    BspCan& bsp = BspCan::GetInstance();
    
    // 1. 配置设备
    CanDeviceConfig config;
    config.device_idx = 0;
    config.port = 4001;
    config.server_ip = "192.168.0.178";
    config.work_mode = TCP_CLIENT;
    
    // 2. 初始化设备
    if (!bsp.InitDevice(0, config)) {
        printf("[ERROR] Failed to initialize device\n");
        return -1;
    }
    
    // 3. 启动设备
    if (!bsp.StartDevice(0)) {
        printf("[ERROR] Failed to start device\n");
        return -1;
    }
    
    printf("[INFO] Device initialized and started\n");
    
    // 4. 发送数据
    BspCanFrame frame;
    frame.id = 0x123;
    frame.dlc = 8;
    frame.is_extended = 0;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = i;
    }
    
    if (!bsp.SendFrame(0, frame)) {
        printf("[ERROR] Failed to send frame\n");
    } else {
        printf("[INFO] Frame sent successfully\n");
    }
    
    // 5. 接收数据
    sleep(1);
    std::vector<BspCanFrame> frames;
    if (bsp.ReceiveFrames(0, frames, 100)) {
        printf("[INFO] Received %zu frames\n", frames.size());
        for (const auto& rx_frame : frames) {
            printf("  ID=0x%03x, DLC=%d, Data=[", rx_frame.id, rx_frame.dlc);
            for (int i = 0; i < rx_frame.dlc; i++) {
                printf("%02x ", rx_frame.data[i]);
            }
            printf("]\n");
        }
    }
    
    // 6. 停止设备
    bsp.StopDevice(0);
    
    return 0;
}
```

---

### 示例 2：使用便捷发送函数

```cpp
#include "bsp/bsp_can.h"

int main() {
    BspCan& bsp = BspCan::GetInstance();
    
    // 初始化和启动设备（省略）
    // ...
    
    // 方式 1：发送 8 字节数据
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    bsp.Can_Tx(0, 0x123, data, 8);
    
    // 方式 2：发送 4 字节数据
    uint8_t short_data[4] = {0x01, 0x02, 0x03, 0x04};
    bsp.Can_Tx(0, 0x456, short_data, 4);
    
    return 0;
}
```

---

### 示例 3：多设备管理

```cpp
#include "bsp/bsp_can.h"

int main() {
    BspCan& bsp = BspCan::GetInstance();
    
    // 配置两个设备
    std::vector<CanDeviceConfig> configs(2);
    
    configs[0].device_idx = 0;
    configs[0].port = 4001;
    configs[0].server_ip = "192.168.0.178";
    configs[0].work_mode = TCP_CLIENT;
    
    configs[1].device_idx = 1;
    configs[1].port = 4002;
    configs[1].server_ip = "192.168.0.179";
    configs[1].work_mode = TCP_CLIENT;
    
    // 逐设备初始化
    for (const auto& cfg : configs) {
        if (!bsp.InitDevice(cfg.device_idx, cfg)) {
            printf("[ERROR] Failed to initialize device %d\n", cfg.device_idx);
            return -1;
        }
    }
    
    // 逐设备启动
    for (const auto& cfg : configs) {
        if (!bsp.StartDevice(cfg.device_idx)) {
            printf("[ERROR] Failed to start device %d\n", cfg.device_idx);
            return -1;
        }
    }
    
    // 在设备 0 上发送数据
    uint8_t data0[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    bsp.Can_Tx(0, 0x100, data0, 8);
    
    // 在设备 1 上发送数据
    uint8_t data1[8] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    bsp.Can_Tx(1, 0x200, data1, 8);
    
    // 逐设备停止
    for (const auto& cfg : configs) {
        bsp.StopDevice(cfg.device_idx);
    }
    
    return 0;
}
```

---

### 示例 4：电机控制集成

```cpp
#include "bsp/bsp_can.h"
#include "motor_drive/ele_motor.h"

int main() {
    BspCan& bsp = BspCan::GetInstance();
    
    // 初始化 CAN 设备
    CanDeviceConfig config;
    config.device_idx = 0;
    config.port = 4001;
    config.server_ip = "192.168.0.178";
    config.work_mode = TCP_CLIENT;
    
    bsp.InitDevice(0, config);
    bsp.StartDevice(0);
    
    // 发送电机参数读写命令
    float param_value = 0.0f;
    uint8_t rw = 1;  // 1=写，0=读
    uint8_t type = 0x01;  // 参数类型
    uint8_t motor_id = 0x01;
    
    float2bag(param_value, rw, type, motor_id);
    
    // 接收电机反馈
    std::vector<BspCanFrame> frames;
    if (bsp.ReceiveFrames(0, frames, 100)) {
        for (const auto& frame : frames) {
            // 处理电机反馈
            printf("Motor feedback: ID=0x%03x\n", frame.id);
        }
    }
    
    bsp.StopDevice(0);
    
    return 0;
}
```

---

## 常见问题

### Q1: 如何判断设备是否初始化成功？

**A**: 检查 `InitDevice()` 的返回值。如果返回 true，则初始化成功。

```cpp
if (bsp.InitDevice(0, config)) {
    printf("Device initialized successfully\n");
}
```

---

### Q2: 发送数据后多久能收到回复？

**A**: 这取决于网络延迟和设备响应时间。建议使用 `ReceiveFrames()` 的 `timeout_ms` 参数设置合理的等待时间。

```cpp
std::vector<BspCanFrame> frames;
bsp.ReceiveFrames(0, frames, 500);  // 等待 500ms
```

---

### Q3: 如何处理发送失败？

**A**: 检查 `SendFrame()` 或 `Can_Tx()` 的返回值。如果返回 false，可以重试或记录错误。

```cpp
if (!bsp.SendFrame(0, frame)) {
    printf("[ERROR] Failed to send frame, retrying...\n");
    // 重试逻辑
}
```

---

### Q4: 是否支持多线程并发访问？

**A**: 是的。BspCan 内部使用 mutex 保护，支持多线程安全访问。

```cpp
// 线程 1：发送数据
std::thread t1([&bsp]() {
    bsp.SendFrame(0, frame);
});

// 线程 2：接收数据
std::thread t2([&bsp]() {
    std::vector<BspCanFrame> frames;
    bsp.ReceiveFrames(0, frames, 100);
});

t1.join();
t2.join();
```

---

### Q5: 如何切换 TCP 工作模式？

**A**: 在 `CanDeviceConfig` 中设置 `work_mode` 字段。

```cpp
// 服务器模式
config.work_mode = TCP_SERVER;
config.port = 4001;
config.server_ip = nullptr;  // 服务器模式不需要 IP

// 客户端模式
config.work_mode = TCP_CLIENT;
config.port = 4001;
config.server_ip = "192.168.0.178";
```

---

## 最佳实践

### 1. 错误处理

始终检查 API 的返回值，并进行适当的错误处理。

```cpp
if (!bsp.InitDevice(0, config)) {
    printf("[ERROR] Failed to initialize device\n");
    return -1;
}

if (!bsp.StartDevice(0)) {
    printf("[ERROR] Failed to start device\n");
    bsp.CloseDevice(0);
    return -1;
}
```

---

### 2. 资源清理

确保在程序退出前关闭所有设备。

```cpp
// 方式 1：逐个关闭
bsp.StopDevice(0);
bsp.CloseDevice(0);

// 方式 2：全部关闭
bsp.ShutdownAll();
```

---

### 3. 数据验证

在发送前验证数据的有效性。

```cpp
BspCanFrame frame;
frame.id = 0x123;
frame.dlc = 8;  // 必须 0-8

// 验证
if (frame.dlc > 8) {
    printf("[ERROR] Invalid DLC\n");
    return -1;
}

bsp.SendFrame(0, frame);
```

---

### 4. 超时设置

根据实际需求设置合理的超时时间。

```cpp
// 快速响应场景：100ms
bsp.ReceiveFrames(0, frames, 100);

// 慢速设备：1000ms
bsp.ReceiveFrames(0, frames, 1000);
```

---

### 5. 性能优化

- 使用 `Can_Tx()` 而不是 `SendFrame()` 来简化代码
- 批量接收多个帧而不是逐个接收
- 定期查询统计信息以监控性能

```cpp
// 高效的接收方式
std::vector<BspCanFrame> frames;
if (bsp.ReceiveFrames(0, frames, 100)) {
    for (const auto& frame : frames) {
        // 处理多个帧
    }
}
```

---

## 总结

BspCan 提供了一个简洁、安全、高效的 CAN 通信接口。通过遵循本指南的最佳实践，你可以快速集成 CAN 功能到你的应用中。

如有问题，请参考源代码注释或联系开发团队。
