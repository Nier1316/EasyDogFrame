# 四足机器狗电机控制框架 — 完整使用指南

> # ⚠️ 过时横幅（2026-09-29 更新）
>
> **本文的「架构设计」「核心模块」「API 文档」「使用示例」「文件结构」等章节，以及文中所有示例代码片段，
> 描述的是 2026-08 重构前的历史结构，仅作历史存档参考，不再代表当前工程。**
>
> 现工程已按 **L0~L7 分层**重组：
> `common`(L0) → `transport`(L1) → `motor`(L2/L3) → `runtime`(L4) → `motion`(L5) → `strategy`(L6) → `app`(L7)。
>
> - **现状真源**：`docs/SIM2REAL_DEPLOY.md`、`memory/FACT.md`（与之冲突时以这两个文件与当前代码为准）。
> - 🔧 **要在真机上跑测试/采数据**：先读 **`docs/REAL_ROBOT_HANDOVER.md`**（真机交接文档：安全铁律 + T0~T8 执行清单 + 回传报告模板）。
> - 重构前的旧路径 / 旧类名，见下方「已失效名称 → 当前实现」对照表。
> - **仍然有效**：第 8 章「编译、运行和调试」与第 9 章「调试指南」（GDB / Valgrind / AddressSanitizer /
>   perf / strace / 网络配置等通用内容）不随重构失效，请继续参考。

### 🔴 已失效名称 → 当前实现（速查对照）

| 历史写法（本文中） | 状态 | 当前实现 |
|---|---|---|
| `BspCan` / `bsp/bsp_can.h` / `BspCanFrame` | 🔴 已删除 | L1 `CanTransport` 纯虚接口（`include/transport/can_transport.h`）+ `CanetTransport` / `Usb2CanTransport` 两个后端；`BspCanFrame` → `CanFrame`（`include/common/types.h`）。**默认后端 = 达妙 USB2CAN（`Usb2CanTransport`）** |
| `include/data_types.h` | 🔴 已移动 | `include/common/types.h` |
| `include/motor_manager.h` | 🔴 已移动 | `include/motor/motor_manager.h` |
| `include/thread/thread_manager.h` | 🔴 已移动 | `include/runtime/thread_manager.h` |
| `include/motor_calibration.h` | 🔴 已移动 | `include/motor/motor_calibration.h` |
| `include/motor_drive/ele_motor.h` | 🔴 已移动 | `include/motor/ele_motor.h` |
| `include/leg_kinematics.h` / `include/SimSync.h` | 🔴 已移动 | `include/motion/leg_kinematics.h` / `include/motion/SimSync.h` |
| `src/base/`（厂商 common / crc16 / log / jsoncpp / md5 / network） | 🔴 已删除 | 目录已不存在；第「文件结构」中的该子树仅为历史存档 |
| `RobotDog` / `robot_dog.h` | 🔴 从未实现 | 全仓无该类/文件；四足语义由 `motion` 层与各示例直接调用 `MotorManager` 承担 |
| CANET TCP 配置（`192.168.0.178`，端口 4001~4004） | ⚠️ 已弃用 | **仅 Example27 / Example28 仍直连 `CanetTransport` 做探针**；正式控制默认走达妙 USB2CAN |
| `main.cpp` 示例编号 `Example9 / 11 / 19 / 21 / 44` | 🔴 已清理或改号 | 现存 **17~57 共 40 个**（编号不连续）；**当前启用 `Example37_RLTeleopControl`** |
| `motor_receive` / `motor_send` 节拍 1ms / 10ms | ⚠️ 过时 | 各 **2ms（500Hz）**，SCHED_FIFO 优先级 **80** |

## 目录
1. [框架概述](#框架概述)
2. [架构设计](#架构设计)
3. [硬件拓扑](#硬件拓扑)
4. [快速开始](#快速开始)
5. [核心模块](#核心模块)
6. [API 文档](#api-文档)
7. [使用示例](#使用示例)
8. [编译、运行和调试](#编译运行和调试)
9. [调试指南](#调试指南)
10. [常见问题](#常见问题)

---

## 框架概述

本框架是一个**分层的四足机器狗电机控制系统**，通过 CAN 总线与 16 个电机通信，支持多种控制模式（阻抗控制、速度控制、位置控制）。

### 核心特性
- **16 电机管理**：4 路 CAN 口，每路 4 个电机（髋、大腿、小腿、轮）
- **多线程架构**：ThreadManager 统一管理所有后台线程（收 + 发双线程）
- **线程安全**：每个电机配备独立互斥锁，支持并发访问
- **灵活的控制模式**：支持阻抗、速度、位置三种控制方式
- **实时优先级**：支持 Linux SCHED_FIFO 实时调度

---

## 架构设计

### 分层结构

> 🗄️ **历史存档（重构前，2026-08 之前）** —— 下图中的 `RobotApp` / `CanDevice` / `BspCan` / CANET
> 组合已不再是当前结构，仅用于理解演进过程。

```
┌─────────────────────────────────────────┐
│         应用层 (RobotApp)               │  用户应用
├─────────────────────────────────────────┤
│    管理层 (MotorManager + ThreadManager) │  电机管理 + 线程管理
├─────────────────────────────────────────┤
│    设备层 (CanDevice + EleMotor)        │  CAN设备 + 单电机
├─────────────────────────────────────────┤
│    BSP层 (BspCan)           🔴 已删除    │  硬件抽象 → 现为 CanTransport
├─────────────────────────────────────────┤
│    硬件 (CANET + 电机)                   │  物理设备（现默认达妙 USB2CAN）
└─────────────────────────────────────────┘
```

**当前分层（L0~L7，以代码为准）**：

```
L7 app        main.cpp + 示例 ex_basic/ex_diag/ex_rl + 示例公共 helper
L6 strategy   观测构建 + PD 律 + 关节序映射、真机↔URDF 转换、IMU、手写 MLP
L5 motion     起立/趴下/回位/RL 单步、腿运动学、整机参数
L4 runtime    ThreadManager（ONCE/LOOP + SCHED_FIFO）、RobotApp、线程注册
L3 motor      MotorManager 单例：16 电机、三模式、500Hz SendOnce、轮控急停
L2 motor      EleMotor 单电机结构、MIT 协议编解码、按电机类型量程
L1 transport  CanTransport 纯虚接口 + CanetTransport(TCP) / Usb2CanTransport(达妙)
L0 common     CanFrame/电机状态类型、共享数据区、日志分类开关、CSV/RL 遥测记录
```

### 模块职责

> 🗄️ 下表为**重构前**的模块划分。若要在现工程中定位，请按「当前路径」一列；L0~L7 分层见上。

| 模块 | 历史文件（本文语境） | 当前路径 | 职责 |
|------|------|------|------|
| **ThreadManager** | `include/thread/thread_manager.h` | `include/runtime/thread_manager.h` | 线程生命周期管理、共享数据区 |
| **CanDevice** | `include/transport/can_device.h` | `include/transport/can_device.h`（仍存，CANET 专用） | CANET 设备封装、TCP 连接管理 |
| **EleMotor** | `include/motor_drive/ele_motor.h` | `include/motor/ele_motor.h` | 单电机数据结构、状态管理 |
| **MotorManager** | `include/motor_manager.h` | `include/motor/motor_manager.h` | 16 电机批量管理、命令分发 |
| **CanTransport** | `include/bsp/bsp_can.h` 🔴 | `include/transport/can_transport.h` + `canet_transport.h` / `usb2can_transport.h` | 传输抽象（`BspCan` 已删，重构为 `CanTransport` 子类，默认 `Usb2CanTransport`） |
| **DataTypes** | `include/data_types.h` 🔴 | `include/common/types.h` | 通用数据结构定义（`CanFrame` / `MotorStatus` / `TransportConfig`） |

---

## 硬件拓扑

### CAN 口分配

```
CAN0 → 左前腿 (FL)
  ├─ motor_id=1 → 髋关节 (Hip)
  ├─ motor_id=2 → 大腿 (Thigh)
  ├─ motor_id=3 → 小腿 (Calf)
  └─ motor_id=4 → 轮 (Wheel)

CAN1 → 右前腿 (FR)
  ├─ motor_id=1 → 髋关节
  ├─ motor_id=2 → 大腿
  ├─ motor_id=3 → 小腿
  └─ motor_id=4 → 轮

CAN2 → 左后腿 (RL)
  ├─ motor_id=1 → 髋关节
  ├─ motor_id=2 → 大腿
  ├─ motor_id=3 → 小腿
  └─ motor_id=4 → 轮

CAN3 → 右后腿 (RR)
  ├─ motor_id=1 → 髋关节
  ├─ motor_id=2 → 大腿
  ├─ motor_id=3 → 小腿
  └─ motor_id=4 → 轮
```

> 说明：每路 CAN 挂载 4 个电机（3 关节 + 1 轮），常量定义见
> `include/motor/motor_calibration.h`（历史路径 `include/motor_calibration.h`）：`CAN_PORTS = 4`、
> `MOTORS_PER_CAN = 4`，共 16 个电机。轮电机 `motor_id = 4`，`motor_id` 取值 [1,4]。

### CAN 帧 ID 映射

| 方向 | CAN ID | 说明 |
|------|--------|------|
| 上位机 → 电机 | 1, 2, 3, 4 | 对应 motor_id |
| 电机 → 上位机 | 51, 52, 53, 54 | 50 + motor_id |

### TCP 连接参数

> ## ⚠️ 本节整体已弃用（2026-09-29 标注）
>
> CANET TCP 后端（`CanetTransport`）**已不作为正式控制链路**，`MotorManager::Initialize()`
> 在未注入传输后端时统一兜底为 **达妙 USB2CAN（`Usb2CanTransport`）**
> （见 `src/motor/motor_manager.cpp` 的兜底分支，日志打印
> `使用默认传输后端 Usb2CanTransport (达妙 USB2CAN)`）。
>
> 现在**仅 Example27（CANET 接收频率探针）与 Example28（CANET 批量发送探针）** 仍直接使用
> `CanetTransport`，用于链路诊断。换后端请用
> `MotorManager::SetTransport()` / `SetChannelTransport()` 注入 `CanTransport*`。
>
> 以下 TCP 配置内容**仅对 Example27/28 或历史代码有效**，保留供诊断时参考。

```
服务器地址：192.168.0.178
端口分配：
  - CAN0: 4001
  - CAN1: 4002
  - CAN2: 4003
  - CAN3: 4004
```

### TCP 端口配置方法

#### 1. 网络配置诊断

在配置 TCP 连接前，需要确保网络连接正常：

```bash
# 查看网络接口配置
ip addr show

# 测试能否 ping 通目标设备
ping -c 3 192.168.0.178

# 查看路由表
ip route show

# 查看监听的端口
ss -tuln | grep LISTEN
```

#### 2. 修改有线网络 IP 地址

如果你的网络 IP 与 CAN 设备不在同一网段，需要修改网络配置：

**查看当前网络连接**：
```bash
nmcli connection show --active
```

**修改有线网络 IP 地址**：
```bash
# 修改 IP 地址
nmcli connection modify "有线连接 1" ipv4.addresses 192.168.0.100/24

# 修改网关
nmcli connection modify "有线连接 1" ipv4.gateway 192.168.0.1

# 重新启动网络连接
nmcli connection up "有线连接 1"

# 验证修改
ip addr show enp45s0
ping -c 3 192.168.0.178
```

**参数说明**：
- `ipv4.addresses` — IP 地址和子网掩码（/24 表示 255.255.255.0）
- `ipv4.gateway` — 网关地址
- `"有线连接 1"` — 网络连接名称（可通过 `nmcli connection show` 查看）

#### 3. 在代码中配置 TCP 端口

> 🔴 **已失效**：下表代码使用已删除的 `BspCan` 单例与 `#include "bsp/bsp_can.h"`，
> 现工程无法编译。当前等价做法是构造 `TransportConfig` 后调用
> `CanTransport::open(idx, cfg)`，或经 `MotorManager::SetTransport()` 注入。
> `BspCanFrame` → `CanFrame`，`device_idx` → `TransportConfig::device_idx`，
> `port` / `server_ip` / `work_mode` → `TransportConfig::tcp_port / tcp_ip / tcp_mode`。仅存档。

**基本配置（🔴 历史代码，不可编译）**：
```cpp
#include "bsp/bsp_can.h"

int main() {
    BspCan& bsp = BspCan::GetInstance();
    
    // 配置 CAN0 设备
    CanDeviceConfig config;
    config.device_idx = 0;              // 设备索引（0-3）
    config.port = 4001;                 // TCP 端口
    config.server_ip = "192.168.0.178"; // 服务器 IP
    config.work_mode = TCP_CLIENT;      // 工作模式（客户端）
    
    // 初始化设备
    if (!bsp.InitDevice(0, config)) {
        printf("[ERROR] Failed to initialize device 0\n");
        return -1;
    }
    
    // 启动设备
    if (!bsp.StartDevice(0)) {
        printf("[ERROR] Failed to start device 0\n");
        return -1;
    }
    
    printf("[INFO] Device 0 connected successfully\n");
    
    // ... 使用设备 ...
    
    bsp.StopDevice(0);
    return 0;
}
```

**多设备配置**：
```cpp
// 配置所有 4 个 CAN 设备
std::vector<CanDeviceConfig> configs(4);

for (int i = 0; i < 4; i++) {
    configs[i].device_idx = i;
    configs[i].port = 4001 + i;         // 端口：4001, 4002, 4003, 4004
    configs[i].server_ip = "192.168.0.178";
    configs[i].work_mode = TCP_CLIENT;
}

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
```

#### 4. TCP 工作模式

**客户端模式**（推荐用于上位机）：
```cpp
config.work_mode = TCP_CLIENT;
config.server_ip = "192.168.0.178";  // 远端服务器 IP
config.port = 4001;                  // 远端服务器端口
```

**服务器模式**（用于设备作为服务器）：
```cpp
config.work_mode = TCP_SERVER;
config.server_ip = nullptr;          // 服务器模式不需要 IP
config.port = 4001;                  // 本机监听端口
```

#### 5. 常见问题排查

**问题 1：无法 ping 通目标 IP**

```bash
# 检查网络接口是否启用
ip link show

# 检查 IP 地址是否正确
ip addr show

# 检查路由是否正确
ip route show

# 尝试修改网络 IP 到同一网段
nmcli connection modify "有线连接 1" ipv4.addresses 192.168.0.100/24
nmcli connection up "有线连接 1"
```

**问题 2：端口被占用**

```bash
# 查看端口是否被占用
ss -tuln | grep 4001

# 如果被占用，可以修改端口号
config.port = 4005;  // 改用其他端口
```

**问题 3：防火墙阻止连接**

```bash
# 查看防火墙状态
sudo ufw status

# 允许特定端口
sudo ufw allow 4001:4004/tcp

# 重启防火墙
sudo ufw reload
```

#### 6. 验证连接

```cpp
// 发送测试帧
BspCanFrame frame;
frame.id = 0x123;
frame.dlc = 8;
frame.is_extended = 0;
for (int i = 0; i < 8; i++) {
    frame.data[i] = i;
}

if (bsp.SendFrame(0, frame)) {
    printf("[INFO] Frame sent successfully\n");
} else {
    printf("[ERROR] Failed to send frame\n");
}

// 接收测试帧
std::vector<BspCanFrame> frames;
if (bsp.ReceiveFrames(0, frames, 1000)) {
    printf("[INFO] Received %zu frames\n", frames.size());
} else {
    printf("[INFO] No frames received (timeout)\n");
}
```

---

## 快速开始

### 1. 编译项目

**依赖前提**：

- CMake ≥ 3.8、支持 C++17 的编译器（gcc/g++）
- **SDL2 开发库**：Example21（Xbox 手柄控制）依赖 SDL2。由于 `main.cpp` 当前启用的 **Example37_RLTeleopControl**（USB2CAN 手柄遥操作）也依赖 SDL2，缺少 SDL2 会导致编译失败
  （`fatal error: SDL2/SDL.h: No such file or directory`）。安装：

  ```bash
  sudo apt-get install libsdl2-dev
  ```

  若不需要手柄控制，可在 `main.cpp` 中改用其它不依赖 SDL2 的示例，即可跳过此依赖。

```bash
cd /home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### 2. 初始化框架

```cpp
#include "motor/motor_manager.h"          // 历史路径: "motor_manager.h"
#include "runtime/thread_manager.h"       // 历史路径: "thread/thread_manager.h"

int main() {
    // 创建线程管理器
    ThreadManager thread_mgr;
    
    // 初始化电机管理器
    MotorManager& motor_mgr = MotorManager::GetInstance();
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("Failed to initialize MotorManager\n");
        return -1;
    }
    // Initialize() 内部已经通过 RegisterMotorIoThreads() 注册好
    // motor_receive / motor_send 两个 LOOP 线程（各 2ms=500Hz，优先级 80）。
    
    // 启动收发线程（各 2ms 间隔，500Hz）
    // 注意：必须同时启动 motor_receive 和 motor_send，
    //       只启动接收线程时，控制命令不会被下发到电机
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    
    // ... 控制电机 ...
    
    // 清理资源
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    
    return 0;
}
```

### 3. 控制电机

```cpp
// 使能电机
motor_mgr.EnableMotor(0, 1);  // CAN0, motor_id=1

// 发送阻抗控制命令
motor_mgr.SendImpedance(
    0, 1,           // CAN0, motor_id=1
    0.0f,           // 目标位置 (rad)
    0.0f,           // 目标速度 (rad/s)
    10.0f,          // 刚度系数 Kp
    1.0f,           // 阻尼系数 Kd
    0.0f            // 扭矩前馈 (Nm)
);

// 读取电机状态
MotorStatus status = motor_mgr.GetStatus(0, 1);
printf("Position: %.2f rad, Velocity: %.2f rad/s, Torque: %.2f Nm\n",
       status.position, status.velocity, status.torque);
```

---

## 核心模块

### ThreadManager — 线程管理

**职责**：统一管理所有后台线程的生命周期

**关键特性**：
- 支持两种执行模式：ONCE（执行一次）、LOOP（循环执行）
- 线程间共享数据区（SharedData）
- 支持 Linux 实时优先级设置（SCHED_FIFO）
- 自动清理：析构时自动停止并 join 所有线程

**使用流程**：
```cpp
ThreadManager mgr;

// 1. 注册线程
mgr.register_thread(
    "receive",              // 线程名称
    []() { /* 任务函数 */ },
    ThreadMode::LOOP,       // 循环模式
    2,                      // 2ms 间隔
    0                       // 普通优先级
);

// 2. 启动线程
mgr.start_thread("receive");

// 3. 线程间通信
mgr.get_shared_data().set<float>("speed", 100.0f);
float speed = mgr.get_shared_data().get<float>("speed");

// 4. 停止线程
mgr.stop_thread("receive");
```

### CanDevice — CAN 设备管理

> ⚠️ **CANET 专用、已弃用为正式控制链路**：`CanDevice` 仍存在于
> `include/transport/can_device.h`，但只被 `CanetTransport` 使用。
> 现在正式控制默认走达妙 USB2CAN（`Usb2CanTransport`），本节的配置方式
> 仅对 Example27/28 或历史代码有效。

**职责**：封装 CANET 库，提供面向对象的 CAN 设备接口

**关键特性**：
- 自动 TCP 连接管理
- 线程安全的收发接口
- 设备状态查询

**使用流程**：
```cpp
#include "can_device.h"

CanDevice can_dev(0);  // CAN0

// 配置设备
CanDeviceConfig config;
config.device_idx = 0;
config.port = 4001;
config.server_ip = "192.168.0.178";
config.work_mode = TCP_CLIENT;

// 初始化
if (!can_dev.Initialize(config)) {
    printf("Failed to initialize CAN device\n");
    return -1;
}

// 启动 CAN 通道
can_dev.Start();

// 发送帧
VCI_CAN_OBJ frame;
frame.ID = 1;
frame.DLC = 8;
// ... 填充数据 ...
can_dev.SendFrame(frame);

// 接收帧
std::vector<VCI_CAN_OBJ> frames;
can_dev.ReceiveFrames(frames, 100);  // 100ms 超时

// 清理
can_dev.Stop();
can_dev.Shutdown();
```

### MotorManager — 电机管理

**职责**：管理 16 个电机，提供统一的控制接口

**关键特性**：
- 单例模式
- 自动后台接收线程
- 每个电机独立互斥锁
- 支持多种控制模式

**对外接口**：

```cpp
class MotorManager {
public:
    // 生命周期
    static MotorManager& GetInstance();
    bool Initialize(ThreadManager& thread_mgr);
    void Stop();
    
    // 电机控制
    void EnableMotor(uint8_t can_port, uint8_t motor_id);
    void DisableMotor(uint8_t can_port, uint8_t motor_id);
    void SetZero(uint8_t can_port, uint8_t motor_id);
    void ClearError(uint8_t can_port, uint8_t motor_id);
    
    // 控制命令
    void SendImpedance(uint8_t can_port, uint8_t motor_id,
                       float pos, float vel, float kp, float kd, float torque);
    void SendSpeed(uint8_t can_port, uint8_t motor_id,
                   float vel, float kp, float ki);
    void SendPosition(uint8_t can_port, uint8_t motor_id,
                      float pos, float kvp, float kp, float kd, float kvi);
    
    // 状态查询
    MotorStatus GetStatus(uint8_t can_port, uint8_t motor_id) const;
};
```

---

## API 文档

### 数据结构

#### MotorStatus — 电机状态

```cpp
struct MotorStatus {
    uint8_t motor_id;   // 电机 ID
    bool    enable;     // 使能状态（应用层命令态）
    float   position;   // 当前位置 (rad)
    float   velocity;   // 当前速度 (rad/s)
    float   torque;     // 当前扭矩 (Nm)
    uint8_t error_code; // 错误码（非 0 即故障）
};
```

#### ControlMode — 控制模式

```cpp
enum ControlMode {
    IMPEDANCE = 0,  // 阻抗控制（位置+速度+力）
    SPEED     = 1,  // 速度控制
    POSITION  = 2   // 位置控制
};
```

### 参数范围

| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| 位置 (pos) | ±12.5 | rad | 关节角度（固件实测量程） |
| 速度 (vel) | 关节 ±3 / 轮 ±48 | rad/s | 角速度（固件实测量程） |
| 扭矩 (torque) | 髋/大腿 ±120、小腿 ±200 / 轮 ±52 | Nm | 命令扭矩 clamp，见 `include/motor/ele_motor_def.h` 的 `TORQUE_CMD_LIMIT` |
| Kp | 0~500 | - | 刚度/位置环比例系数 |
| Kd | 0~100 | - | 阻尼系数 |
| Ki | 0~500 | - | 速度环积分系数 |

---

## 使用示例

> ⚠️ **注意编号歧义**：本节「示例 1/2/3/4」是本文自编的教学代码片段，
> **与工程内 `src/app/examples/` 的 `Example17~Example57` 毫无对应关系**。
> 工程内现存示例共 **40 个**（编号 17~57，不连续），切换方式见第 8 章 2.3 节。
> 实际可运行的示例清单与定位请以 `docs/SIM2REAL_DEPLOY.md`、`memory/FACT.md` 为准。

### 示例 1：基础电机控制

```cpp
#include "motor/motor_manager.h"       // 历史路径: "motor_manager.h"
#include "runtime/thread_manager.h"    // 历史路径: "thread/thread_manager.h"
#include <unistd.h>

int main() {
    ThreadManager thread_mgr;
    MotorManager& motor_mgr = MotorManager::GetInstance();
    
    // 初始化
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("Initialize failed\n");
        return -1;
    }
    
    // 启动收发线程
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    
    // 使能 CAN0 上的电机 1
    motor_mgr.EnableMotor(0, 1);
    sleep(1);
    
    // 发送阻抗控制命令（归零）
    motor_mgr.SendImpedance(0, 1, 0.0f, 0.0f, 10.0f, 1.0f, 0.0f);
    
    // 循环读取状态
    for (int i = 0; i < 10; i++) {
        MotorStatus status = motor_mgr.GetStatus(0, 1);
        printf("Motor 1: pos=%.2f, vel=%.2f, torque=%.2f\n",
               status.position, status.velocity, status.torque);
        sleep(1);
    }
    
    // 禁用电机
    motor_mgr.DisableMotor(0, 1);
    
    // 清理
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    
    return 0;
}
```

### 示例 2：多电机同时控制

```cpp
// 使能所有 16 个电机（每路 motor_id 1~4）
for (int can_port = 0; can_port < 4; can_port++) {
    for (int motor_id = 1; motor_id <= 4; motor_id++) {
        motor_mgr.EnableMotor(can_port, motor_id);
    }
}

sleep(1);

// 所有电机归零
for (int can_port = 0; can_port < 4; can_port++) {
    for (int motor_id = 1; motor_id <= 4; motor_id++) {
        motor_mgr.SendImpedance(can_port, motor_id,
                                0.0f, 0.0f, 10.0f, 1.0f, 0.0f);
    }
}
```

### 示例 3：速度控制

```cpp
// 电机以 10 rad/s 的速度旋转
motor_mgr.SendSpeed(0, 1, 10.0f, 50.0f, 10.0f);
// 参数：CAN0, motor_id=1, 速度=10 rad/s, Kp=50, Ki=10
```

### 示例 4：位置控制

```cpp
// 电机移动到 1.57 rad（π/2）
motor_mgr.SendPosition(0, 1, 1.57f, 20.0f, 50.0f, 1.0f, 10.0f);
// 参数：CAN0, motor_id=1, 位置=1.57 rad, 
//      位置环Kp=20, 速度环Kp=50, 位置环Kd=1.0, 速度环Ki=10
```

---

## 调试指南

### 1. 调试工作流

#### 1.1 快速调试流程

```bash
# 1. 用 Debug 模式编译
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)

# 2. 启动 GDB
gdb ./bin/can_motor_app

# 3. 在 GDB 中设置断点并运行
(gdb) break main
(gdb) run

# 4. 调试完成后退出
(gdb) quit
```

#### 1.2 调试常见问题

**问题**：程序崩溃，需要找到崩溃位置

```bash
# 1. 用 Debug 模式编译
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)

# 2. 启动 GDB 并运行程序
gdb ./bin/can_motor_app

# 3. 在 GDB 中运行程序
(gdb) run

# 4. 程序崩溃后，查看调用栈
(gdb) backtrace
(gdb) frame 0  # 查看第一帧
(gdb) info locals  # 查看本地变量
(gdb) print variable_name  # 打印变量值
```

### 2. GDB 详细使用指南

#### 2.1 常用 GDB 命令

```bash
# 启动 GDB
gdb ./bin/can_motor_app

# 在 GDB 中的命令：

# 断点管理
(gdb) break main                    # 在 main 函数设置断点
(gdb) break motor_manager.cpp:100   # 在指定文件的第 100 行设置断点
(gdb) break MotorManager::Initialize  # 在成员函数设置断点
(gdb) info breakpoints              # 查看所有断点
(gdb) delete 1                      # 删除第 1 个断点
(gdb) disable 1                     # 禁用第 1 个断点
(gdb) enable 1                      # 启用第 1 个断点
(gdb) clear motor_manager.cpp:100   # 清除指定位置的断点

# 程序执行
(gdb) run                           # 运行程序
(gdb) run arg1 arg2                 # 带参数运行程序
(gdb) continue                      # 继续执行
(gdb) next                          # 单步执行（不进入函数）
(gdb) step                          # 单步执行（进入函数）
(gdb) finish                        # 执行到函数返回
(gdb) until 100                     # 执行到第 100 行

# 变量和内存检查
(gdb) print variable_name           # 打印变量值
(gdb) print &variable_name          # 打印变量地址
(gdb) print *pointer                # 打印指针指向的值
(gdb) print array[0]@10             # 打印数组的前 10 个元素
(gdb) info locals                   # 查看所有本地变量
(gdb) info args                     # 查看函数参数
(gdb) watch variable_name           # 监视变量（变量改变时停止）
(gdb) x/10x 0x7fff0000              # 查看内存（16进制格式）

# 调用栈
(gdb) backtrace                     # 打印完整调用栈
(gdb) frame 0                       # 切换到第 0 帧
(gdb) up                            # 切换到上一帧
(gdb) down                          # 切换到下一帧
(gdb) info frame                    # 查看当前帧信息

# 其他
(gdb) quit                          # 退出 GDB
(gdb) help                          # 查看帮助
(gdb) help break                    # 查看 break 命令的帮助
```

#### 2.2 GDB 调试示例

**示例 1：调试电机初始化失败**

```bash
gdb ./bin/can_motor_app

# 在 MotorManager::Initialize 处设置断点
(gdb) break MotorManager::Initialize
(gdb) run

# 程序停在断点处，查看参数
(gdb) info args
(gdb) print thread_mgr

# 单步执行，找到失败的位置
(gdb) step
(gdb) step
(gdb) print m_motors[0][0]

# 继续执行直到返回
(gdb) finish
```

**示例 2：调试电机状态异常**

```bash
gdb ./bin/can_motor_app

# 在 MotorManager::GetStatus 处设置断点
(gdb) break MotorManager::GetStatus
(gdb) run

# 程序停在断点处，查看电机状态
(gdb) print can_port
(gdb) print motor_id
(gdb) print m_motors[can_port][motor_id-1]

# 查看电机的详细信息
(gdb) print m_motors[0][0].current_position
(gdb) print m_motors[0][0].current_speed
(gdb) print m_motors[0][0].error_code
```

**示例 3：调试线程问题**

```bash
gdb ./bin/can_motor_app

# 在线程函数处设置断点
(gdb) break MotorManager::ReceiveThreadFunc
(gdb) run

# 查看线程信息
(gdb) info threads
(gdb) thread 1  # 切换到线程 1
(gdb) backtrace  # 查看线程 1 的调用栈

# 继续执行
(gdb) continue
```

### 3. 日志调试

#### 3.1 添加调试日志

在代码中添加日志输出：

```cpp
#include <cstdio>
#include <ctime>

// 定义日志宏
#define LOG_DEBUG(fmt, ...) \
    do { \
        time_t now = time(nullptr); \
        struct tm* tm_info = localtime(&now); \
        char time_str[20]; \
        strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_info); \
        printf("[DEBUG %s] " fmt "\n", time_str, ##__VA_ARGS__); \
    } while(0)

#define LOG_INFO(fmt, ...) \
    printf("[INFO] " fmt "\n", ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...) \
    printf("[ERROR] " fmt "\n", ##__VA_ARGS__)

// 使用日志
LOG_DEBUG("Motor status: pos=%.2f, vel=%.2f", status.position, status.velocity);
LOG_INFO("Motor enabled successfully");
LOG_ERROR("Failed to initialize motor: %d", error_code);
```

#### 3.2 条件编译调试日志

```cpp
// 在头文件中定义
#ifdef DEBUG_LOG
    #define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

// 在代码中使用
DEBUG_PRINT("Motor %d status: %d", motor_id, status.error_code);
```

编译时启用调试日志：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-DDEBUG_LOG"
cmake --build build -j$(nproc)
```

#### 3.3 日志输出到文件

```cpp
#include <fstream>

// 创建日志文件
std::ofstream log_file("debug.log", std::ios::app);

// 写入日志
log_file << "[DEBUG] Motor status: pos=" << status.position << std::endl;
log_file.flush();

// 关闭文件
log_file.close();
```

运行程序并查看日志：

```bash
./bin/can_motor_app > app.log 2>&1
tail -f app.log
```

### 4. 内存调试

#### 4.1 使用 Valgrind 检测内存泄漏

```bash
# 安装 Valgrind
sudo apt-get install valgrind

# 运行内存检测
valgrind --leak-check=full --show-leak-kinds=all ./bin/can_motor_app

# 生成详细报告
valgrind --leak-check=full --log-file=valgrind.log ./bin/can_motor_app
cat valgrind.log

# 检测缓冲区溢出
valgrind --tool=memcheck --track-origins=yes ./bin/can_motor_app
```

#### 4.2 使用 AddressSanitizer

```bash
# 编译时启用 AddressSanitizer
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=address -g"
cmake --build build -j$(nproc)

# 运行程序
./bin/can_motor_app

# 输出示例：
# =================================================================
# ==12345==ERROR: AddressSanitizer: heap-buffer-overflow on unknown address 0x...
```

### 5. 性能调试

#### 5.1 使用 perf 分析性能

```bash
# 安装 perf
sudo apt-get install linux-tools-generic

# 记录性能数据
sudo perf record -g ./bin/can_motor_app

# 查看性能报告
sudo perf report

# 生成火焰图
sudo perf script | stackcollapse-perf.pl | flamegraph.pl > flame.svg
```

#### 5.2 使用 time 命令测量执行时间

```bash
# 测量程序执行时间
time ./bin/can_motor_app

# 输出示例：
# real    0m10.234s
# user    0m8.123s
# sys     0m2.111s
```

#### 5.3 使用 strace 追踪系统调用

```bash
# 追踪所有系统调用
strace ./bin/can_motor_app

# 只追踪特定系统调用
strace -e trace=open,read,write ./bin/can_motor_app

# 输出到文件
strace -o trace.log ./bin/can_motor_app
cat trace.log

# 统计系统调用
strace -c ./bin/can_motor_app
```

### 6. 线程调试

#### 6.1 使用 GDB 调试多线程

```bash
gdb ./bin/can_motor_app

# 查看所有线程
(gdb) info threads

# 切换到指定线程
(gdb) thread 2

# 在所有线程中设置断点
(gdb) break motor_manager.cpp:100 thread all

# 只在特定线程中设置断点
(gdb) break motor_manager.cpp:100 thread 2

# 继续执行所有线程
(gdb) continue

# 只继续当前线程
(gdb) continue -a off
```

#### 6.2 检测死锁

```bash
# 使用 GDB 检测死锁
gdb ./bin/can_motor_app

# 程序似乎卡住了，按 Ctrl+C 中断
# 查看所有线程的调用栈
(gdb) info threads
(gdb) thread apply all backtrace

# 查看互斥锁状态
(gdb) info threads
(gdb) thread 1
(gdb) print m_motor_mutex[0][0]
```

### 7. 调试技巧

#### 7.1 条件断点

```bash
gdb ./bin/can_motor_app

# 设置条件断点（只在特定条件下停止）
(gdb) break motor_manager.cpp:100 if motor_id == 1

# 设置断点后添加条件
(gdb) break motor_manager.cpp:100
(gdb) condition 1 motor_id == 1
```

#### 7.2 命令断点

```bash
gdb ./bin/can_motor_app

# 设置断点并自动执行命令
(gdb) break motor_manager.cpp:100
(gdb) commands 1
> print motor_id
> print status.position
> continue
> end
```

#### 7.3 远程调试

```bash
# 在远程机器上启动 GDB 服务器
gdbserver localhost:2345 ./bin/can_motor_app

# 在本地连接到远程调试器
gdb ./bin/can_motor_app
(gdb) target remote localhost:2345
(gdb) break main
(gdb) continue
```

### 8. 调试检查清单

调试时检查以下项目：

- [ ] 程序是否用 Debug 模式编译？
- [ ] 是否在正确的位置设置了断点？
- [ ] 是否检查了函数参数和返回值？
- [ ] 是否检查了指针是否为 nullptr？
- [ ] 是否检查了数组边界？
- [ ] 是否检查了互斥锁是否正确使用？
- [ ] 是否检查了线程是否正确启动和停止？
- [ ] 是否检查了内存是否正确分配和释放？
- [ ] 是否检查了 CAN 帧是否正确发送和接收？
- [ ] 是否检查了电机状态是否正确更新？

---

### Q1: 如何判断电机是否已连接 / 有响应？

**A**: 注意：`MotorStatus` 结构体（`include/common/types.h`，历史路径 `data_types.h`）
现在**只有 `motor_id / enable / position / velocity / torque / error_code` 六个字段**；
历史上曾有的 `ack` / `fault` 字段**已删除**，`MotorManager::GetStatus`
（`src/motor/motor_manager.cpp`）自然也不再填它们。`enable` 只是应用层命令态，
并不代表链路已通，不能用来判断连接状态。

判断电机是否有响应，建议用以下两种方式之一：

```cpp
// 方式 1：检查使能状态（GetStatus 会填充 enable）
MotorStatus status = motor_mgr.GetStatus(0, 1);
if (status.enable) {
    printf("Motor is enabled\n");
}

// 方式 2：观察反馈值是否更新（收到电机反馈帧后 position/velocity 会变化），
//         或在接收线程侧统计 CanTransport 的接收帧计数来确认链路是否有数据
```

> 若你需要额外的应答/故障语义，需自行在 `MotorStatus` 中新增字段并在
> `GetStatus` 中赋值（当前发送/接收链路见 `include/transport/can_transport.h`）。

### Q2: 电机报错怎么处理？

**A**: 检查 `error_code` 并清除错误：
```cpp
MotorStatus status = motor_mgr.GetStatus(0, 1);
if (status.error_code != 0) {
    printf("Error code: 0x%02x\n", status.error_code);
    motor_mgr.ClearError(0, 1);
}
```

### Q3: 如何设置实时优先级？

**A**: 在注册线程时指定优先级（需要 root 权限）。当前工程中 `motor_receive` /
`motor_send` 实际就是 **2ms（500Hz）+ 优先级 80**（见 `src/runtime/motor_io.cpp`）：
```cpp
thread_mgr.register_thread(
    "motor_receive",
    []() { /* 任务 */ },
    ThreadMode::LOOP,
    2,      // 2ms 间隔（500Hz）
    80      // SCHED_FIFO 优先级 80（1~99）
);
```

### Q4: 参数超出范围会怎样？

**A**: 框架会自动将参数限制在有效范围内。建议在发送前检查参数：
```cpp
float pos = 15.0f;  // 超出范围 ±12.5
if (pos > 12.5f) pos = 12.5f;
if (pos < -12.5f) pos = -12.5f;
motor_mgr.SendImpedance(0, 1, pos, 0.0f, 10.0f, 1.0f, 0.0f);
```

### Q5: 如何在线程间共享数据？

**A**: 使用 `SharedData` 接口：
```cpp
// 线程 A：写入数据
thread_mgr.get_shared_data().set<float>("target_speed", 50.0f);

// 线程 B：读取数据
float speed = thread_mgr.get_shared_data().get<float>("target_speed");
```

### Q6: 如何调试通信问题？

**A**: 启用日志输出（如果框架支持）：
```cpp
// 检查线程状态
ThreadState state = thread_mgr.get_thread_state("motor_receive");
printf("Thread state: %d\n", (int)state);
```

---

## 编译、运行和调试

### 1. 编译项目

#### 1.1 快速编译（Release 模式）

```bash
cd /home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

**说明**：
- `-B build` — 指定编译输出目录
- `-DCMAKE_BUILD_TYPE=Release` — 发布模式，优化性能
- `-j$(nproc)` — 使用所有 CPU 核心并行编译

#### 1.2 调试编译（Debug 模式）

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

**说明**：
- 包含调试符号，便于 GDB 调试
- 禁用优化，代码执行速度较慢
- 生成的可执行文件较大

#### 1.3 清理编译文件

```bash
# 删除编译目录
rm -rf build bin

# 重新编译
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

#### 1.4 增量编译

```bash
# 只编译修改过的文件
cmake --build build -j$(nproc)
```

#### 1.5 编译特定目标

```bash
# 只编译可执行文件
cmake --build build --target can_motor_app

# 查看所有可用目标
cmake --build build --target help
```

### 2. 运行程序

#### 2.1 直接运行

```bash
./bin/can_motor_app
```

**输出示例**：
```
[INFO] Running. Press Ctrl+C to exit.
```

#### 2.2 后台运行

```bash
# 后台运行，输出重定向到日志文件
./bin/can_motor_app > app.log 2>&1 &

# 查看进程
ps aux | grep can_motor_app

# 查看日志
tail -f app.log

# 停止进程
pkill can_motor_app
```

#### 2.3 选择要运行的示例

示例入口在 **`src/app/main.cpp`**（历史路径 `src/main.cpp`）的 `int main()`，
**不接收命令行参数**（`./bin/can_motor_app 2/3/4`、`./bin/can_motor_app <编号>` 这类用法
**都不存在**，也没有示例注册表）。切换示例的方式是：

**在 `main.cpp` 中取消注释目标示例的那 3 行（printf / 调用 / printf），其余全部保持注释，然后重新编译。**

- 现存示例：**17~57 共 40 个**（编号不连续；1~16 已清理，`Example55_SingleLegLimitMeasure`
  从未实现、已从示例体系移除，故 55 号不存在）。
- **当前启用：`Example37_RLTeleopControl`**（RL 遥操作，手柄前进/后退 + 转向，USB2CAN 4 路）。

```cpp
// src/app/main.cpp —— 只保留目标示例的 3 行不注释，其余保持注释
// printf("[INFO] Running Example36_RLStandLoop...\n");
// Example36_RLStandLoop();
// printf("[INFO] Example36 completed.\n");

// 运行示例37 - RL 遥操作（手柄前进/后退 + 转向，USB2CAN 4 路）
printf("[INFO] Running Example37_RLTeleopControl...\n");
Example37_RLTeleopControl();
printf("[INFO] Example37 completed.\n");

// printf("[INFO] Running Example56_FixedYawRecord...\n");
// Example56_FixedYawRecord();
// printf("[INFO] Example56 completed.\n");
```

修改后需要重新编译：

```bash
cmake --build build -j$(nproc)
./bin/can_motor_app
```

> `main.cpp` 顶部的「示例切换说明」注释块已同步为「现存 17~57（共 40 个；55 从未实现，
> 已删除）；当前启用 Example37」。切换示例时请顺手更新该注释块，并注意以文件末尾
> **实际未注释的那三行**为准。
>
> 如需支持 `./bin/can_motor_app <编号>` 这种命令行选择方式，需自行改造
> `main.cpp` 加入 `argc/argv` 解析。当前版本不支持。

**常用示例定位**：

| 编号 | 名称 | 用途 |
|---|---|---|
| Ex25 | `RLPolicyControl` | 完整 RL + 手柄 |
| Ex30 | `RLPolicyLinkTest` | 离线链路回归（不碰 CAN） |
| Ex36 | `RLStandLoop` | RL 站立 |
| **Ex37** | **`RLTeleopControl`** | **RL 遥操作（当前激活）** |
| Ex38 | `ActionDelayMeasure` | 动作延迟辨识 |
| Ex47 | `ChirpSysId` | 整狗 chirp 辨识 |
| Ex49 | `StandAndWheelSpeedLoopTest` | 轮 SPEED 环 kvp 扫描 |
| Ex51 | `StandRLThenLieDown` | 站立 → 趴下 |
| Ex54 | `FrictionSysId` | 吊装摩擦辨识 |
| Ex56 | `FixedYawRecord` | 固定 yaw 遥测落盘 |
| Ex57 | `SingleLegZeroAlign` | 单腿零位对照（验证 CONV_A/B） |

> ⚠️ 安全现状（2026-09-29 grep 核对）：40 个示例中 **27 个会调用
> `EnableMotor()` / `PreEnableZeroTorque()` 使能电机**，但仅 **11 个**装了 `SIGINT` 急停
> （**Ex25/34/35/36/37/38/51/52/53/54/56**）。运行其余示例前请确认现场安全与独立断电手段。

#### 2.4 停止程序（Ctrl+C）

```bash
./bin/can_motor_app
# 按 Ctrl+C 停止
```

> ⚠️ **当前 `main.cpp` 没有任何全局信号处理或统一清理逻辑。**
> 历史上此处的 `static RobotApp g_app` 以及只置位、从未被读取的
> `signal(SIGINT, signal_handler)` + `g_running` 处理器，已于 **2026-09-29 删除**
> （空处理器会吞掉 Ctrl+C，让未自装处理的示例"按了不退、电机持续使能"，比默认行为更危险）。
> 因此：
> - 未自行安装 `SIGINT` 处理的示例（40 个示例中其余 29 个）
>   **回归默认终止语义**：Ctrl+C 直接结束进程，不会调用 `MotorManager::Stop()`，
>   也**不会自动给电机发失能帧**。
> - 需要"优雅退出 / 急停"的示例自行调用 `signal(SIGINT, rl_signal_handler)`
>   （helper 见 `include/app/examples_common.h` 的 `g_rl_stop` + `rl_signal_handler`）；
>   2026-09-29 grep 核对的完整清单为 **Ex25/34/35/36/37/38/51/52/53/54/56**（共 11 个）。
>
> 调试运行前请确认现场安全与独立断电手段。若需完整清理流程，示意如下
> （示例多自建局部 `ThreadManager`，`RobotApp` 当前无人使用，见 `include/runtime/README.md`）：
> `thread_mgr.stop_thread("motor_receive")` / `stop_thread("motor_send")`
> → `MotorManager::GetInstance().Stop()`。

### 3. 调试方法

#### 3.1 使用 GDB 调试

```bash
# 1. 用 Debug 模式编译
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)

# 2. 启动 GDB
gdb ./bin/can_motor_app

# 3. GDB 命令
(gdb) break main              # 在 main 函数设置断点
(gdb) break motor_manager.cpp:100  # 在指定文件的第 100 行设置断点
(gdb) run                     # 运行程序
(gdb) next                    # 单步执行（不进入函数）
(gdb) step                    # 单步执行（进入函数）
(gdb) continue                # 继续执行
(gdb) print variable_name     # 打印变量值
(gdb) backtrace               # 打印调用栈
(gdb) quit                    # 退出 GDB
```

#### 3.2 使用 Valgrind 检测内存泄漏

```bash
# 安装 Valgrind（如果未安装）
sudo apt-get install valgrind

# 运行内存检测
valgrind --leak-check=full --show-leak-kinds=all ./bin/can_motor_app

# 生成详细报告
valgrind --leak-check=full --log-file=valgrind.log ./bin/can_motor_app
cat valgrind.log
```

#### 3.3 使用 strace 追踪系统调用

```bash
# 追踪所有系统调用
strace ./bin/can_motor_app

# 只追踪特定系统调用（如 open, read, write）
strace -e trace=open,read,write ./bin/can_motor_app

# 输出到文件
strace -o trace.log ./bin/can_motor_app
cat trace.log
```

#### 3.4 使用 perf 进行性能分析

```bash
# 安装 perf（如果未安装）
sudo apt-get install linux-tools-generic

# 记录性能数据
sudo perf record -g ./bin/can_motor_app

# 查看性能报告
sudo perf report

# 生成火焰图（需要安装 FlameGraph）
sudo perf script | stackcollapse-perf.pl | flamegraph.pl > flame.svg
```

#### 3.5 添加日志输出

在代码中添加调试日志：

```cpp
#include <cstdio>

// 在关键位置添加日志
printf("[DEBUG] Motor status: pos=%.2f, vel=%.2f\n", status.position, status.velocity);

// 条件编译调试日志
#ifdef DEBUG
    printf("[DEBUG] Entering function: %s\n", __FUNCTION__);
#endif
```

编译时启用调试日志：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DENABLE_DEBUG_LOG=ON
cmake --build build -j$(nproc)
```

### 4. 常见编译错误和解决方案

#### 错误 1：找不到 CMakeLists.txt

```
CMake Error: The source directory does not appear to contain CMakeLists.txt.
```

**解决方案**：
```bash
# 确保在项目根目录
cd /home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify
ls CMakeLists.txt  # 确认文件存在
```

#### 错误 2：缺少依赖库

```
error: CANET.h: No such file or directory
```

> ⚠️ **历史错误（CANET 已弃用）**：当前默认传输后端是达妙 USB2CAN，正式构建**不需要 CANET 库**。
> 只有编译/运行 Example27、Example28 这两个 CANET 探针示例时才会走到这条路径。
> 若这两个示例不参与当前编译，可忽略本条。

**解决方案**：
```bash
# 检查依赖库是否安装
find /usr -name "CANET.h" 2>/dev/null

# 如果未找到，需要安装 CANET 库
# 或在 CMakeLists.txt 中配置正确的包含路径
```

#### 错误 3：编译器版本过低

```
error: 'std::any' is not a member of 'std'
```

**解决方案**：
```bash
# 检查 C++ 标准版本
g++ --version

# 使用 C++17 或更高版本编译
cmake -B build -DCMAKE_CXX_STANDARD=17
cmake --build build -j$(nproc)
```

#### 错误 4：权限不足

```
error: Permission denied
```

**解决方案**：
```bash
# 添加执行权限
chmod +x ./bin/can_motor_app

# 或使用 sudo 运行
sudo ./bin/can_motor_app
```

#### 错误 5：找不到 SDL2 头文件

```
fatal error: SDL2/SDL.h: No such file or directory
```

**原因**：手柄示例依赖 SDL2，而 `main.cpp` 当前启用的 **Example37_RLTeleopControl**（USB2CAN Xbox 手柄遥操作）也需要 SDL2。

**解决方案**：
```bash
# 安装 SDL2 开发库
sudo apt-get install libsdl2-dev

# 重新配置并编译（必须重跑 cmake 才能检测到新装的 SDL2）
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

或在 `src/app/main.cpp` 中改用其它不依赖 SDL2 的示例（例如 Example30 离线链路回归、Example24 只读诊断）。

### 5. 性能优化

#### 5.1 编译优化选项

```bash
# 最大优化（-O3）
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3"
cmake --build build -j$(nproc)

# 链接时优化（LTO）
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON
cmake --build build -j$(nproc)
```

#### 5.2 运行时性能监控

```bash
# 使用 time 命令测量执行时间
time ./bin/can_motor_app

# 输出示例：
# real    0m10.234s
# user    0m8.123s
# sys     0m2.111s
```

#### 5.3 CPU 亲和性设置

```bash
# 将程序绑定到特定 CPU 核心
taskset -c 0,1 ./bin/can_motor_app

# 查看 CPU 亲和性
taskset -p $$
```

### 6. 完整的开发工作流

```bash
# 1. 进入项目目录
cd /home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify

# 2. 清理旧编译
rm -rf build bin

# 3. 用 Debug 模式编译
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)

# 4. 用 GDB 调试
gdb ./bin/can_motor_app

# 5. 修复 bug 后，用 Release 模式编译
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 6. 运行程序
./bin/can_motor_app

# 7. 按 Ctrl+C 停止程序
```

---

## 文件结构

> 🗄️ **下面的旧目录树是重构前的历史存档，请勿据此找文件。**
> `include/bsp/`、`include/thread/`、`include/motor_drive/`、`include/RoboTasks/`、
> `include/data_types.h`、`include/motor_manager.h`、`src/base/`、`src/example.cpp`、
> `src/bsp/` 等**全部已不存在**（详见文首「已失效名称 → 当前实现」对照表）。

**历史目录树（重构前）**：
```
simplify/
├── include/
│   ├── motor_manager.h          # 电机管理器（16 电机）      🔴 已移至 include/motor/
│   ├── can_device.h             # CAN 设备（VCI_CAN_OBJ 封装）
│   ├── data_types.h             # MotorStatus / CanDeviceConfig 等   🔴 现 include/common/types.h
│   ├── motor_calibration.h      # 电机标定参数                🔴 已移至 include/motor/
│   ├── motor_logger.h           # 收发日志（CSV）             🔴 已移至 include/common/
│   ├── leg_kinematics.h         # 腿部运动学                  🔴 已移至 include/motion/
│   ├── xbox_controller.h        # Xbox 手柄封装（依赖 SDL2）  🔴 已移至 include/strategy/
│   ├── SimSync.h                # 仿真同步                    🔴 已移至 include/motion/
│   ├── example.h                # 示例声明                    🔴 现 include/app/examples.h
│   ├── thread/                  🔴 已删（现 include/runtime/）
│   ├── motor_drive/             🔴 已删（现 include/motor/）
│   ├── bsp/bsp_can.h            🔴 已删（现 include/transport/）
│   └── RoboTasks/robot_app.h    🔴 已删（现 include/runtime/robot_app.h）
├── src/
│   ├── main.cpp                 # 入口（硬编码选择示例）      🔴 现 src/app/main.cpp
│   ├── example.cpp              # 示例 Example9~22          🔴 现 src/app/examples/*.cpp（17~57）
│   ├── motor_manager.cpp        🔴 现 src/motor/motor_manager.cpp
│   ├── can_device.cpp           🔴 现 src/transport/can_device.cpp
│   ├── motor_drive/ele_motor.cpp 🔴 现 src/motor/ele_motor.cpp
│   ├── thread/thread_manager.cpp 🔴 现 src/runtime/thread_manager.cpp
│   ├── bsp/bsp_can.cpp          🔴 已删（现 src/transport/{canet,usb2can}_transport.cpp）
│   ├── RoboTasks/robot_app.cpp  🔴 现 src/runtime/robot_app.cpp
│   └── base/                    🔴 已删除（厂商 common/crc16/log/jsoncpp/md5/network）
├── CMakeLists.txt
└── FRAMEWORK_GUIDE.md           # 本文档
```

**当前目录树（以文件系统为准）**：
```
simplify/
├── include/
│   ├── common/      # L0  types.h shared_data.h log_control.h motor_logger.h s2r_recorder.h
│   ├── transport/   # L1  can_transport.h canet_transport.h usb2can_transport.h can_device.h + CAN_TRANSPORT_GUIDE.md
│   ├── motor/       # L2/L3 ele_motor.h ele_motor_def.h motor_calibration.h motor_manager.h (+ 设计文档)
│   ├── runtime/     # L4  thread_manager.h robot_app.h motor_io.h (+ README/ThreadPlan)
│   ├── motion/      # L5  motion_controller.h leg_kinematics.h robot_calibration.h SimSync.h wheel_position_loop.h
│   ├── strategy/    # L6  rl_controller.h sim2real_conv.h mlp.h policy_weights.h policy_test_ref.h imu_device.h xbox_controller.h
│   └── app/         # L7  examples.h examples_common.h examples/{ex_basic,ex_diag,ex_rl}.h
├── src/
│   ├── common/s2r_recorder.cpp
│   ├── transport/{can_device,canet_transport,usb2can_transport}.cpp
│   ├── motor/{ele_motor,motor_manager}.cpp
│   ├── runtime/{thread_manager,robot_app,motor_io}.cpp
│   ├── motion/motion_controller.cpp
│   ├── strategy/{rl_controller,sim2real_conv,imu_device}.cpp
│   └── app/
│       ├── main.cpp                      # 入口（注释切换示例，无命令行参数）
│       └── examples/{ex_basic,ex_diag,ex_rl,examples_common}.cpp   # Example17~57 共 40 个
├── docs/            # 现状真源：SIM2REAL_DEPLOY.md 等
├── memory/          # FACT.md / JOURNAL.jsonl
├── weights/         # iteration_9754.pkl（真机当前权重）
├── dogurdf_sim2sim_deploy/   # sim2sim 部署（默认 checkpoint 已指向 ../weights/iteration_9754.pkl）
├── tool/            # Python 工具（export_policy.py / compare_sim2real.py / friction_id_offline.py ...）
├── lib/  log/  bin/          # 第三方库 / 运行日志 / 产物 bin/can_motor_app
├── CMakeLists.txt
└── FRAMEWORK_GUIDE.md        # 本文档（历史指南，通用调试章节仍有效）
```

> 注：`docs/SIM2REAL_DEPLOY.md` 与 `memory/FACT.md` 是现状真源；上述 `*.md` 指南文件
> 可能随文档同步继续更名，以实际 `ls` 结果为准。

---

## 总结

> ⚠️ 下面的总结是**重构前的视角**。当前工程：L0~L7 分层、16 电机（4 路 × 4）、
> 默认达妙 USB2CAN、示例 17~57 共 40 个（当前启用 Example37_RLTeleopControl）、
> 收发线程各 2ms/500Hz（优先级 80）、策略环 50Hz。现状以
> `docs/SIM2REAL_DEPLOY.md`、`memory/FACT.md` 为准。

本框架提供了一个**完整的、生产级别的**四足机器狗电机控制解决方案。通过分层设计和线程管理，实现了高效、安全、易用的电机控制接口。

**关键要点**：
- 使用 `MotorManager` 进行电机管理
- 使用 `ThreadManager` 进行线程管理
- 支持三种控制模式：阻抗、速度、位置
- 线程安全的并发访问
- 灵活的参数配置

更多信息请参考源代码注释和示例程序。
