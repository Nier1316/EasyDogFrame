# CAN 传输层使用指南（L1）

> ✅ **最新（2026-09-29）**
> 本文档描述 `include/transport/can_transport.h` 定义的 `CanTransport` 抽象接口及其两个实现。
> 历史文档《BSP CAN 硬件抽象层使用指南》描述的 `BspCan` 类已在 **2026-08-26 分层重构时删除**，
> 原 `bsp/bsp_can.h`、`BspCan::InitDevice/StartDevice/SendFrame/ReceiveFrames/ShutdownAll` 等 API 均已不存在；
> 旧 `BspCanFrame` 现为 `CanFrame` 的兼容别名（`canet_transport.h`）。
> 若你要用 CANET，请走 `CanetTransport`；默认后端是达妙 `Usb2CanTransport`。

## 📋 目录

1. [概述](#概述)
2. [核心类型](#核心类型)
3. [CanTransport 接口](#cantransport-接口)
4. [两个实现](#两个实现)
5. [注入 MotorManager](#注入-motormanager)
6. [使用示例](#使用示例)
7. [已知限制与注意事项](#已知限制与注意事项)
8. [常见问题](#常见问题)

---

## 概述

CAN 传输层（L1）是后端无关的收发抽象：上层（`MotorManager`、`EleMotor`）只依赖 `CanTransport` 纯虚接口，
换硬件（CANET 网口转换器 / 达妙 USB2FDCAN / 未来 SocketCAN）不需要改动电机编解码与管理层。

```
┌───────────────────────────────────────────────┐
│ 上层：MotorManager / EleMotor                 │
│   只持有 CanTransport*（每路一个）             │
└───────────────────┬───────────────────────────┘
                    │ open / send / sendBatch / recv / close / shutdown
┌───────────────────▼───────────────────────────┐
│ CanTransport（纯虚接口，can_transport.h）      │
└──────┬──────────────────────────┬─────────────┘
       │                          │
┌──────▼────────────┐   ┌─────────▼──────────────┐
│ CanetTransport    │   │ Usb2CanTransport       │
│ CANET TCP         │   │ 达妙 DM-USB2FDCAN      │
│ ⚠ 已弃用           │   │ ★ 默认后端             │
└───────────────────┘   └────────────────────────┘
```

| 后端 | 类 | 状态 | 适用 |
|------|----|------|------|
| CANET TCP | `CanetTransport` | ⚠️ 已弃用（2026-08-29 起默认切走，仅 Example27/28 仍直连） | 早期 CANET 网口转换器 |
| 达妙 USB2FDCAN | `Usb2CanTransport` | ✅ 当前默认后端 | 2 个双路模块覆盖 4 路 CAN |
| SocketCAN | `SocketCanTransport` | 🔴 预留，未实现（仅头注释提及，仓库无此类） | 未来 Linux canX |

---

## 核心类型

### `CanFrame`（`include/common/types.h`）

```cpp
struct CanFrame {
    uint32_t id;           // CAN ID
    uint8_t  dlc;          // 数据长度（0-8）
    uint8_t  data[8];      // 数据区
    uint8_t  is_extended;  // 0=标准帧，1=扩展帧

    CanFrame() : id(0), dlc(0), is_extended(0) { memset(data, 0, sizeof(data)); }
};
```

> `canet_transport.h` 里 `using BspCanFrame = CanFrame;` 仅为兼容旧调用点，新代码请用 `CanFrame`。

### `TransportConfig`（`include/transport/can_transport.h`）

一次 `open()` 的后端无关配置，**各实现只取自己需要的字段**：

```cpp
struct TransportConfig {
    uint8_t  device_idx = 0;   // 逻辑 CAN 通道索引（0~3）

    // CANET TCP 后端字段
    const char* tcp_ip   = nullptr;   // 远端 IP（客户端模式）
    uint16_t    tcp_port = 0;         // 端口
    uint8_t     tcp_mode = 0;         // TCP_CLIENT=0 / TCP_SERVER=1

    // USB2CAN 后端字段（达妙 DM-USB2FDCAN，ttyACM 串口协议）
    const char* usb_dev  = nullptr;   // USB 设备路径（如 "/dev/ttyACM0"）
    uint8_t     usb_baud = 0;         // CAN 波特率索引：0=1000k 1=800k 3=500k ...
};
```

`TCP_CLIENT` / `TCP_SERVER` 定义在 `include/common/types.h`（0 / 1）。

---

## CanTransport 接口

`include/transport/can_transport.h`：

```cpp
class CanTransport {
public:
    virtual ~CanTransport() = default;

    /** 打开并启动指定通道；cfg 后端相关字段见 TransportConfig */
    virtual bool open(uint8_t idx, const TransportConfig& cfg) = 0;

    /** 发送单帧 */
    virtual bool send(uint8_t idx, const CanFrame& f) = 0;

    /** 批量发送（后端可合并传输，避开逐帧延迟地板） */
    virtual bool sendBatch(uint8_t idx, const CanFrame* f, int n) = 0;

    /** 阻塞接收；有帧返回 true，超时返回 false */
    virtual bool recv(uint8_t idx, std::vector<CanFrame>& out, int timeout_ms) = 0;

    /** 停止并关闭通道 */
    virtual bool close(uint8_t idx) = 0;

    /** 关闭所有通道并释放资源 */
    virtual void shutdown() = 0;
};
```

约定：

- `idx` 是**逻辑 CAN 路**（0~3），与 `MotorManager` 的 `can_port`、四条腿（CAN0=FL / CAN1=FR / CAN2=RL / CAN3=RR）一一对应。
- `recv` **不做内部清空**：由调用者自备 `out`，有帧返回 `true`（可能一批多帧），超时无帧返回 `false`。
- 接口本身无锁；线程安全由各实现内部保证（两个实现都用 `std::mutex`）。

---

## 两个实现

### 1. `Usb2CanTransport`（达妙 DM-USB2FDCAN，★ 默认后端）

`include/transport/usb2can_transport.h` / `src/transport/usb2can_transport.cpp`，单例：`Usb2CanTransport::GetInstance()`。

- **物理映射**：逻辑路 `idx` → 物理 `(设备索引 = idx/2, 通道 = idx%2)`。
  2 个双路模块 = `CAN0=设备0 ch0`、`CAN1=设备0 ch1`、`CAN2=设备1 ch0`、`CAN3=设备1 ch1`。
- **`dmcan_find_devices` 只调用一次**（SDK 重复 find 会触发 `device_finder` 析构崩溃）。
- **`recv` 一次取空该路队列**：SDK 回调按 `(handle→设备索引, frame->head.channel)` 定位逻辑路并 `push_back` 入队，
  `recv` 按 CANET `VCI_Receive` 的"一次返回一批"语义把队列全部搬进 `out`（避免旧帧积压导致观测恒为旧值）。
  队列为空时按 `timeout_ms` 用条件变量等待。
- **`sendBatch`**：USB2CAN 无合并传输，实现为逐帧 `send()`，任一失败返回 `false`。
- **`close(idx)`**：只从逻辑通道表移除该路，**物理设备保持打开**（同一设备的 ch0/ch1 共享句柄），由 `shutdown()` 统一关。
- **诊断**：`Usb2CanTransport::RxCount()` 是 SDK 回调收到的总帧数，可与 `MotorManager` 的接收心跳对比，
  区分"SDK 回调停（USB 接收链路挂）"与"接收线程卡死（回调正常）"。

默认参数（`MotorManager` 私有成员，`Initialize()` 时填入 `TransportConfig`）：
`usb_dev = "/dev/ttyACM0"`、`usb_baud = 0`（1000k）。

### 2. `CanetTransport`（CANET TCP，⚠️ 已弃用）

`include/transport/canet_transport.h` / `src/transport/canet_transport.cpp`，单例：`CanetTransport::GetInstance()`。

- `open(idx, cfg)` = `InitDevice` + `StartDevice`，取 `cfg.tcp_ip / tcp_port / tcp_mode`。
- 除 `CanTransport` 六个虚函数外，还保留一套 CANET 专用 API（供 Example27/28 等直接调用）：

```cpp
bool InitDevice(uint8_t device_idx, const CanDeviceConfig& config);
bool StartDevice(uint8_t device_idx);
bool StopDevice(uint8_t device_idx);
bool CloseDevice(uint8_t device_idx);

bool SendFrame(uint8_t device_idx, const BspCanFrame& frame);            // BspCanFrame = CanFrame
bool Can_Tx(uint8_t device_idx, uint32_t can_id, const uint8_t* data, uint8_t dlc = 8);
bool ReceiveFrames(uint8_t device_idx, std::vector<BspCanFrame>& frames, int timeout_ms = 100);
bool SendFramesBatch(uint8_t device_idx, const std::vector<BspCanFrame>& frames);

void ShutdownAll();
```

`CanDeviceConfig`（`include/common/types.h`）：

```cpp
struct CanDeviceConfig {
    uint8_t     device_idx;    // CANET 设备索引（0~3）
    uint16_t    port;          // TCP 端口
    const char* server_ip;     // 远端 IP（仅客户端模式）
    uint8_t     work_mode;     // TCP_SERVER=1 / TCP_CLIENT=0
};
```

`MotorManager::Initialize()` 给 CANET 后端填的默认值是 `192.168.0.178:4001~4004`、`TCP_CLIENT`
——但那条路径当前已不生效，除非你在 `Initialize()` 前显式注入 `CanetTransport`。

---

## 注入 MotorManager

`MotorManager` 持有 `CanTransport* m_transport[CAN_PORTS]`（每路一个），两个注入入口：

```cpp
// 4 路全部换成同一后端
void MotorManager::SetTransport(CanTransport* transport);

// 只覆盖某一路（其余路保持默认）
void MotorManager::SetChannelTransport(uint8_t can_port, CanTransport* transport);
```

> ⚠️ **必须在 `MotorManager::Initialize(thread_mgr)` 之前调用。**
> `Initialize()` 会对仍为 `nullptr` 的通道做兜底（`m_transport[i] = &Usb2CanTransport::GetInstance()`），
> 并把当时的指针复制到每个 `EleMotor::transport`。初始化之后再调用 `SetTransport` 不会改变已创建电机持有的指针。

两个后端都是单例（`GetInstance()`），所以不需要自己 `new`，也不由调用方 `delete`。

---

## 使用示例

### 示例 1：默认后端（达妙 USB2CAN，无需注入）

```cpp
#include "motor/motor_manager.h"
#include "runtime/thread_manager.h"

MotorManager& motor_mgr = MotorManager::GetInstance();
ThreadManager thread_mgr;

// 不调用 SetTransport：Initialize() 会为 4 路兜底注入 Usb2CanTransport
if (!motor_mgr.Initialize(thread_mgr)) {
    printf("[ERROR] MotorManager 初始化失败\n");
    return -1;
}
thread_mgr.start_thread("motor_receive");
thread_mgr.start_thread("motor_send");
```

### 示例 2：4 路全走达妙 USB2CAN（显式注入）

```cpp
#include "motor/motor_manager.h"
#include "runtime/thread_manager.h"
#include "transport/usb2can_transport.h"

MotorManager& motor_mgr = MotorManager::GetInstance();
ThreadManager thread_mgr;

motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());   // 必须在 Initialize 之前

if (!motor_mgr.Initialize(thread_mgr)) { /* ... */ }
thread_mgr.start_thread("motor_receive");
thread_mgr.start_thread("motor_send");
```

### 示例 3：只把 CAN1 换成达妙模块（混用后端）

```cpp
#include "motor/motor_manager.h"
#include "runtime/thread_manager.h"
#include "transport/usb2can_transport.h"
#include "transport/canet_transport.h"

MotorManager& motor_mgr = MotorManager::GetInstance();
ThreadManager thread_mgr;

motor_mgr.SetTransport(&CanetTransport::GetInstance());              // 4 路先默认 CANET
motor_mgr.SetChannelTransport(1, &Usb2CanTransport::GetInstance());  // CAN1 覆盖为达妙

if (!motor_mgr.Initialize(thread_mgr)) { /* ... */ }
```

### 示例 4：直接使用 `CanTransport` 接口收发

```cpp
#include "transport/can_transport.h"
#include "transport/usb2can_transport.h"
#include <vector>

CanTransport& tp = Usb2CanTransport::GetInstance();

TransportConfig cfg;
cfg.device_idx = 0;
cfg.usb_dev    = "/dev/ttyACM0";
cfg.usb_baud   = 0;              // 0 = 1000k

if (!tp.open(0, cfg)) {
    printf("[ERROR] 打开 CAN0 失败\n");
    return -1;
}

CanFrame f;
f.id = 0x01;                     // motor_id
f.dlc = 8;
f.is_extended = 0;
for (int i = 0; i < 8; i++) f.data[i] = 0xFF;
if (!tp.send(0, f)) printf("[WARN] 发送失败\n");

std::vector<CanFrame> frames;    // 调用者自备容器
if (tp.recv(0, frames, 100)) {   // 阻塞最多 100ms
    for (const auto& rx : frames) {
        printf("ID=0x%03X dlc=%u\n", rx.id, rx.dlc);
    }
}

tp.close(0);
tp.shutdown();                   // 程序退出前统一释放
```

### 示例 5：CANET 后端（⚠️ 已弃用，仅作迁移参考）

```cpp
#include "transport/canet_transport.h"

CanetTransport& canet = CanetTransport::GetInstance();

CanDeviceConfig config;
config.device_idx = 0;
config.port       = 4001;
config.server_ip  = "192.168.0.178";
config.work_mode  = TCP_CLIENT;

if (!canet.InitDevice(0, config)) { /* ... */ }
if (!canet.StartDevice(0))        { /* ... */ }

uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
canet.Can_Tx(0, 0x123, data, 8);

std::vector<CanFrame> frames;
if (canet.ReceiveFrames(0, frames, 100)) {
    for (const auto& rx : frames) printf("ID=0x%03X\n", rx.id);
}

canet.StopDevice(0);
canet.CloseDevice(0);
canet.ShutdownAll();
```

---

## 已知限制与注意事项

### `libdm_device.so` 的运行环境依赖（达妙后端）

- `dmcan` SDK（`libdm_device.so`）依赖**新版 libusb（≥1.0.26）**与**新版 libstdc++（GLIBCXX ≥ 3.4.32）**。
  系统自带 gcc11 的 libstdc++ 只到 GLIBCXX_3.4.30、libusb 可能只有 1.0.25（缺 `libusb_init_context`），
  直接用系统库会加载/链接失败。
- CMake 会**自动探测本机 conda lib 目录**，并把 `libusb-1.0.so` 以绝对路径、`libstdc++` 以 `-L`/`-rpath` 指向它：

  ```cmake
  set(CONDA_LIB_DIR "" CACHE STRING "conda lib dir for damiao deps")
  # 未指定时依次探测 /home/bruce/miniforge3/lib、/home/sysu/miniconda3/lib（取含 libusb-1.0.so 的那个）
  ```

  探测不到会直接 `FATAL_ERROR`，此时用 `-DCONDA_LIB_DIR=<path>` 显式指定。
- 因此可执行文件**不能脱离该 conda 环境**（或需保证 rpath 指向的 lib 目录在位）。

### SDK `context_destroy` 析构 bug

`Usb2CanTransport::shutdown()` **只调用 `dmcan_device_close()` 关闭物理设备，然后清空内部表，
不调用 `dmcan_context_destroy()`**——SDK 的 context 析构路径有 bug（重复 find / destroy 会崩溃）。
`device_close` 时可能出现 `libusb_transfer_cancelled` 噪音，属已知现象、不崩。
同理 `dmcan_find_devices()` 全局只调用一次。

### 其他

- **`SetTransport`/`SetChannelTransport` 必须在 `Initialize()` 之前**（见上文）。
- `usb_dev` 需要读写权限（建议 udev `0666`），否则 `ensureDevice` 打开失败。
- `TransportConfig` 里的 `const char*` 字段不拷贝字符串，需保证生命周期覆盖整个 `open()` 调用。
- `recv` 返回 `true` 只表示"取到了至少一帧"，不代表该路一定已打开：
  `Usb2CanTransport::recv` 对**未打开的通道直接返回 `false`**（内部用 `m_channels.find(idx)`，
  不会像 `operator[]` 那样凭空建条目）。
- `can_transport.h` 的头注释与本指南口径一致：`Usb2CanTransport` = **当前默认后端**
  （`MotorManager::Initialize` 未注入时的兜底，见 `src/motor/motor_manager.cpp:63-67`）；
  `CanetTransport` = **已弃用**，仅 Example27/28 仍直连；`SocketCanTransport` = **预留，未实现**
  （头注释提到但仓库中无此类）。

---

## 常见问题

### Q1：怎么判断该用哪个后端？

- 现役硬件（2 个达妙双路模块）：用默认的 `Usb2CanTransport`，什么都不用注入。
- 只有老 CANET 网口转换器时才注入 `CanetTransport`（已弃用；注意 CANET 逐帧 `VCI_Transmit` 有 ~10ms 延迟地板，
  批量发送要用 `SendFramesBatch`）。

### Q2：为什么 `recv` 会一次返回很多帧？

`Usb2CanTransport` 按"一次取空队列"实现（对齐 CANET `VCI_Receive` 语义）：
高速回帧下若每次只取队头，旧帧会积压，上层永远解到过期状态。调用者应按批处理，不要假设 `frames.size() == 1`。

### Q3：调用 `close(idx)` 之后设备为什么还开着？

达妙一个物理设备带 2 路 CAN（ch0/ch1），`close` 只注销逻辑通道；物理设备由 `shutdown()` 统一关闭。
程序退出前调用 `shutdown()`（`MotorManager::Stop()` 内部会调用）。

### Q4：多线程并发收发安全吗？

两个后端内部都用 `std::mutex` 保护各自的通道表/队列，`Usb2CanTransport` 的接收队列还用条件变量唤醒。
上层 `motor_receive` / `motor_send` 两个 500Hz 线程分头调用 `recv` / `send`，不需要额外加锁。

### Q5：发送失败怎么排查？

- `Usb2CanTransport::send` 失败会限流打印 `[Usb2Can] send 失败 (n) CANx id=0x...`，并累计失败计数；
- 检查 `usb_dev` 是否存在、权限、波特率索引是否与电机总线一致（`usb_baud=0` → 1000k）；
- 若收不到任何回帧，先用 `Usb2CanTransport::RxCount()` 判断 SDK 回调是否还在触发（见上文诊断说明）。

---

## 相关文档

- `include/motor/MOTOR_MANAGER_GUIDE.md` —— 上层 `MotorManager` 的完整用法
- `include/transport/can_transport.h`、`usb2can_transport.h`、`canet_transport.h` —— 接口与实现
- `src/motor/motor_manager.cpp` —— 默认后端兜底与 `TransportConfig` 填充

---

**更新时间：** 2026-09-29
