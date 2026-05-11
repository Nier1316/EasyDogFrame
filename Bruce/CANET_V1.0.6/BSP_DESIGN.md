# BSP 层架构设计文档

## 1. 概述

### 目标
将所有 CAN 硬件操作封装在 BSP（Board Support Package）层，使上层应用（Motor、MotorManager）与硬件细节解耦，便于测试和移植。

### 新架构
```
应用层
  ↓
MotorController / MotorManager
  ↓
Motor (电机编解码)
  ↓
bsp_can (BSP 层 - 硬件抽象)
  ↓
CANET 库 (底层驱动)
```

---

## 2. BSP 层职责划分

### BSP 层负责
- ✅ 所有 CANET 库 API 调用（VCI_OpenDevice、VCI_Transmit 等）
- ✅ CAN 设备的生命周期管理（打开、配置、启动、停止、关闭）
- ✅ CAN 帧的收发（SendFrame、ReceiveFrames）
- ✅ 线程安全的并发访问控制
- ✅ 硬件状态管理（运行状态、计数器等）

### 上层（Motor）负责
- ✅ CAN 帧数据的编解码（命令 ↔ 字节序列）
- ✅ 电机状态缓存和查询
- ✅ 电机命令构造

### 中层（MotorManager）负责
- ✅ 电机和 CAN 设备的拓扑管理
- ✅ 命令路由（找到目标电机 → 找到对应 CAN 设备 → 调用 BSP 发送）
- ✅ 后台接收线程（轮询 BSP 接收帧 → 分发到 Motor）

---

## 3. BSP 层接口设计

### 3.1 数据结构

```cpp
// 保留现有的 data_types.h 中的结构体
// CanDeviceConfig、MotorCommand、MotorStatus 等不变

// BSP 层新增：CAN 帧的通用表示（隐藏 VCI_CAN_OBJ）
struct BspCanFrame {
    uint32_t id;           // CAN ID
    uint8_t  dlc;          // 数据长度（0-8）
    uint8_t  data[8];      // 数据区
    uint8_t  is_extended;  // 是否扩展帧（0=标准帧，1=扩展帧）
};
```

### 3.2 核心接口

#### 3.2.1 BspCan 类（单例）

```cpp
class BspCan {
public:
    // 单例入口
    static BspCan& GetInstance();

    // ============ 设备生命周期 ============
    
    // 初始化指定 CAN 设备
    // @param device_idx: 设备索引（0-3）
    // @param config: 设备配置（TCP 模式、端口等）
    // @return: 成功返回 true
    bool InitDevice(uint8_t device_idx, const CanDeviceConfig& config);
    
    // 启动 CAN 设备（使其能收发）
    bool StartDevice(uint8_t device_idx);
    
    // 停止 CAN 设备
    bool StopDevice(uint8_t device_idx);
    
    // 关闭 CAN 设备（释放资源）
    bool CloseDevice(uint8_t device_idx);
    
    // 查询设备是否运行中
    bool IsDeviceRunning(uint8_t device_idx) const;

    // ============ 数据收发 ============
    
    // 发送单个 CAN 帧
    // @param device_idx: 目标设备
    // @param frame: 帧数据
    // @return: 成功返回 true
    bool SendFrame(uint8_t device_idx, const BspCanFrame& frame);
    
    // 批量接收 CAN 帧
    // @param device_idx: 源设备
    // @param frames: 输出容器（调用者负责清空）
    // @param timeout_ms: 阻塞等待时间（毫秒）
    // @return: 有帧返回 true，无帧返回 false
    bool ReceiveFrames(uint8_t device_idx, 
                       std::vector<BspCanFrame>& frames, 
                       int timeout_ms = 100);

    // ============ 状态查询 ============
    
    // 获取设备接收帧计数
    uint32_t GetReceivedFrameCount(uint8_t device_idx) const;
    
    // 获取设备发送帧计数
    uint32_t GetSentFrameCount(uint8_t device_idx) const;

    // ============ 全局控制 ============
    
    // 初始化所有设备
    bool InitAllDevices(const std::vector<CanDeviceConfig>& configs);
    
    // 启动所有设备
    bool StartAllDevices();
    
    // 停止所有设备
    bool StopAllDevices();
    
    // 关闭所有设备并释放资源
    void ShutdownAll();

private:
    BspCan();
    ~BspCan();
    
    // 禁用拷贝
    BspCan(const BspCan&) = delete;
    BspCan& operator=(const BspCan&) = delete;
    
    // 内部实现细节（对外隐藏）
    class CanDeviceWrapper;
    std::map<uint8_t, std::unique_ptr<CanDeviceWrapper>> m_devices;
    mutable std::mutex m_mutex;
};
```

---

## 4. 接口转换示例

### 4.1 从 VCI_CAN_OBJ 到 BspCanFrame

```cpp
// BSP 内部转换（对外隐藏）
BspCanFrame ConvertFromVciFrame(const VCI_CAN_OBJ& vci_frame) {
    BspCanFrame frame;
    frame.id = vci_frame.ID;
    frame.dlc = vci_frame.DataLen;
    frame.is_extended = vci_frame.ExternFlag;
    memcpy(frame.data, vci_frame.Data, 8);
    return frame;
}

VCI_CAN_OBJ ConvertToVciFrame(const BspCanFrame& frame) {
    VCI_CAN_OBJ vci_frame;
    memset(&vci_frame, 0, sizeof(vci_frame));
    vci_frame.ID = frame.id;
    vci_frame.DataLen = frame.dlc;
    vci_frame.ExternFlag = frame.is_extended;
    vci_frame.RemoteFlag = 0;
    memcpy(vci_frame.Data, frame.data, 8);
    return vci_frame;
}
```

---

## 5. 数据流示例

### 5.1 发送流程

```
MotorController::MoveMotor()
    ↓
MotorManager::SendMotorCommand()
    ↓
Motor::EncodeCommand() → MotorCommand → BspCanFrame
    ↓
BspCan::SendFrame(device_idx, frame)
    ↓
CANET 库 VCI_Transmit()
```

### 5.2 接收流程

```
后台线程 MotorManager::ProcessReceivedData()
    ↓
BspCan::ReceiveFrames(device_idx, frames)
    ↓
CANET 库 VCI_Receive()
    ↓
遍历 frames，按 frame.id 分发到对应 Motor
    ↓
Motor::DecodeStatus(frame) → MotorStatus
    ↓
Motor::UpdateStatus()
```

---

## 6. 测试方案

由于你的电脑已连接 CANET 设备，可以直接在硬件上测试 CAN 通信，即使没有电机也能验证：
- CAN 帧的发送和接收
- 编解码逻辑
- 设备管理和线程安全

**测试步骤**：
1. 初始化 CANET 设备（无需电机）
2. 发送测试帧到 CAN 总线
3. 接收并验证帧数据
4. 检查编解码是否正确

---

## 7. 迁移步骤（已完成）

### 第一阶段：创建 BSP 层 ✅
1. ✅ 创建 `include/bsp/bsp_can.h` — 接口定义
2. ✅ 创建 `src/bsp/bsp_can.cpp` — 真实实现（基于现有 CanDevice）

### 第二阶段：修改上层代码 ✅
1. ✅ 修改 `Motor` — 使用 BspCanFrame 而不是 VCI_CAN_OBJ
2. ✅ 修改 `MotorManager` — 调用 BspCan 接口而不是 CanDevice
3. ✅ 保留 CanDevice 为 BSP 内部实现

### 第三阶段：编译和测试 ✅
1. ✅ 编译 Release 版本成功
2. ✅ 生成 `bin/motor_app` (280KB)
3. ✅ 生成 `lib/libmotor_framework.a` (96KB)

---

## 8. 优势总结

| 方面 | 优势 |
|------|------|
| **测试** | 无需硬件即可测试电机逻辑 |
| **移植** | 只需重新实现 BSP 层，上层代码不变 |
| **维护** | 硬件相关代码集中在 BSP，易于修改 |
| **解耦** | Motor 不依赖 CANET 库，依赖倒置 |
| **灵活性** | 支持多种 CAN 硬件（只需不同的 BSP 实现） |

---

## 9. 文件结构

```
CANET_V1.0.6/
├── include/
│   ├── bsp/
│   │   └── bsp_can.h              ← BSP 接口定义 ✅
│   ├── can_device.h               ← 保留（BSP 内部使用）
│   ├── motor.h                    ← 修改（使用 BspCanFrame）✅
│   ├── motor_manager.h            ← 修改（使用 BspCan）✅
│   ├── data_types.h               ← 新增 TCP_SERVER/TCP_CLIENT 常量 ✅
│   └── motor_controller.h
├── src/
│   ├── bsp/
│   │   └── bsp_can.cpp            ← 真实实现 ✅
│   ├── motor.cpp                  ← 修改 ✅
│   ├── motor_manager.cpp          ← 修改 ✅
│   ├── can_device.cpp             ← 保留（BSP 内部使用）
│   ├── motor_controller.cpp
│   └── main.cpp
└── CMakeLists.txt                 ← 添加 BSP 编译 ✅
```

---

## 10. 关键设计决策

| 决策 | 理由 |
|------|------|
| **BspCanFrame 而非 VCI_CAN_OBJ** | 隐藏 CANET 库细节，便于移植 |
| **单例模式** | 全局唯一的 CAN 硬件资源管理 |
| **CanDeviceWrapper 包装** | 在 BSP 内部管理 CanDevice，对外隐藏实现细节 |
| **保留 CanDevice** | 作为 BSP 内部实现，无需大幅重构 |
| **最小化接口** | 只暴露必要的功能，易于维护 |

---

## 11. 实现总结

### 编译结果 ✅
- **编译状态**：成功（仅有格式警告，无错误）
- **产物大小**：
  - `bin/motor_app` — 280KB（示例可执行文件）
  - `lib/libmotor_framework.a` — 96KB（框架静态库）

### 新增文件
- `include/bsp/bsp_can.h` — BSP 接口定义（~150 行）
- `src/bsp/bsp_can.cpp` — BSP 实现（~260 行）

### 修改文件
- `include/motor.h` — 使用 BspCanFrame
- `src/motor.cpp` — 更新编解码方法
- `include/motor_manager.h` — 移除 m_can_devices
- `src/motor_manager.cpp` — 使用 BspCan 接口
- `include/data_types.h` — 添加 TCP_SERVER/TCP_CLIENT 常量
- `CMakeLists.txt` — 添加 BSP 编译

### 架构改进
```
原架构：应用 → MotorManager → Motor → CanDevice → CANET 库
新架构：应用 → MotorManager → Motor → BspCan → CanDevice → CANET 库
                                        ↑
                                    硬件抽象层
```

### 优势
✅ **硬件解耦** — 上层代码与 CANET 库完全解耦  
✅ **易于移植** — 只需重新实现 BSP 层即可支持其他 CAN 硬件  
✅ **代码清晰** — 职责分离，各层独立  
✅ **无硬件测试** — 可以在没有电机的情况下测试 CAN 通信  

