# CAN 电机通信框架设计

## 系统架构概览

```
┌─────────────────────────────────────────────────────────┐
│                    应用层 (Application)                  │
│              - 电机控制逻辑                              │
│              - 状态管理                                  │
│              - 数据处理                                  │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                  电机管理层 (Motor Manager)              │
│         - 电机对象管理                                   │
│         - 电机命令下发                                   │
│         - 电机状态反馈                                   │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                 CAN 设备层 (CAN Device)                  │
│         - 设备初始化/启动/停止                           │
│         - 数据收发                                       │
│         - 缓冲管理                                       │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│              CANET 库 (CANET Library)                    │
│         - TCP 通信                                       │
│         - 设备管理                                       │
└─────────────────────────────────────────────────────────┘
```

## 硬件配置

```
can0 ──┬─→ Motor 1 (ID=1, RxID=51)
       ├─→ Motor 2 (ID=2, RxID=52)
       └─→ Motor 3 (ID=3, RxID=53)

can1 ──┬─→ Motor 4 (ID=1, RxID=51)
       ├─→ Motor 5 (ID=2, RxID=52)
       └─→ Motor 6 (ID=3, RxID=53)

can2 ──┬─→ Motor 7 (ID=1, RxID=51)
       ├─→ Motor 8 (ID=2, RxID=52)
       └─→ Motor 9 (ID=3, RxID=53)

can3 ──┬─→ Motor 10 (ID=1, RxID=51)
       ├─→ Motor 11 (ID=2, RxID=52)
       └─→ Motor 12 (ID=3, RxID=53)
```

## 框架结构

### 1. 数据定义层 (data_types.h)

```cpp
// CAN 帧定义
struct MotorCommand {
    uint8_t motor_id;      // 电机 ID (1-3)
    uint8_t cmd_type;      // 命令类型
    uint16_t speed;        // 速度
    uint16_t torque;       // 扭矩
    uint8_t direction;     // 方向
};

struct MotorStatus {
    uint8_t motor_id;      // 电机 ID
    uint16_t speed;        // 当前速度
    uint16_t torque;       // 当前扭矩
    uint8_t temperature;   // 温度
    uint8_t error_code;    // 错误码
};

// CAN 设备配置
struct CanDeviceConfig {
    uint8_t device_idx;    // 设备索引 (0-3)
    uint16_t port;         // TCP 端口
    const char* server_ip; // 服务器 IP
    uint8_t work_mode;     // TCP_SERVER 或 TCP_CLIENT
};

// 电机配置
struct MotorConfig {
    uint8_t can_port;      // CAN 口 (0-3)
    uint8_t motor_id;      // 电机 ID (1-3)
    uint16_t rx_id;        // 接收 CAN ID (51-53)
    uint16_t tx_id;        // 发送 CAN ID (1-3)
};
```

### 2. CAN 设备层 (CanDevice)

**职责**：
- 管理单个 CAN 设备的生命周期
- 处理数据收发
- 缓冲管理

```cpp
class CanDevice {
public:
    CanDevice(uint8_t device_idx);
    ~CanDevice();

    // 生命周期管理
    bool Initialize(const CanDeviceConfig& config);
    bool Start();
    bool Stop();
    void Shutdown();

    // 数据收发
    bool SendFrame(const VCI_CAN_OBJ& frame);
    bool ReceiveFrames(std::vector<VCI_CAN_OBJ>& frames, int timeout_ms = 100);

    // 状态查询
    bool IsRunning() const;
    uint32_t GetReceivedFrameCount() const;

private:
    uint8_t m_device_idx;
    bool m_is_running;
    uint32_t m_received_count;
    std::mutex m_mutex;
};
```

### 3. 电机驱动层 (Motor)

**职责**：
- 管理单个电机
- 命令编码/解码
- 状态管理

```cpp
class Motor {
public:
    Motor(uint8_t can_port, uint8_t motor_id, uint16_t rx_id, uint16_t tx_id);
    ~Motor();

    // 命令接口
    bool SetSpeed(uint16_t speed);
    bool SetTorque(uint16_t torque);
    bool SetDirection(uint8_t direction);
    bool Stop();
    bool Reset();

    // 状态查询
    MotorStatus GetStatus() const;
    bool IsHealthy() const;
    uint8_t GetErrorCode() const;

    // 内部使用
    void UpdateStatus(const MotorStatus& status);
    VCI_CAN_OBJ EncodeCommand(const MotorCommand& cmd);
    MotorStatus DecodeStatus(const VCI_CAN_OBJ& frame);

private:
    uint8_t m_can_port;
    uint8_t m_motor_id;
    uint16_t m_rx_id;
    uint16_t m_tx_id;
    MotorStatus m_current_status;
    std::mutex m_status_mutex;
};
```

### 4. 电机管理层 (MotorManager)

**职责**：
- 管理所有电机
- 协调多个 CAN 设备
- 命令分发
- 状态收集

```cpp
class MotorManager {
public:
    static MotorManager& GetInstance();

    // 初始化
    bool Initialize(const std::vector<CanDeviceConfig>& can_configs);
    bool Start();
    bool Stop();

    // 电机管理
    Motor* GetMotor(uint8_t can_port, uint8_t motor_id);
    std::vector<Motor*> GetAllMotors();

    // 命令下发
    bool SendMotorCommand(uint8_t can_port, uint8_t motor_id, const MotorCommand& cmd);
    bool BroadcastCommand(const MotorCommand& cmd);  // 广播命令

    // 状态查询
    MotorStatus GetMotorStatus(uint8_t can_port, uint8_t motor_id);
    std::vector<MotorStatus> GetAllMotorStatus();

    // 后台处理
    void ProcessReceivedData();  // 在独立线程中运行

private:
    MotorManager();
    ~MotorManager();

    std::map<uint8_t, CanDevice*> m_can_devices;      // CAN 设备
    std::map<std::string, Motor*> m_motors;           // 电机 (key: "can_port_motor_id")
    std::thread m_process_thread;
    bool m_is_running;
    std::mutex m_mutex;

    std::string MakeMotorKey(uint8_t can_port, uint8_t motor_id);
};
```

### 5. 应用层接口 (MotorController)

**职责**：
- 提供高级 API
- 电机控制逻辑
- 错误处理

```cpp
class MotorController {
public:
    MotorController();
    ~MotorController();

    // 系统初始化
    bool Initialize();
    bool Start();
    bool Stop();

    // 高级控制接口
    bool MoveMotor(uint8_t can_port, uint8_t motor_id, uint16_t speed, uint8_t direction);
    bool StopMotor(uint8_t can_port, uint8_t motor_id);
    bool StopAllMotors();

    // 状态监控
    void PrintMotorStatus(uint8_t can_port, uint8_t motor_id);
    void PrintAllMotorStatus();

    // 错误处理
    bool CheckMotorHealth(uint8_t can_port, uint8_t motor_id);
    void HandleMotorError(uint8_t can_port, uint8_t motor_id, uint8_t error_code);

private:
    MotorManager& m_motor_manager;
};
```

## 数据流

### 发送流程
```
应用层
  ↓
MotorController::MoveMotor()
  ↓
MotorManager::SendMotorCommand()
  ↓
Motor::EncodeCommand() → VCI_CAN_OBJ
  ↓
CanDevice::SendFrame()
  ↓
CANET 库
  ↓
CAN 总线 → 电机
```

### 接收流程
```
电机 → CAN 总线
  ↓
CANET 库
  ↓
CanDevice::ReceiveFrames()
  ↓
MotorManager::ProcessReceivedData()
  ↓
Motor::DecodeStatus()
  ↓
Motor::UpdateStatus()
  ↓
应用层读取 Motor::GetStatus()
```

## 关键设计特点

### 1. 分层设计
- 清晰的职责划分
- 易于维护和扩展
- 低耦合度

### 2. 线程安全
- 使用 mutex 保护共享数据
- 后台线程处理接收数据
- 异步处理机制

### 3. 可扩展性
- 支持动态添加/删除电机
- 支持多种工作模式
- 易于添加新的命令类型

### 4. 错误处理
- 完整的错误检测
- 电机健康状态监控
- 异常恢复机制

## 使用示例

```cpp
#include "motor_controller.h"

int main() {
    // 1. 创建控制器
    MotorController controller;

    // 2. 初始化系统
    if (!controller.Initialize()) {
        printf("初始化失败\n");
        return -1;
    }

    // 3. 启动系统
    if (!controller.Start()) {
        printf("启动失败\n");
        return -1;
    }

    // 4. 控制电机
    // can0 上的电机 1，速度 1000，正向
    controller.MoveMotor(0, 1, 1000, 1);

    // 5. 查询状态
    controller.PrintMotorStatus(0, 1);

    // 6. 停止电机
    controller.StopMotor(0, 1);

    // 7. 停止系统
    controller.Stop();

    return 0;
}
```

## 文件组织

```
project/
├── include/
│   ├── data_types.h           # 数据定义
│   ├── can_device.h           # CAN 设备
│   ├── motor.h                # 电机驱动
│   ├── motor_manager.h        # 电机管理
│   └── motor_controller.h     # 应用接口
├── src/
│   ├── can_device.cpp
│   ├── motor.cpp
│   ├── motor_manager.cpp
│   ├── motor_controller.cpp
│   └── main.cpp
├── CMakeLists.txt
└── README.md
```

## 编译和运行

```bash
# 编译
mkdir build && cd build
cmake ..
make

# 运行
./motor_app
```

## 扩展建议

1. **添加日志系统** - 记录所有操作
2. **添加配置文件** - 从 JSON/YAML 读取配置
3. **添加 Web 界面** - 远程监控和控制
4. **添加数据记录** - 记录电机运行数据
5. **添加故障诊断** - 自动诊断和恢复
