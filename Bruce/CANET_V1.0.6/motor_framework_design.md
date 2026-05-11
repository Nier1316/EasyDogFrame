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
// 控制模式枚举
enum ControlMode {
    IMPEDANCE = 0,  // 阻抗控制
    SPEED     = 1,  // 速度控制
    POSITION  = 2   // 位置控制
};

// CAN 命令类型枚举
enum MotorCommandType {
    // 基础命令
    CMD_SET_SPEED     = 0x01,
    CMD_SET_TORQUE    = 0x02,
    CMD_SET_DIRECTION = 0x03,
    CMD_STOP          = 0x04,
    CMD_RESET         = 0x05,
    CMD_QUERY_STATUS  = 0x06,
    
    // 特殊指令
    CMD_ENABLE        = 0x10,  // 电机使能（0xFC）
    CMD_DISABLE       = 0x11,  // 电机失能（0xFD）
    CMD_SET_ZERO      = 0x12,  // 角度置零（0xFE）
    CMD_CLEAR_ERROR   = 0x13,  // 清除错误（0xF4）
    CMD_ANGLE_CORRECT = 0x14,  // 角度矫正（0xF7）
    
    // 参数指令
    CMD_READ_PARAM    = 0x20,  // 读参数
    CMD_WRITE_PARAM   = 0x21,  // 写参数
    
    // 控制指令
    CMD_IMPEDANCE_CTRL = 0x30, // 阻抗控制
    CMD_SPEED_CTRL     = 0x31, // 速度控制
    CMD_POSITION_CTRL  = 0x32  // 位置控制
};

// 电机命令结构体（支持浮点物理量）
struct MotorCommand {
    uint8_t     motor_id;      // 电机 ID (1-3)
    uint8_t     cmd_type;      // 命令类型
    ControlMode mode;          // 控制模式
    
    // 控制指令参数（浮点物理量）
    float       pos;           // 期望角度 rad，范围 ±12.5
    float       vel;           // 期望角速度 rad/s，范围 ±14
    float       kp;            // 刚度/位置环Kp，范围 0~500
    float       kd;            // 阻尼/位置环Kd，范围 0~100
    float       torque;        // 扭矩前馈 Nm，范围 ±200
    float       kp_speed;      // 速度环Kp
    float       ki_speed;      // 速度环Ki
    
    // 参数读写指令参数
    float       param_value;   // 参数值
    uint8_t     param_type;    // 参数类型
    uint8_t     param_rw;      // 0=读，1=写
};

// 电机状态结构体（按硬件实际 6 字节返回帧格式）
struct MotorStatus {
    uint8_t motor_id;   // CAN_ID（data[0] bit3-0）
    bool    ack;        // data[0] bit7：收到指令=1
    bool    fault;      // data[0] bit6：驱动错误=1
    bool    enable;     // data[0] bit4：使能=1
    float   position;   // rad，由 data[1-2] 16位解码
    float   velocity;   // rad/s，由 data[3]高4位+data[5] 12位解码
    float   torque;     // Nm，由 data[3]低4位+data[4] 12位解码
    uint8_t error_code; // 框架层错误码
};

// CAN 设备配置
struct CanDeviceConfig {
    uint8_t     device_idx;    // 设备索引 (0-3)
    uint16_t    port;          // TCP 端口
    const char* server_ip;     // 服务器 IP
    uint8_t     work_mode;     // TCP_SERVER 或 TCP_CLIENT
};

// 电机配置
struct MotorConfig {
    uint8_t  can_port;     // CAN 口 (0-3)
    uint8_t  motor_id;     // 电机 ID (1-3)
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
- 命令编码/解码（支持硬件协议位域打包）
- 状态管理

```cpp
class Motor {
public:
    Motor(uint8_t can_port, uint8_t motor_id, uint16_t rx_id, uint16_t tx_id);
    ~Motor();

    // 特殊指令接口
    bool Enable();
    bool Disable();
    bool SetZero();
    bool ClearError();
    bool SetControlMode(ControlMode mode);

    // 控制指令接口
    bool ImpedanceControl(float pos, float vel, float kp, float kd, float torque);
    bool SpeedControl(float vel, float kp, float ki);
    bool PositionControl(float pos, float kvp, float kp, float kd, float kvi);

    // 参数读写接口
    bool ReadParam(uint8_t param_type);
    bool WriteParam(uint8_t param_type, float value);

    // 兼容旧接口
    bool SetSpeed(uint16_t speed);
    bool SetTorque(uint16_t torque);
    bool SetDirection(uint8_t direction);
    bool Stop();
    bool Reset();
    bool QueryStatus();

    // 状态查询
    MotorStatus GetStatus() const;
    bool IsHealthy() const;
    uint8_t GetErrorCode() const;

    // 内部使用
    void UpdateStatus(const MotorStatus& status);
    BspCanFrame EncodeCommand(const MotorCommand& cmd);
    MotorStatus DecodeStatus(const BspCanFrame& frame);

private:
    uint8_t m_can_port;
    uint8_t m_motor_id;
    uint16_t m_rx_id;
    uint16_t m_tx_id;
    MotorStatus m_current_status;
    std::mutex m_status_mutex;
};
```

**编码格式**：
- **特殊指令**：`{0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, special_byte}`
- **参数指令**：`{0x80, val[0], val[1], val[2], val[3], RW, param_type, 0xEC}`
- **阻抗控制**：5参数位域打包（pos/vel/kp/kd/torque）
- **速度控制**：vel/kvp/kvi 位域打包
- **位置控制**：pos/kvp/kp/kd/kvi 位域打包

**解码格式**（6字节硬件返回帧）：
- `data[0]`：ACK(bit7) + FAULT(bit6) + ENABLE(bit4) + CAN_ID(bit3-0)
- `data[1-2]`：position（16位）
- `data[3]`：velocity高4位 + current高4位
- `data[4]`：current低8位
- `data[5]`：velocity低8位

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
    bool IsRunning() const;

    // 高级控制接口
    bool MoveMotor(uint8_t can_port, uint8_t motor_id, uint16_t speed, uint8_t direction);
    bool SetMotorTorque(uint8_t can_port, uint8_t motor_id, uint16_t torque);
    bool StopMotor(uint8_t can_port, uint8_t motor_id);
    bool StopAllMotors();
    bool ResetMotor(uint8_t can_port, uint8_t motor_id);

    // 新增控制接口（支持三种控制模式）
    bool EnableMotor(uint8_t can_port, uint8_t motor_id);
    bool DisableMotor(uint8_t can_port, uint8_t motor_id);
    bool SetMotorZero(uint8_t can_port, uint8_t motor_id);
    bool ClearMotorError(uint8_t can_port, uint8_t motor_id);
    bool SetControlMode(uint8_t can_port, uint8_t motor_id, ControlMode mode);
    bool ImpedanceControl(uint8_t can_port, uint8_t motor_id,
                          float pos, float vel, float kp, float kd, float torque);
    bool SpeedControl(uint8_t can_port, uint8_t motor_id,
                      float vel, float kp, float ki);
    bool PositionControl(uint8_t can_port, uint8_t motor_id,
                         float pos, float kvp, float kp, float kd, float kvi);

    // 状态监控
    MotorStatus GetMotorStatus(uint8_t can_port, uint8_t motor_id);
    void PrintMotorStatus(uint8_t can_port, uint8_t motor_id);
    void PrintAllMotorStatus();

    // 错误处理
    bool CheckMotorHealth(uint8_t can_port, uint8_t motor_id);
    void HandleMotorError(uint8_t can_port, uint8_t motor_id, uint8_t error_code);

private:
    MotorManager& m_motor_manager;
    bool m_is_initialized;
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
