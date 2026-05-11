# CAN 电机控制框架 - 快速开始指南

## 项目概述

这是一个为 CANET 库设计的**分层电机控制框架**，用于管理 4 个 CAN 口上的 12 个电机（每个口 3 个）。

### 硬件配置
- **CAN 口**: can0 ~ can3（4 个）
- **电机数量**: 12 个（每个 CAN 口 3 个）
- **电机 ID**: 1-3（每个 CAN 口内）
- **接收 CAN ID**: 51-53
- **发送 CAN ID**: 1-3

## 框架架构

```
应用层 (Application)
    ↓
MotorController (高级 API)
    ↓
MotorManager (电机管理)
    ↓
Motor (电机驱动) + CanDevice (CAN 设备)
    ↓
CANET 库
    ↓
CAN 总线
```

## 文件结构

```
project/
├── include/                          # 头文件
│   ├── data_types.h                 # 数据定义
│   ├── can_device.h                 # CAN 设备类
│   ├── motor.h                      # 电机驱动类
│   ├── motor_manager.h              # 电机管理器
│   └── motor_controller.h           # 应用接口
├── src/                              # 源文件
│   ├── can_device.cpp
│   ├── motor.cpp
│   ├── motor_manager.cpp
│   ├── motor_controller.cpp
│   └── main.cpp                     # 示例程序
├── CMakeLists.txt                   # 编译配置
└── README.md                        # 本文件
```

## 编译和运行

### 1. 编译

```bash
# 进入项目目录
cd /path/to/CANET_V1.0.6

# 创建编译目录
mkdir -p build
cd build

# 配置编译（Release 版本）
cmake -DCMAKE_BUILD_TYPE=Release ..

# 编译
make -j$(nproc)
```

### 2. 运行

```bash
# 运行示例 1（基本控制）
./bin/motor_app 1

# 运行示例 2（多电机控制）
./bin/motor_app 2

# 运行示例 3（多 CAN 口控制）
./bin/motor_app 3

# 运行示例 4（电机健康检查）
./bin/motor_app 4

# 运行示例 5（交互式控制）
./bin/motor_app 5
```

## 核心类说明

### 1. MotorController（应用接口）

**最常用的类，提供高级 API**

```cpp
MotorController controller;

// 初始化和启动
controller.Initialize();
controller.Start();

// 控制电机
controller.MoveMotor(can_port, motor_id, speed, direction);
controller.StopMotor(can_port, motor_id);
controller.StopAllMotors();

// 查询状态
controller.PrintMotorStatus(can_port, motor_id);
controller.PrintAllMotorStatus();

// 停止
controller.Stop();
```

### 2. MotorManager（电机管理器）

**单例模式，管理所有电机和 CAN 设备**

```cpp
MotorManager& manager = MotorManager::GetInstance();

// 初始化
manager.Initialize(can_configs);
manager.Start();

// 获取电机
Motor* motor = manager.GetMotor(can_port, motor_id);

// 发送命令
manager.SendMotorCommand(can_port, motor_id, cmd);

// 查询状态
MotorStatus status = manager.GetMotorStatus(can_port, motor_id);
```

### 3. Motor（电机驱动）

**代表单个电机**

```cpp
Motor motor(can_port, motor_id, rx_id, tx_id);

// 命令
motor.SetSpeed(speed);
motor.SetTorque(torque);
motor.Stop();

// 状态
MotorStatus status = motor.GetStatus();
bool healthy = motor.IsHealthy();
```

### 4. CanDevice（CAN 设备）

**代表单个 CAN 设备**

```cpp
CanDevice device(device_idx);

// 初始化
device.Initialize(config);
device.Start();

// 收发
device.SendFrame(frame);
device.ReceiveFrames(frames);

// 停止
device.Stop();
```

## 使用示例

### 示例 1：基本控制

```cpp
#include "motor_controller.h"

int main() {
    MotorController controller;
    
    // 初始化
    if (!controller.Initialize() || !controller.Start()) {
        return -1;
    }
    
    // 控制 can0 上的电机 1
    controller.MoveMotor(0, 1, 1000, 1);  // 速度 1000，正向
    sleep(2);
    
    // 查询状态
    controller.PrintMotorStatus(0, 1);
    
    // 停止
    controller.StopMotor(0, 1);
    controller.Stop();
    
    return 0;
}
```

### 示例 2：多电机控制

```cpp
// 同时控制多个电机
for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
    controller.MoveMotor(0, motor_id, 1000, 1);
}

sleep(3);

// 打印所有状态
controller.PrintAllMotorStatus();

// 停止所有
controller.StopAllMotors();
```

### 示例 3：多 CAN 口控制

```cpp
// 控制不同 CAN 口上的电机
for (uint8_t can_port = 0; can_port < 4; can_port++) {
    controller.MoveMotor(can_port, 1, 1000, 1);
}

sleep(3);

// 停止所有
controller.StopAllMotors();
```

### 示例 4：错误处理

```cpp
// 检查电机健康状态
if (!controller.CheckMotorHealth(0, 1)) {
    MotorStatus status = controller.GetMotorStatus(0, 1);
    controller.HandleMotorError(0, 1, status.error_code);
}
```

## 数据结构

### MotorCommand（电机命令）

```cpp
struct MotorCommand {
    uint8_t motor_id;      // 电机 ID (1-3)
    uint8_t cmd_type;      // 命令类型
    uint16_t speed;        // 速度 (0-65535)
    uint16_t torque;       // 扭矩 (0-65535)
    uint8_t direction;     // 方向 (0=反向, 1=正向)
};
```

### MotorStatus（电机状态）

```cpp
struct MotorStatus {
    uint8_t motor_id;      // 电机 ID
    uint16_t speed;        // 当前速度
    uint16_t torque;       // 当前扭矩
    uint8_t temperature;   // 温度 (°C)
    uint8_t error_code;    // 错误码 (0=正常)
    uint8_t state;         // 状态 (0=停止, 1=运行, 2=故障)
};
```

## 命令类型

```cpp
enum MotorCommandType {
    CMD_SET_SPEED = 0x01,      // 设置速度
    CMD_SET_TORQUE = 0x02,     // 设置扭矩
    CMD_SET_DIRECTION = 0x03,  // 设置方向
    CMD_STOP = 0x04,           // 停止
    CMD_RESET = 0x05,          // 复位
    CMD_QUERY_STATUS = 0x06    // 查询状态
};
```

## 错误码

```cpp
enum ErrorCode {
    ERR_OK = 0x00,                  // 正常
    ERR_MOTOR_OVERHEAT = 0x01,      // 电机过热
    ERR_MOTOR_OVERCURRENT = 0x02,   // 电机过流
    ERR_MOTOR_STALL = 0x03,         // 电机堵转
    ERR_COMMUNICATION = 0x04,       // 通信错误
    ERR_TIMEOUT = 0x05,             // 超时
    ERR_INVALID_PARAM = 0x06,       // 无效参数
    ERR_DEVICE_OFFLINE = 0x07       // 设备离线
};
```

## 线程安全

框架使用 `std::mutex` 保护所有共享数据，支持多线程访问：

- **MotorManager** 在后台线程中处理 CAN 数据接收
- **Motor** 使用 mutex 保护状态更新
- **CanDevice** 使用 mutex 保护设备操作

## 扩展建议

### 1. 添加日志系统

```cpp
// 在 motor_controller.cpp 中添加日志
#include <fstream>

std::ofstream log_file("motor_control.log");
log_file << "[INFO] Motor started\n";
```

### 2. 添加配置文件支持

```cpp
// 从 JSON 文件读取配置
#include "jsoncpp/json.h"

Json::Value config;
// 解析配置...
```

### 3. 添加 Web 界面

```cpp
// 使用 HTTP 库提供 REST API
// GET /motors - 获取所有电机状态
// POST /motors/{id}/move - 控制电机
```

### 4. 添加数据记录

```cpp
// 记录电机运行数据
struct MotorRecord {
    uint64_t timestamp;
    MotorStatus status;
};
```

## 常见问题

### Q1: 如何修改 CAN 口数量或电机数量？

**A:** 修改 `motor_manager.cpp` 中的 `CreateMotors()` 函数：

```cpp
void MotorManager::CreateMotors() {
    // 修改这里的循环范围
    for (uint8_t can_port = 0; can_port < 4; can_port++) {  // 修改 CAN 口数
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {  // 修改电机数
            // ...
        }
    }
}
```

### Q2: 如何自定义 CAN ID 映射？

**A:** 修改 `motor_manager.cpp` 中的 `CreateMotors()` 函数：

```cpp
uint16_t rx_id = 50 + motor_id;  // 修改接收 ID 计算
uint16_t tx_id = motor_id;       // 修改发送 ID 计算
```

### Q3: 如何修改 TCP 端口？

**A:** 修改 `motor_controller.cpp` 中的 `Initialize()` 函数：

```cpp
config.port = 4001 + i;  // 修改端口号
```

### Q4: 如何添加自定义命令？

**A:** 
1. 在 `data_types.h` 中添加命令类型
2. 在 `motor.cpp` 中实现编码/解码
3. 在 `motor_controller.h` 中添加公共接口

### Q5: 如何处理电机离线？

**A:** 框架会自动检测离线状态（错误码 `ERR_DEVICE_OFFLINE`），可以在 `motor_controller.cpp` 中的 `HandleMotorError()` 中添加处理逻辑。

## 性能优化

1. **增加接收缓冲区** - 在 `can_device.cpp` 中修改缓冲区大小
2. **调整线程睡眠时间** - 在 `motor_manager.cpp` 中修改 `ProcessCanData()` 的睡眠时间
3. **使用 Release 编译** - 编译时使用 `-DCMAKE_BUILD_TYPE=Release`

## 技术支持

- 查看 `motor_framework_design.md` 了解详细的架构设计
- 查看 `LINUX_USER_GUIDE.md` 了解 CANET 库的使用
- 查看源代码中的注释了解实现细节

---

**版本**: 1.0  
**最后更新**: 2026-05-10  
**作者**: Nier1316
