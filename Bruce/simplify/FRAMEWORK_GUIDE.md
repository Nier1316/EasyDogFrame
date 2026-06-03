# 四足机器狗电机控制框架 — 完整使用指南

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

本框架是一个**分层的四足机器狗电机控制系统**，通过 CAN 总线与 12 个电机通信，支持多种控制模式（阻抗控制、速度控制、位置控制）。

### 核心特性
- **12 电机管理**：4 路 CAN 口，每路 3 个电机（髋、大腿、小腿）
- **多线程架构**：ThreadManager 统一管理所有后台线程
- **线程安全**：每个电机配备独立互斥锁，支持并发访问
- **灵活的控制模式**：支持阻抗、速度、位置三种控制方式
- **实时优先级**：支持 Linux SCHED_FIFO 实时调度

---

## 架构设计

### 分层结构

```
┌─────────────────────────────────────────┐
│         应用层 (RobotApp)               │  用户应用
├─────────────────────────────────────────┤
│    管理层 (MotorManager + ThreadManager) │  电机管理 + 线程管理
├─────────────────────────────────────────┤
│    设备层 (CanDevice + EleMotor)        │  CAN设备 + 单电机
├─────────────────────────────────────────┤
│    BSP层 (BspCan)                       │  硬件抽象
├─────────────────────────────────────────┤
│    硬件 (CANET + 电机)                   │  物理设备
└─────────────────────────────────────────┘
```

### 模块职责

| 模块 | 文件 | 职责 |
|------|------|------|
| **ThreadManager** | `include/thread/thread_manager.h` | 线程生命周期管理、共享数据区 |
| **CanDevice** | `include/can_device.h` | CANET 设备封装、TCP 连接管理 |
| **EleMotor** | `include/motor_drive/ele_motor.h` | 单电机数据结构、状态管理 |
| **MotorManager** | `include/motor_manager.h` | 12 电机批量管理、命令分发 |
| **BspCan** | `include/bsp/bsp_can.h` | CAN 收发底层接口 |
| **DataTypes** | `include/data_types.h` | 通用数据结构定义 |

---

## 硬件拓扑

### CAN 口分配

```
CAN0 → 左前腿 (FL)
  ├─ motor_id=1 → 髋关节 (Hip)
  ├─ motor_id=2 → 大腿 (Thigh)
  └─ motor_id=3 → 小腿 (Calf)

CAN1 → 右前腿 (FR)
  ├─ motor_id=1 → 髋关节
  ├─ motor_id=2 → 大腿
  └─ motor_id=3 → 小腿

CAN2 → 左后腿 (RL)
  ├─ motor_id=1 → 髋关节
  ├─ motor_id=2 → 大腿
  └─ motor_id=3 → 小腿

CAN3 → 右后腿 (RR)
  ├─ motor_id=1 → 髋关节
  ├─ motor_id=2 → 大腿
  └─ motor_id=3 → 小腿
```

### CAN 帧 ID 映射

| 方向 | CAN ID | 说明 |
|------|--------|------|
| 上位机 → 电机 | 1, 2, 3 | 对应 motor_id |
| 电机 → 上位机 | 51, 52, 53 | 50 + motor_id |

### TCP 连接参数

```
服务器地址：192.168.0.178
端口分配：
  - CAN0: 4001
  - CAN1: 4002
  - CAN2: 4003
  - CAN3: 4004
```

---

## 快速开始

### 1. 编译项目

```bash
cd /home/nier1316/Desktop/EasyDogFrame/Bruce/simplify
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### 2. 初始化框架

```cpp
#include "motor_manager.h"
#include "thread/thread_manager.h"

int main() {
    // 创建线程管理器
    ThreadManager thread_mgr;
    
    // 初始化电机管理器
    MotorManager& motor_mgr = MotorManager::GetInstance();
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("Failed to initialize MotorManager\n");
        return -1;
    }
    
    // 启动接收线程（1ms 间隔）
    thread_mgr.start_thread("motor_receive");
    
    // ... 控制电机 ...
    
    // 清理资源
    thread_mgr.stop_thread("motor_receive");
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
    1,                      // 1ms 间隔
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

**职责**：管理 12 个电机，提供统一的控制接口

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

#### MotorCommand — 电机命令

```cpp
struct MotorCommand {
    uint8_t     motor_id;      // 电机 ID（1~3）
    uint8_t     cmd_type;      // 命令类型（MotorCommandType 枚举）
    ControlMode mode;          // 控制模式
    
    // 控制参数
    float       pos;           // 期望位置 (rad)，范围 ±12.5
    float       vel;           // 期望速度 (rad/s)，范围 ±65
    float       kp;            // 刚度系数，范围 0~500
    float       kd;            // 阻尼系数，范围 0~5
    float       torque;        // 扭矩前馈 (Nm)，范围 ±18
    float       kp_speed;      // 速度环 Kp
    float       ki_speed;      // 速度环 Ki
    
    // 参数读写
    float       param_value;   // 参数值
    uint8_t     param_type;    // 参数类型
    uint8_t     param_rw;      // 0=读，1=写
};
```

#### MotorStatus — 电机状态

```cpp
struct MotorStatus {
    uint8_t motor_id;   // 电机 ID
    bool    ack;        // 收到指令标志
    bool    fault;      // 驱动错误标志
    bool    enable;     // 使能状态
    float   position;   // 当前位置 (rad)
    float   velocity;   // 当前速度 (rad/s)
    float   torque;     // 当前扭矩 (Nm)
    uint8_t error_code; // 错误码
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

#### MotorCommandType — 命令类型

```cpp
enum MotorCommandType {
    // 特殊指令
    CMD_ENABLE        = 0x10,  // 电机使能
    CMD_DISABLE       = 0x11,  // 电机失能
    CMD_SET_ZERO      = 0x12,  // 角度置零
    CMD_CLEAR_ERROR   = 0x13,  // 清除错误
    CMD_ANGLE_CORRECT = 0x14,  // 角度矫正
    
    // 参数指令
    CMD_READ_PARAM    = 0x20,  // 读参数
    CMD_WRITE_PARAM   = 0x21,  // 写参数
    
    // 控制指令
    CMD_IMPEDANCE_CTRL = 0x30, // 阻抗控制
    CMD_SPEED_CTRL     = 0x31, // 速度控制
    CMD_POSITION_CTRL  = 0x32  // 位置控制
};
```

### 参数范围

| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| 位置 (pos) | ±12.5 | rad | 关节角度 |
| 速度 (vel) | ±65 | rad/s | 关节角速度 |
| 扭矩 (torque) | ±18 | Nm | 关节扭矩 |
| Kp | 0~500 | - | 刚度/位置环比例系数 |
| Kd | 0~5 | - | 阻尼系数 |
| Ki | 0~500 | - | 速度环积分系数 |

---

## 使用示例

### 示例 1：基础电机控制

```cpp
#include "motor_manager.h"
#include "thread/thread_manager.h"
#include <unistd.h>

int main() {
    ThreadManager thread_mgr;
    MotorManager& motor_mgr = MotorManager::GetInstance();
    
    // 初始化
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("Initialize failed\n");
        return -1;
    }
    
    // 启动接收线程
    thread_mgr.start_thread("motor_receive");
    
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
    motor_mgr.Stop();
    
    return 0;
}
```

### 示例 2：多电机同时控制

```cpp
// 使能所有 12 个电机
for (int can_port = 0; can_port < 4; can_port++) {
    for (int motor_id = 1; motor_id <= 3; motor_id++) {
        motor_mgr.EnableMotor(can_port, motor_id);
    }
}

sleep(1);

// 所有电机归零
for (int can_port = 0; can_port < 4; can_port++) {
    for (int motor_id = 1; motor_id <= 3; motor_id++) {
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

### Q1: 如何判断电机是否已连接？

**A**: 检查 `MotorStatus` 中的 `ack` 标志：
```cpp
MotorStatus status = motor_mgr.GetStatus(0, 1);
if (status.ack) {
    printf("Motor is connected\n");
} else {
    printf("Motor is not responding\n");
}
```

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

**A**: 在注册线程时指定优先级（需要 root 权限）：
```cpp
thread_mgr.register_thread(
    "motor_receive",
    []() { /* 任务 */ },
    ThreadMode::LOOP,
    1,      // 1ms 间隔
    50      // SCHED_FIFO 优先级 50（1~99）
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
// 检查接收帧计数
uint32_t count = can_dev.GetReceivedFrameCount();
printf("Received %u frames\n", count);

// 检查线程状态
ThreadState state = thread_mgr.get_thread_state("motor_receive");
printf("Thread state: %d\n", (int)state);
```

---

## 编译、运行和调试

### 1. 编译项目

#### 1.1 快速编译（Release 模式）

```bash
cd /home/nier1316/Desktop/EasyDogFrame/Bruce/simplify
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

#### 2.3 带参数运行（如果支持）

```bash
# 基础电机控制
./bin/can_motor_app 1

# 使能所有电机
./bin/can_motor_app 2

# 全关节归零
./bin/can_motor_app 3

# 站立姿态
./bin/can_motor_app 4
```

#### 2.4 优雅停止程序

程序支持 **Ctrl+C** 优雅停止：

```bash
./bin/can_motor_app
# 按 Ctrl+C 停止程序
# 程序会自动调用 g_app.stop() 进行清理
```

**工作原理**：
- 捕获 `SIGINT` (Ctrl+C) 和 `SIGTERM` 信号
- 设置 `g_running = false` 标志
- 主循环检测到标志后退出
- 调用 `g_app.stop()` 进行资源清理

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
cd /home/nier1316/Desktop/EasyDogFrame/Bruce/simplify
ls CMakeLists.txt  # 确认文件存在
```

#### 错误 2：缺少依赖库

```
error: CANET.h: No such file or directory
```

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
cd /home/nier1316/Desktop/EasyDogFrame/Bruce/simplify

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

```
simplify/
├── include/
│   ├── motor_manager.h          # 电机管理器
│   ├── can_device.h             # CAN 设备
│   ├── data_types.h             # 数据结构定义
│   ├── thread/
│   │   └── thread_manager.h     # 线程管理器
│   ├── motor_drive/
│   │   ├── ele_motor.h          # 单电机
│   │   └── ele_motor_def.h      # 电机参数定义
│   └── bsp/
│       └── bsp_can.h            # CAN 底层接口
├── src/
│   ├── motor_manager.cpp
│   ├── can_device.cpp
│   ├── motor_drive/
│   │   └── ele_motor.cpp
│   ├── thread/
│   │   └── thread_manager.cpp
│   ├── bsp/
│   │   └── bsp_can.cpp
│   ├── main.cpp
│   └── example.cpp
├── CMakeLists.txt
└── FRAMEWORK_GUIDE.md           # 本文档
```

---

## 总结

本框架提供了一个**完整的、生产级别的**四足机器狗电机控制解决方案。通过分层设计和线程管理，实现了高效、安全、易用的电机控制接口。

**关键要点**：
- 使用 `MotorManager` 进行电机管理
- 使用 `ThreadManager` 进行线程管理
- 支持三种控制模式：阻抗、速度、位置
- 线程安全的并发访问
- 灵活的参数配置

更多信息请参考源代码注释和示例程序。
