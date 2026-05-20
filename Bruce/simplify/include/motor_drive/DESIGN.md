# 电机驱动模块设计文档

## 📋 目录
1. [概述](#概述)
2. [异步状态同步设计](#异步状态同步设计)
3. [核心概念](#核心概念)
4. [关键设计要素](#关键设计要素)
5. [代码架构](#代码架构)
6. [实现流程](#实现流程)
7. [关键考虑点](#关键考虑点)
8. [使用示例](#使用示例)
9. [优势](#优势)

---

## 概述

本文档描述了电机驱动模块（`EleMotor`）的设计方案，特别是**异步状态同步机制**的实现思路。

该设计允许外部代码只需修改电机对象的目标状态，后台线程会自动检测状态变化并发送相应的 CAN 命令，确保电机的实际状态与类中的状态保持一致。

---

## 异步状态同步设计

### 设计流程图

```
┌─────────────────────────────────────────────────────────────┐
│                     外部代码                                  │
│  motor.set_target_speed(10.0f)                              │
│  motor.set_target_position(3.14f)                           │
└────────────────────┬────────────────────────────────────────┘
                     │ 修改目标状态
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                   EleMotor 对象                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 目标状态: target_speed, target_position, ...        │   │
│  │ 实际状态: current_speed, current_position, ...      │   │
│  │ 状态标志: state_version, need_sync                  │   │
│  └─────────────────────────────────────────────────────┘   │
└────────────────────┬────────────────────────────────────────┘
                     │ 标记需要同步
                     ▼
┌─────────────────────────────────────────────────────────────┐
│              后台同步线程 (sync_thread)                      │
│  1. 检测状态变化 (need_sync)                                │
│  2. 发送 CAN 命令 (set_motor_para_bt)                       │
│  3. 接收电机反馈 (unpack_cmd)                               │
│  4. 更新实际状态                                             │
│  5. 循环等待 (10-50ms)                                      │
└────────────────────┬────────────────────────────────────────┘
                     │ 发送 CAN 命令
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    CAN 总线                                  │
│              (BspCan 硬件抽象层)                             │
└────────────────────┬────────────────────────────────────────┘
                     │ CAN 帧
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                   电机硬件                                    │
│              执行控制命令                                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 核心概念

### 1. 状态分离
- **目标状态**：外部代码设置的期望值
  - `target_speed` - 期望速度
  - `target_torque` - 期望扭矩
  - `target_position` - 期望位置
  
- **实际状态**：从电机反馈的当前值
  - `current_speed` - 当前速度
  - `current_torque` - 当前扭矩
  - `current_position` - 当前位置
  - `current_temp` - 当前温度

### 2. 异步同步
- 外部代码**不直接**调用 `enable()`, `disable()`, `set_motor_para_bt()`
- 只修改目标状态
- 后台线程**自动**检测变化并发送 CAN 命令

### 3. 线程安全
- 使用 `std::mutex` 保护共享状态
- 使用 `std::atomic` 实现无锁标志位
- 避免死锁和数据竞争

---

## 关键设计要素

### 1. 状态变化检测

**方案 A：版本号（推荐）**
```cpp
std::atomic<uint32_t> state_version;  // 状态版本号

void set_target_speed(float speed) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        target_speed = speed;
        state_version++;  // 版本号递增
    }
    need_sync = true;
}
```

**优点：**
- 可以检测到具体哪个状态改变了
- 支持增量同步
- 精确度高

**方案 B：脏标志位**
```cpp
std::atomic<bool> need_sync;  // 是否需要同步

void set_target_speed(float speed) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        target_speed = speed;
    }
    need_sync = true;
}
```

**优点：**
- 实现简单
- 开销小

### 2. 后台线程管理

```cpp
std::thread sync_thread;           // 同步线程
std::atomic<bool> running;         // 运行标志
std::mutex state_mutex;            // 状态互斥锁
std::condition_variable cv;        // 条件变量（可选）
```

### 3. 同步频率

- **推荐：10-50ms 一次**
  - 10ms：高实时性，适合快速响应
  - 50ms：低功耗，适合后台监控
  - 可配置：`sync_interval_ms`

### 4. 错误处理

```cpp
struct SyncResult {
    bool success;
    uint16_t error_code;
    std::string error_msg;
};

SyncResult last_sync_result;  // 最后一次同步结果
```

---

## 代码架构

### 类定义框架

```cpp
class EleMotor {
private:
    // ========== 配置信息 ==========
    uint8_t device_idx;
    uint8_t motor_id;
    
    // ========== 状态信息 ==========
    // 目标状态
    float target_speed;
    float target_torque;
    float target_position;
    
    // 实际状态
    float current_speed;
    float current_torque;
    float current_position;
    float current_temp;
    
    // 状态标志
    uint16_t error_code;
    bool enabled;
    
    // ========== 同步机制 ==========
    std::thread sync_thread;
    std::atomic<bool> running;
    std::atomic<bool> need_sync;
    std::atomic<uint32_t> state_version;
    std::mutex state_mutex;
    
    uint32_t last_synced_version;  // 上次同步的版本号
    int sync_interval_ms;          // 同步间隔（毫秒）
    
    // ========== 私有方法 ==========
    void sync_thread_func();       // 后台线程主函数
    void sync_state();             // 同步状态到电机
    bool has_state_changed();      // 检查状态是否改变
    
public:
    // ========== 生命周期 ==========
    EleMotor();
    ~EleMotor();
    
    void init();
    void start_sync();             // 启动同步线程
    void stop_sync();              // 停止同步线程
    
    // ========== 外部接口（只修改状态）==========
    void set_target_speed(float speed);
    void set_target_torque(float torque);
    void set_target_position(float position);
    void set_control_mode(ControlMode mode);
    
    // ========== 状态查询 ==========
    float get_current_speed() const;
    float get_current_torque() const;
    float get_current_position() const;
    float get_current_temp() const;
    bool is_enabled() const;
    bool has_error() const;
    
    // ========== 配置 ==========
    void set_sync_interval(int ms);
    int get_sync_interval() const;
};
```

---

## 实现流程

### 1. 初始化阶段

```cpp
EleMotor::EleMotor() 
    : device_idx(0), motor_id(0), running(false), need_sync(false),
      state_version(0), last_synced_version(0), sync_interval_ms(20) {
    // 初始化所有状态
    init();
}

void EleMotor::init() {
    std::lock_guard<std::mutex> lock(state_mutex);
    current_speed = 0.0f;
    current_torque = 0.0f;
    current_position = 0.0f;
    current_temp = 0.0f;
    target_speed = 0.0f;
    target_torque = 0.0f;
    target_position = 0.0f;
    error_code = 0;
    enabled = false;
}
```

### 2. 启动同步线程

```cpp
void EleMotor::start_sync() {
    if (running) return;
    
    running = true;
    sync_thread = std::thread(&EleMotor::sync_thread_func, this);
}

void EleMotor::stop_sync() {
    running = false;
    if (sync_thread.joinable()) {
        sync_thread.join();
    }
}
```

### 3. 后台线程主函数

```cpp
void EleMotor::sync_thread_func() {
    while (running) {
        // 检查状态是否改变
        if (has_state_changed()) {
            sync_state();
        }
        
        // 定期接收电机反馈
        unpack_cmd(*this, 100);
        
        // 等待指定时间
        std::this_thread::sleep_for(
            std::chrono::milliseconds(sync_interval_ms)
        );
    }
}
```

### 4. 状态变化检测

```cpp
bool EleMotor::has_state_changed() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return state_version != last_synced_version;
}
```

### 5. 状态同步

```cpp
void EleMotor::sync_state() {
    std::lock_guard<std::mutex> lock(state_mutex);
    
    // 根据当前模式发送对应命令
    if (control_mode == SPEED) {
        set_motor_para_bt(*this, target_speed, kp, ki, 0, 0, SPEED);
    } 
    else if (control_mode == POSITION) {
        set_motor_para_bt(*this, target_position, kp, kd, ki, 0, POSITION);
    }
    else if (control_mode == IMPEDANCE) {
        set_motor_para_bt(*this, target_position, target_speed, kp, kd, 
                         target_torque, IMPEDANCE);
    }
    
    // 更新同步版本号
    last_synced_version = state_version;
}
```

### 6. 外部接口

```cpp
void EleMotor::set_target_speed(float speed) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        target_speed = speed;
        state_version++;
    }
    need_sync = true;
}

void EleMotor::set_target_position(float position) {
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        target_position = position;
        state_version++;
    }
    need_sync = true;
}
```

### 7. 析构函数

```cpp
EleMotor::~EleMotor() {
    stop_sync();  // 确保线程被正确停止
}
```

---

## 关键考虑点

| 考虑点 | 方案 | 说明 |
|------|------|------|
| **线程安全** | `std::mutex` + `std::atomic` | 保护共享状态，避免数据竞争 |
| **状态变化检测** | 版本号 | 精确检测哪个状态改变了 |
| **同步频率** | 10-50ms（可配置） | 平衡实时性和功耗 |
| **错误处理** | 重试机制 + 错误日志 | CAN 发送失败时重试 |
| **资源清理** | 析构函数中停止线程 | 避免资源泄漏 |
| **死锁防止** | 使用 `lock_guard` | 自动释放锁，避免死锁 |
| **多电机支持** | 每个电机独立线程 | 支持多个电机并行控制 |
| **性能优化** | 无锁标志位 | 减少锁竞争 |

---

## 使用示例

### 基础使用

```cpp
#include "ele_motor.h"

int main() {
    // 创建电机对象
    EleMotor motor;
    motor.device_idx = 0;
    motor.motor_id = 1;
    
    // 启动后台同步线程
    motor.start_sync();
    
    // 外部代码只需修改目标状态
    motor.set_target_speed(10.0f);      // 设置目标速度
    motor.set_target_position(3.14f);   // 设置目标位置
    
    // 后台线程自动发送 CAN 命令并同步状态
    // 无需手动调用 enable/disable/set_motor_para_bt
    
    // 查询当前状态
    float current_speed = motor.get_current_speed();
    float current_position = motor.get_current_position();
    
    // 停止同步
    motor.stop_sync();
    
    return 0;
}
```

### 多电机控制

```cpp
std::vector<EleMotor> motors(3);

// 初始化所有电机
for (int i = 0; i < 3; i++) {
    motors[i].device_idx = 0;
    motors[i].motor_id = i + 1;
    motors[i].start_sync();
}

// 并行控制多个电机
motors[0].set_target_speed(10.0f);
motors[1].set_target_speed(20.0f);
motors[2].set_target_speed(15.0f);

// 后台线程自动同步所有电机
```

### 配置同步间隔

```cpp
EleMotor motor;
motor.device_idx = 0;
motor.motor_id = 1;

// 设置同步间隔为 10ms（高实时性）
motor.set_sync_interval(10);

motor.start_sync();
```

---

## 优势

✅ **解耦外部调用和 CAN 通信**
- 外部代码只需修改状态
- 不需要关心 CAN 通信细节

✅ **自动状态同步**
- 后台线程自动检测变化
- 自动发送 CAN 命令
- 自动接收反馈

✅ **线程安全**
- 使用互斥锁保护共享状态
- 避免数据竞争和死锁

✅ **易于扩展**
- 支持多电机并行控制
- 支持多种控制模式
- 易于添加新功能

✅ **降低外部代码复杂度**
- API 简洁易用
- 无需手动管理 CAN 通信
- 无需手动处理状态同步

✅ **高实时性**
- 可配置的同步频率
- 快速响应状态变化
- 低延迟反馈

---

## 实现检查清单

- [ ] 添加线程相关成员变量
- [ ] 实现 `start_sync()` 和 `stop_sync()`
- [ ] 实现 `sync_thread_func()` 后台线程
- [ ] 实现 `sync_state()` 状态同步
- [ ] 实现 `has_state_changed()` 变化检测
- [ ] 实现外部接口（`set_target_*`）
- [ ] 实现状态查询接口（`get_current_*`）
- [ ] 添加错误处理和日志
- [ ] 编写单元测试
- [ ] 编写集成测试
- [ ] 性能测试和优化
- [ ] 文档完善

---

## 参考资源

- C++ 线程库：`<thread>`, `<mutex>`, `<atomic>`
- CAN 通信：`BspCan` 硬件抽象层
- 电机控制：`float2bag()`, `set_motor_para_bt()`, `unpack_cmd()`

---

**文档版本：1.0**  
**最后更新：2026-05-21**
