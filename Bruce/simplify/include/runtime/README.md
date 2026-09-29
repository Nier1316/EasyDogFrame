# ThreadManager 使用说明

## 概览

`ThreadManager` 是一个轻量级 C++ 线程管理库，提供：

- 具名线程的注册、启动、停止和状态查询
- ONCE（单次执行）和 LOOP（定时循环）两种执行模式
- Linux 实时调度优先级支持（SCHED_FIFO）
- 线程安全的共享数据区（`SharedData`）
- 析构时自动 join 所有线程，无需手动清理

**依赖**：C++17、pthread（Linux）

---

## 快速开始

```cpp
#include "runtime/thread_manager.h"

ThreadManager mgr;

// 1. 注册线程（此时不启动）
mgr.register_thread("reader", []() {
    // 每 10ms 执行一次的任务
}, ThreadMode::LOOP, 10);

// 2. 启动
mgr.start_thread("reader");

// 3. 停止
mgr.stop_thread("reader");
// 或者：让 mgr 析构，所有线程自动停止
```

---

## API 参考

### register_thread

```cpp
void register_thread(
    const std::string& name,       // 线程唯一名称
    std::function<void()> func,    // 任务函数
    ThreadMode mode,               // ONCE 或 LOOP
    uint32_t interval_ms = 0,      // LOOP 模式下的执行间隔（毫秒）
    int priority = 0               // 实时优先级，0 = 普通调度
);
```

- 同名线程已存在时抛出 `std::runtime_error`
- 注册后线程处于 `REGISTERED` 状态，**不会自动启动**

### start_thread

```cpp
void start_thread(const std::string& name);
```

- 将线程从 `REGISTERED` 或 `STOPPED` 状态启动为 `RUNNING`
- 线程已在运行时抛出异常
- 支持重启：`STOPPED` 状态的线程可以再次 `start_thread`

### stop_thread

```cpp
void stop_thread(const std::string& name);
```

- 对 LOOP 线程：置停止标志，等待当前任务执行完毕后退出
- **阻塞调用**：函数返回时线程已完全退出
- 停止后状态变为 `STOPPED`，可再次启动

### get_thread_state

```cpp
ThreadState get_thread_state(const std::string& name);
```

返回值：

| 状态 | 含义 |
|------|------|
| `UNREGISTERED` | 未注册 |
| `REGISTERED` | 已注册，未启动 |
| `RUNNING` | 运行中 |
| `STOPPED` | 已停止，可重启 |

### get_shared_data

```cpp
SharedData& get_shared_data();
```

返回线程间共享数据区的引用，见下方 [SharedData](#shareddata) 章节。

---

## 执行模式

### ONCE 模式

任务函数执行一次后线程自动退出，状态变为 `STOPPED`。

```cpp
mgr.register_thread("init_task", []() {
    // 只执行一次的初始化逻辑
}, ThreadMode::ONCE);

mgr.start_thread("init_task");
// 任务完成后线程自动退出，无需手动 stop
```

### LOOP 模式

任务函数按 `interval_ms` 间隔反复执行，直到调用 `stop_thread()`。

```cpp
mgr.register_thread("sensor_poll", []() {
    // 每 5ms 执行一次
}, ThreadMode::LOOP, 5);
```

**执行时序**：先执行任务，再等待剩余时间。如果任务本身耗时超过 `interval_ms`，则立即进入下一轮（不会积压）。

```
|-- 任务(2ms) --|-- 等待(3ms) --|-- 任务(2ms) --|-- 等待(3ms) --|
|<-------- 5ms interval ------->|
```

---

## 实时优先级

`priority` 参数使用 Linux `SCHED_FIFO` 调度策略，范围 1~99，数值越大优先级越高。

```cpp
// 高优先级实时线程（需要 root 权限或 CAP_SYS_NICE）
// 本项目电机收发线程实际用 2ms（500Hz）+ 优先级 80
mgr.register_thread("motor_ctrl", motor_func, ThreadMode::LOOP, 2, 80);

// 普通优先级线程
mgr.register_thread("monitor", monitor_func, ThreadMode::LOOP, 100, 0);
```

> **注意**：非 root 用户设置实时优先级会静默失败，线程仍以普通优先级运行。

---

## SharedData

线程安全的键值存储，用于在线程间传递数据。

```cpp
SharedData& shared = mgr.get_shared_data();

// 写入（任意线程）
shared.set<float>("motor_speed", 150.0f);
shared.set<int>("error_code", 0);

// 读取（任意线程）
float speed = shared.get<float>("motor_speed");

// 检查 key 是否存在
if (shared.has("motor_speed")) { ... }
```

**类型安全**：`get<T>` 的类型 `T` 必须与 `set` 时一致，否则抛出 `std::bad_any_cast`。

**典型模式**：一个线程写，另一个线程读。

```cpp
// motor_receive 线程写入（实际节拍 2ms = 500Hz）
mgr.register_thread("motor_receive", [&]() {
    float speed = read_from_can();
    mgr.get_shared_data().set<float>("speed", speed);
}, ThreadMode::LOOP, 2);

// state_calc 线程读取（⚠️ 预留线程，至今未实现，见下文）
mgr.register_thread("state_calc", [&]() {
    float speed = mgr.get_shared_data().get<float>("speed");
    // 使用 speed 进行计算...
}, ThreadMode::LOOP, 5);
```

---

## 析构自动清理

`ThreadManager` 析构时会自动停止并 join 所有仍在运行的线程，无需手动调用 `stop_thread`。

```cpp
{
    ThreadManager mgr;
    mgr.register_thread("worker", worker_func, ThreadMode::LOOP, 10);
    mgr.start_thread("worker");
    // ...
} // mgr 析构，worker 线程自动停止
```

---

## RobotApp 的现状（⚠️ 目前没有任何调用者）

`include/runtime/robot_app.h` 里的 `RobotApp` 设计上是要持有唯一的 `ThreadManager` 实例、统一编排线程：

```
RobotApp
├── ThreadManager thread_mgr_   ← 设计中的唯一实例
├── MotorManager::Initialize(thread_mgr_)  ← 注册 motor_receive / motor_send
└── start() / stop()            ← 设计上统一控制所有线程
```

**但截至 2026-09-29，`RobotApp` 没有任何调用者**：`init()` / `start()` / `stop()` 从未被调用过。
实际运行路径是所有示例各自创建局部 `ThreadManager`，再调用 `MotorManager::Initialize(thread_mgr)`：

```cpp
// 现行实际写法（各 Example 一致，如 ex_rl.cpp / ex_basic.cpp）
MotorManager& motor_mgr = MotorManager::GetInstance();
ThreadManager thread_mgr;                  // 局部实例，示例函数返回时析构
motor_mgr.SetChannelTransport(1, &Usb2CanTransport::GetInstance());  // 可选，须在 Initialize 前
motor_mgr.Initialize(thread_mgr);          // 注册 motor_receive / motor_send
thread_mgr.start_thread("motor_receive");
thread_mgr.start_thread("motor_send");
```

`RobotApp::init()` 里预留的 `state_calc` / `monitor` 两个线程**至今未实现**——`src/runtime/robot_app.cpp`
中两处注册和两处启动仍是注释（`// thread_mgr_.register_thread("state_calc", ...)` 等），
`RobotApp::start()` 只启动 `motor_receive` / `motor_send`。

`src/app/main.cpp` 也**不再引用 `RobotApp`**（原先的全局 `static RobotApp g_app;` 已于 2026-09-29 删除，
它从未被 `init()`/`start()`，且静态析构会触碰已销毁的 `MotorManager` 单例）。
若要启用 `RobotApp`，需要重新在 `main.cpp` 里显式 `RobotApp app; app.init(); app.start();`，
并把上文的 `state_calc` / `monitor` 逻辑补全。

---

## 注意事项

1. **任务函数不要阻塞太久**：`stop_thread` 会等待当前任务执行完毕，任务函数中避免长时间阻塞。
2. **不要在任务函数内调用 `stop_thread` 停止自身**：会导致死锁（线程等待自己 join）。
3. **`SharedData` 存储的是值的副本**：`set` 和 `get` 都是拷贝操作，不适合传递大型对象。
4. **实时优先级需要权限**：生产环境中通过 `setcap` 或 `sudo` 赋予程序 `CAP_SYS_NICE`。
