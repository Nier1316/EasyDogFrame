# Thread Manager 设计计划

## 注册线程函数入参

```cpp
void register_thread(
    const std::string& name,          // 线程名称，用于标识和管理
    std::function<void()> func,       // 线程执行的任务函数
    ThreadMode mode,                  // 执行模式：ONCE（单次）或 LOOP（循环）
    uint32_t interval_ms,             // 循环间隔（ms），ONCE模式下忽略
    int priority                      // 线程优先级（Linux: SCHED_FIFO 1~99，0为普通）
);
```

### 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `std::string` | 线程唯一标识，用于后续查找/停止/监控 |
| `func` | `std::function<void()>` | 线程体，支持 lambda、函数指针、成员函数 |
| `mode` | `ThreadMode` | `ONCE`：执行一次后退出；`LOOP`：按间隔反复执行 |
| `interval_ms` | `uint32_t` | LOOP模式下每次执行的间隔时间（毫秒） |
| `priority` | `int` | 0 = 普通优先级；1~99 = 实时优先级（需要root权限） |

---

## 接口设计

注册与启动分离，完整接口如下：

```cpp
// 注册线程（不启动）
void register_thread(const std::string& name, std::function<void()> func,
                     ThreadMode mode, uint32_t interval_ms = 0, int priority = 0);

// 启动指定线程
void start_thread(const std::string& name);

// 停止指定线程
void stop_thread(const std::string& name);

// 查询线程状态
ThreadState get_thread_state(const std::string& name);

// 共享数据区（线程安全）
SharedData& get_shared_data();
```

### ThreadState 枚举

```cpp
enum class ThreadState {
    UNREGISTERED,  // 未在 ThreadManager 中注册
    REGISTERED,    // 已注册，尚未启动
    RUNNING,       // 线程正在运行
    STOPPED        // 线程已停止（可重新启动）
};
```

> 注：早期计划里的 `ThreadState::ERROR` **已删除**（`include/runtime/thread_manager.h` 当前只有上述四态）。
> 状态转移：`UNREGISTERED → REGISTERED`（register_thread）→ `RUNNING`（start_thread）
> → `STOPPED`（stop_thread 或任务执行完毕）→ `RUNNING`（再次 start_thread）。
> 未注册的线程名，`get_thread_state()` 返回 `UNREGISTERED`。

### 共享数据区

库内提供一个线程安全的 key-value 容器，线程间通过它交换数据：

```cpp
// 写入数据（任意线程）
shared.set<float>("motor_speed", 100.0f);

// 读取数据（任意线程）
float speed = shared.get<float>("motor_speed");
```

---

## 待讨论

- [x] 共享数据区：模板 map + mutex，接口为 `set<T>(key, val)` / `get<T>(key)`
