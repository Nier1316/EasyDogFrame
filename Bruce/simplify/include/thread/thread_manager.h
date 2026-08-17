/**
 * @file thread_manager.h
 * @brief 线程管理库
 *
 * 提供线程的注册、启动、停止、状态查询，以及线程间共享数据区。
 *
 * 使用流程：
 *   1. 创建 ThreadManager 实例
 *   2. 调用 register_thread() 注册线程（此时线程不启动）
 *   3. 调用 start_thread() 启动指定线程
 *   4. 通过 get_shared_data() 在线程间交换数据
 *   5. 调用 stop_thread() 停止线程，或析构时自动停止所有线程
 *
 * 示例：
 *   ThreadManager mgr;
 *   mgr.register_thread("reader", []() { ... }, ThreadMode::LOOP, 10);
 *   mgr.start_thread("reader");
 *   mgr.get_shared_data().set<float>("speed", 100.0f);
 */

#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include <thread>
#include <mutex>
#include <functional>
#include <atomic>
#include <string>
#include <unordered_map>
#include <any>
#include <memory>
#include <stdexcept>
#include <chrono>

/**
 * @brief 线程执行模式
 */
enum class ThreadMode {
    ONCE,  // 任务函数执行一次后线程自动退出
    LOOP   // 任务函数按 interval_ms 间隔反复执行，直到调用 stop_thread()
};

/**
 * @brief 线程生命周期状态
 *
 * 状态转移：
 *   UNREGISTERED → REGISTERED（register_thread 后）
 *   REGISTERED   → RUNNING   （start_thread 后）
 *   RUNNING      → STOPPED   （stop_thread 或任务执行完毕后）
 *   STOPPED      → RUNNING   （再次 start_thread 后）
 */
enum class ThreadState {
    UNREGISTERED,  // 未在 ThreadManager 中注册
    REGISTERED,    // 已注册，尚未启动
    RUNNING,       // 线程正在运行
    STOPPED        // 线程已停止（可重新启动）
};

/**
 * @brief 线程安全的共享数据区
 *
 * 使用 std::any 存储任意类型数据，以字符串 key 索引。
 * 所有读写操作都通过内部 mutex 保护，可在多个线程中安全调用。
 *
 * 示例：
 *   SharedData shared;
 *   shared.set<float>("speed", 100.0f);   // 写入
 *   float v = shared.get<float>("speed"); // 读取
 */
class SharedData {
private:
    std::unordered_map<std::string, std::any> data_; // 数据存储
    mutable std::mutex mutex_;                        // 保护 data_ 的互斥锁

public:
    /**
     * @brief 写入数据
     * @tparam T   数据类型（自动推导）
     * @param key  字符串键名
     * @param value 要写入的值
     */
    template<typename T>
    void set(const std::string& key, const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_[key] = value;
    }

    /**
     * @brief 读取数据
     * @tparam T   期望的数据类型，必须与 set 时的类型一致，否则抛出 std::bad_any_cast
     * @param key  字符串键名
     * @return     存储的值
     * @throws std::runtime_error  key 不存在时抛出
     * @throws std::bad_any_cast   类型不匹配时抛出
     */
    template<typename T>
    T get(const std::string& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = data_.find(key);
        if (it == data_.end())
            throw std::runtime_error("SharedData: key not found: " + key);
        return std::any_cast<T>(it->second);
    }

    /**
     * @brief 检查 key 是否存在
     */
    bool has(const std::string& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.count(key) > 0;
    }
};

/**
 * @brief 单个线程的完整信息（ThreadManager 内部使用，外部不直接操作）
 */
struct ThreadInfo {
    std::string name;              // 线程名称，唯一标识
    std::function<void()> func;    // 线程任务函数
    ThreadMode mode;               // 执行模式：ONCE 或 LOOP
    uint32_t interval_ms;          // LOOP 模式下的执行间隔（毫秒）
    int priority;                  // 0 = 普通调度；1~99 = SCHED_FIFO 实时优先级（需 root）
    std::thread thread;            // 底层 std::thread 对象
    std::atomic<ThreadState> state{ThreadState::REGISTERED}; // 当前状态（原子，线程安全）
    std::atomic<bool> stop_flag{false}; // LOOP 模式退出信号，置 true 后线程在当次任务完成后退出
};

/**
 * @brief 线程管理器
 *
 * 管理多个具名线程的生命周期，支持注册、启动、停止和状态查询。
 * 析构时自动停止并 join 所有仍在运行的线程，无需手动清理。
 *
 * 线程安全：所有公开接口通过 threads_mutex_ 保护，可在多线程环境中调用。
 *
 * 典型用法：
 *   ThreadManager mgr;
 *   mgr.register_thread("io",   io_func,   ThreadMode::LOOP, 10);
 *   mgr.register_thread("calc", calc_func, ThreadMode::LOOP, 5);
 *   mgr.start_thread("io");
 *   mgr.start_thread("calc");
 *   // ... 运行中 ...
 *   mgr.stop_thread("io");
 */
class ThreadManager {
private:
    // 以线程名称为 key 存储所有已注册线程的信息，unique_ptr 保证内存由 map 独占管理
    std::unordered_map<std::string, std::unique_ptr<ThreadInfo>> threads_;
    std::mutex threads_mutex_; // 保护 threads_ 的互斥锁

    SharedData shared_data_;   // 线程间共享数据区

    /**
     * @brief 线程体入口，由 start_thread() 在新线程中调用
     *
     * 负责设置优先级、驱动任务循环、更新线程状态。
     * @param info 裸指针，生命周期由 threads_ 中的 unique_ptr 保证，调用期间一定有效
     */
    void run_thread(ThreadInfo* info);

    /**
     * @brief 为当前线程设置 Linux 实时调度优先级（SCHED_FIFO）
     * @param priority 优先级 1~99，需要 CAP_SYS_NICE 权限，否则静默失败
     */
    void apply_priority(int priority);

public:
    ThreadManager() = default;

    /** @brief 析构时自动 stop 并 join 所有仍在运行的线程 */
    ~ThreadManager();

    /**
     * @brief 注册线程（不启动）
     * @param name        线程唯一名称，重复注册抛出异常
     * @param func        任务函数，支持 lambda / 函数指针 / std::bind
     * @param mode        ONCE（执行一次）或 LOOP（按间隔循环）
     * @param interval_ms LOOP 模式下每轮间隔（毫秒），默认 0
     * @param priority    实时优先级，默认 0（普通调度）
     * @throws std::runtime_error 同名线程已存在
     */
    void register_thread(const std::string& name, std::function<void()> func,
                         ThreadMode mode, uint32_t interval_ms = 0, int priority = 0);

    /**
     * @brief 启动已注册的线程
     * @param name 目标线程名称
     * @throws std::runtime_error 线程未注册，或当前状态不可启动（已在运行）
     */
    void start_thread(const std::string& name);

    /**
     * @brief 请求停止线程，阻塞等待其退出
     *
     * 对 LOOP 线程：置 stop_flag，等待当前任务执行完毕后退出。
     * 对 ONCE 线程：任务已在运行则等待其自然结束。
     * @param name 目标线程名称
     * @throws std::runtime_error 线程未注册
     */
    void stop_thread(const std::string& name);

    /**
     * @brief 查询线程当前状态
     * @return 对应的 ThreadState；未注册时返回 UNREGISTERED
     */
    ThreadState get_thread_state(const std::string& name);

    /**
     * @brief 获取线程间共享数据区的引用
     * @return SharedData&，可在任意线程中调用 set<T> / get<T>
     */
    SharedData& get_shared_data();
};

#endif // THREAD_MANAGER_H
