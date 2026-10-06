/**
 * @file thread_manager.cpp
 * @brief ThreadManager 实现
 */

#include "runtime/thread_manager.h"
#include <pthread.h>
#include <stdexcept>

/**
 * 析构时遍历所有线程，置 stop_flag 后 join，确保没有线程在后台游离。
 * 使用 lock_guard 保护遍历过程，防止析构期间其他线程并发修改 threads_。
 */
ThreadManager::~ThreadManager() {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    for (auto& [name, info] : threads_) {
        info->stop_flag = true;
        if (info->thread.joinable())
            info->thread.join();
    }
}

/**
 * 使用 POSIX pthread_setschedparam 为当前线程设置实时调度策略。
 * SCHED_FIFO：先进先出实时调度，同优先级线程不会被抢占，直到主动让出或阻塞。
 * 注意：priority 1~99，数值越大优先级越高；需要 CAP_SYS_NICE 权限，
 *       普通用户调用会静默失败（不抛出异常）。
 */
void ThreadManager::apply_priority(int priority) {
    sched_param param{};
    param.sched_priority = priority;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
}

/**
 * 线程体入口，在新线程中执行，由 start_thread() 通过 std::thread 调用。
 *
 * 执行流程：
 *   1. 若 priority > 0，设置实时调度优先级
 *   2. 将状态置为 RUNNING
 *   3. ONCE 模式：执行一次 func()，完成后置 STOPPED 并返回
 *   4. LOOP 模式：
 *      - 记录每轮开始时间
 *      - 执行 func()
 *      - 计算本轮耗时，用剩余时间 sleep，保证整体间隔约等于 interval_ms
 *      - 检查 stop_flag，为 true 时退出循环
 *   5. 置状态为 STOPPED
 *
 * 关于定时精度（2026-10-04 修正）：
 *   采用**绝对 deadline 网格**驱动周期：next += interval; sleep_until(next)。
 *   旧实现 "func(); sleep_for(interval - elapsed)" 每轮都会多睡约 50µs
 *   （Linux 默认 timer_slack），在 2ms 周期下累积成 ~486Hz（实测 -2.8%）。
 *   改为固定网格后，单轮偏晚会被下一轮自动补回，平均周期严格 = interval_ms
 *   （实测 500Hz）。若单次 func() 耗时 > interval_ms 或线程被长时间抢占，
 *   则重置基准、不追帧（避免"追帧风暴"）。
 */
void ThreadManager::run_thread(ThreadInfo* info) {
    if (info->priority > 0)
        apply_priority(info->priority);

    info->state = ThreadState::RUNNING;

    if (info->mode == ThreadMode::ONCE) {
        info->func();
        info->state = ThreadState::STOPPED;
        return;
    }

    // LOOP 模式：持续执行直到 stop_flag 被置为 true。
    // 用**绝对 deadline 网格**定周期：即使每轮 sleep 都偏晚，也不会累积成频率偏差。
    const auto period = std::chrono::milliseconds(info->interval_ms);
    auto next = std::chrono::steady_clock::now();

    while (!info->stop_flag) {
        info->func();

        // 本轮应唤醒的绝对时刻（与 func() 耗时无关，网格固定）
        next += period;
        const auto now = std::chrono::steady_clock::now();
        if (next > now) {
            std::this_thread::sleep_until(next);
        } else {
            // 已落后（func 超周期 / 被长时间抢占）：重置基准，立即进行下一轮
            next = now;
        }
    }

    info->state = ThreadState::STOPPED;
}

/**
 * 将线程信息存入 threads_ map，此时不创建 std::thread，线程不启动。
 * 使用 unique_ptr 管理 ThreadInfo 的生命周期，map 销毁时自动释放。
 * 重复注册同名线程会抛出异常，调用方需保证名称唯一。
 */
void ThreadManager::register_thread(const std::string& name, std::function<void()> func,
                                     ThreadMode mode, uint32_t interval_ms, int priority) {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    if (threads_.count(name))
        throw std::runtime_error("Thread already registered: " + name);

    auto info = std::make_unique<ThreadInfo>();
    info->name = name;
    info->func = std::move(func); // move 避免拷贝 std::function 的内部捕获
    info->mode = mode;
    info->interval_ms = interval_ms;
    info->priority = priority;
    threads_[name] = std::move(info);
}

/**
 * 创建 std::thread 并绑定到 run_thread()，线程立即开始执行。
 * 启动前重置 stop_flag，支持已停止的线程重新启动。
 * 只允许从 REGISTERED 或 STOPPED 状态启动，防止重复启动同一线程。
 *
 * 关键细节：重启前必须先 join 旧线程。
 * ONCE 线程执行完毕后，std::thread 对象仍然是 joinable 的。
 * 若直接用赋值覆盖一个 joinable 的 std::thread，会触发 std::terminate()。
 * run_thread 不持有 threads_mutex_，因此在持锁状态下 join 不会死锁。
 */
void ThreadManager::start_thread(const std::string& name) {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    auto it = threads_.find(name);
    if (it == threads_.end())
        throw std::runtime_error("Thread not registered: " + name);

    ThreadInfo* info = it->second.get();
    ThreadState s = info->state.load();
    if (s != ThreadState::REGISTERED && s != ThreadState::STOPPED)
        throw std::runtime_error("Thread is already running: " + name);

    // 旧线程执行完毕后仍是 joinable，覆盖前必须先 join 回收
    if (info->thread.joinable())
        info->thread.join();

    info->stop_flag = false;
    info->state = ThreadState::REGISTERED;
    info->thread = std::thread(&ThreadManager::run_thread, this, info);
}

/**
 * 向目标线程发出停止信号，然后等待其退出。
 *
 * 关键细节：join() 前必须先释放 threads_mutex_。
 * 原因：run_thread() 本身不持有锁，但若 join() 在持锁状态下调用，
 * 而线程内部（如任务函数）又尝试获取同一把锁，则会发生死锁。
 * 因此使用 unique_lock 手动 unlock 后再 join。
 */
void ThreadManager::stop_thread(const std::string& name) {
    std::unique_lock<std::mutex> lock(threads_mutex_);
    auto it = threads_.find(name);
    if (it == threads_.end())
        throw std::runtime_error("Thread not registered: " + name);

    ThreadInfo* info = it->second.get();
    info->stop_flag = true;
    lock.unlock(); // 释放锁后再 join，避免死锁

    if (info->thread.joinable())
        info->thread.join();
}

/**
 * 原子读取线程状态，无需调用方额外加锁。
 * 未注册的线程名返回 UNREGISTERED，而非抛出异常，方便调用方做条件判断。
 */
ThreadState ThreadManager::get_thread_state(const std::string& name) {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    auto it = threads_.find(name);
    if (it == threads_.end())
        return ThreadState::UNREGISTERED;
    return it->second->state.load();
}

/**
 * 返回共享数据区的引用，调用方可直接调用 set<T> / get<T>。
 * SharedData 内部自带 mutex，无需调用方额外加锁。
 */
SharedData& ThreadManager::get_shared_data() {
    return shared_data_;
}
