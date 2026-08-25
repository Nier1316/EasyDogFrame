/**
 * @file    shared_data.h
 * @brief   线程安全的共享数据区（L0 公共基础）
 * @details 从 runtime/thread_manager.h 移出（阶段5）：共享数据是全局公共能力，
 *          不属于线程管理器本身。ThreadManager 持有并暴露它。
 *          使用 std::any 存储任意类型，以字符串 key 索引；所有读写加锁。
 *
 * 示例：
 *   SharedData shared;
 *   shared.set<float>("speed", 100.0f);
 *   float v = shared.get<float>("speed");
 */
#ifndef SHARED_DATA_H_
#define SHARED_DATA_H_

#include <string>
#include <unordered_map>
#include <any>
#include <mutex>
#include <stdexcept>

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

    /** @brief 检查 key 是否存在 */
    bool has(const std::string& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.count(key) > 0;
    }
};

#endif // SHARED_DATA_H_
