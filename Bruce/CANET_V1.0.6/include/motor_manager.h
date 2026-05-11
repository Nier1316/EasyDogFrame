/**
 * @file    motor_manager.h
 * @brief   电机与 CAN 设备的统一管理层（单例）
 * @details MotorManager 是框架的核心协调者，负责：
 *          1) 创建并持有全部 CanDevice 和 Motor 实例
 *          2) 接收 MotorController 下发的命令，找到目标 CanDevice 完成发送
 *          3) 在后台线程中轮询所有 CanDevice，收到帧后分发到对应 Motor
 *          4) 提供按 (can_port, motor_id) 索引电机和状态的查询接口
 *          采用单例模式，保证整个进程中只有一份电机拓扑视图。
 */
#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_

#include <cstdint>
#include <map>            // 以 device_idx 和 motor_key 为索引存储设备/电机
#include <vector>
#include <memory>         // std::unique_ptr：自动管理设备/电机生命周期
#include <mutex>
#include <thread>         // 后台数据处理线程
#include "can_device.h"
#include "motor.h"
#include "data_types.h"

class MotorManager {
public:
    // 单例入口：全局唯一实例，避免多处重复创建导致资源冲突
    static MotorManager& GetInstance();

    // ---------------- 初始化 / 运行控制 ----------------
    // 根据配置批量创建 CanDevice，并自动建立 4 个 CAN 口 × 3 个电机的默认拓扑
    bool Initialize(const std::vector<CanDeviceConfig>& can_configs);
    bool Start();                                   // 启动所有 CAN 设备 + 后台接收线程
    bool Stop();                                    // 停线程 + 停所有 CAN 设备
    bool IsRunning() const { return m_is_running; } // 是否整体在运行

    // ---------------- 电机访问 ----------------
    // 按 (can_port, motor_id) 查找电机，找不到返回 nullptr
    Motor* GetMotor(uint8_t can_port, uint8_t motor_id);
    std::vector<Motor*> GetAllMotors();             // 返回全部电机指针列表
    uint32_t GetMotorCount() const;                 // 当前管理的电机总数

    // ---------------- 命令下发 ----------------
    // 定向发送命令：编码成帧后由对应 CanDevice 发出
    bool SendMotorCommand(uint8_t can_port, uint8_t motor_id, const MotorCommand& cmd);
    // 广播命令：对所有电机执行同一命令（例如全局 STOP）
    bool BroadcastCommand(const MotorCommand& cmd);

    // ---------------- 状态查询 ----------------
    MotorStatus GetMotorStatus(uint8_t can_port, uint8_t motor_id);  // 单个电机
    std::vector<MotorStatus> GetAllMotorStatus();                    // 全部电机

    // 后台线程入口（公开只是为了可测试，正常不由应用层调用）
    void ProcessReceivedData();

    void Shutdown();  // 彻底释放资源（Stop + clear 所有容器）

private:
    MotorManager();   // 私有构造，强制走 GetInstance
    ~MotorManager();

    // 禁用拷贝与赋值，单例不允许复制
    MotorManager(const MotorManager&) = delete;
    MotorManager& operator=(const MotorManager&) = delete;

    // 以 device_idx 为 key 存 CAN 设备；unique_ptr 自动释放
    std::map<uint8_t, std::unique_ptr<CanDevice>> m_can_devices;
    // 以 "can_port_motor_id" 字符串为 key 存电机，支持 O(log n) 查找
    std::map<std::string, std::unique_ptr<Motor>> m_motors;
    std::thread m_process_thread;   // 后台接收/分发线程
    bool m_is_running;              // 运行标志；线程循环的退出条件
    mutable std::mutex m_mutex;     // 保护 m_can_devices / m_motors / m_is_running

    // ---------------- 内部工具方法 ----------------
    std::string MakeMotorKey(uint8_t can_port, uint8_t motor_id); // 生成 map 的 key
    void CreateMotors();          // 根据硬件拓扑批量创建 12 个电机
    void ProcessCanData();        // 后台线程主循环：收帧 → 分发到 Motor
};

#endif // MOTOR_MANAGER_H_
