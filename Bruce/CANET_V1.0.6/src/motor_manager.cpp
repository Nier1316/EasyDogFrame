/**
 * @file   motor_manager.cpp
 * @brief  MotorManager 的实现：设备/电机生命周期管理、命令分发、后台接收线程
 */
#include "motor_manager.h"
#include <cstdio>
#include <cstring>
#include <chrono>

// 单例构造函数：私有，只在 GetInstance 内被调用一次
MotorManager::MotorManager() : m_is_running(false) {
}

// 析构：通过 Shutdown 确保后台线程结束、所有设备关闭
MotorManager::~MotorManager() {
    Shutdown();
}

// C++11 起，静态局部变量的初始化是线程安全的（Magic Statics）
MotorManager& MotorManager::GetInstance() {
    static MotorManager instance;
    return instance;
}

// 初始化：根据配置列表批量创建 CanDevice，再按硬件拓扑创建电机
bool MotorManager::Initialize(const std::vector<CanDeviceConfig>& can_configs) {
    std::lock_guard<std::mutex> lock(m_mutex);

    printf("[INFO] Initializing MotorManager...\n");

    // 1) 为每一条 CAN 配置创建一个 CanDevice 并完成初始化
    //    任何一个设备初始化失败都会中断，但已创建的设备由 unique_ptr 自动释放
    for (const auto& config : can_configs) {
        auto device = std::make_unique<CanDevice>(config.device_idx);
        if (!device->Initialize(config)) {
            printf("[ERROR] Failed to initialize CAN device %d\n", config.device_idx);
            return false;
        }
        m_can_devices[config.device_idx] = std::move(device);
    }

    // 2) 根据硬件拓扑（4 口 × 3 电机）批量创建 Motor 实例
    CreateMotors();

    printf("[INFO] MotorManager initialized with %zu CAN devices and %zu motors\n",
           m_can_devices.size(), m_motors.size());

    return true;
}

// 启动：按序启动所有 CanDevice，然后拉起后台接收线程
bool MotorManager::Start() {
    std::lock_guard<std::mutex> lock(m_mutex);

    printf("[INFO] Starting MotorManager...\n");

    // 启动每一个 CAN 设备；任一失败则整体失败（已启动的设备在 Stop 时统一回滚）
    for (auto& pair : m_can_devices) {
        if (!pair.second->Start()) {
            printf("[ERROR] Failed to start CAN device %d\n", pair.first);
            return false;
        }
    }

    m_is_running = true;

    // 启动后台线程：循环从各 CAN 口收帧并分发到对应 Motor 更新状态
    m_process_thread = std::thread([this]() { ProcessCanData(); });

    printf("[INFO] MotorManager started\n");
    return true;
}

// 停止：先通知后台线程退出，再停所有 CAN 设备
bool MotorManager::Stop() {
    {
        // 仅在设置标志位时加锁，避免持锁 join 引发死锁
        std::lock_guard<std::mutex> lock(m_mutex);
        m_is_running = false;
    }

    // 等待后台线程安全退出（线程循环会检测 m_is_running 并自行 break）
    if (m_process_thread.joinable()) {
        m_process_thread.join();
    }

    // 依次停止所有 CAN 设备
    for (auto& pair : m_can_devices) {
        pair.second->Stop();
    }

    printf("[INFO] MotorManager stopped\n");
    return true;
}

// 按 (can_port, motor_id) 查找电机；找不到返回 nullptr，调用方需判空
Motor* MotorManager::GetMotor(uint8_t can_port, uint8_t motor_id) {
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string key = MakeMotorKey(can_port, motor_id);
    auto it = m_motors.find(key);
    if (it != m_motors.end()) {
        return it->second.get();   // 返回裸指针，所有权仍在 manager
    }
    return nullptr;
}

// 遍历所有电机，返回裸指针列表（仅供观察，不得删除）
std::vector<Motor*> MotorManager::GetAllMotors() {
    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<Motor*> motors;
    for (auto& pair : m_motors) {
        motors.push_back(pair.second.get());
    }
    return motors;
}

uint32_t MotorManager::GetMotorCount() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_motors.size();
}

bool MotorManager::SendMotorCommand(uint8_t can_port, uint8_t motor_id, const MotorCommand& cmd) {
    Motor* motor = GetMotor(can_port, motor_id);
    if (!motor) {
        printf("[ERROR] Motor not found: can_port=%d, motor_id=%d\n", can_port, motor_id);
        return false;
    }

    auto device_it = m_can_devices.find(can_port);
    if (device_it == m_can_devices.end()) {
        printf("[ERROR] CAN device not found: can_port=%d\n", can_port);
        return false;
    }

    // 让 Motor 把命令编码成 CAN 帧（ID 已填为 tx_id），再交给对应 CanDevice 发送
    VCI_CAN_OBJ frame = motor->EncodeCommand(cmd);
    return device_it->second->SendFrame(frame);
}

// 广播：对所有电机执行同一命令；任一失败整体返回 false，但继续发完剩余电机
bool MotorManager::BroadcastCommand(const MotorCommand& cmd) {
    bool success = true;
    auto motors = GetAllMotors();
    for (auto motor : motors) {
        if (!SendMotorCommand(motor->GetCanPort(), motor->GetMotorId(), cmd)) {
            success = false;
        }
    }
    return success;
}

// 查询电机状态；电机不存在时返回"设备离线"而非崩溃
MotorStatus MotorManager::GetMotorStatus(uint8_t can_port, uint8_t motor_id) {
    Motor* motor = GetMotor(can_port, motor_id);
    if (motor) {
        return motor->GetStatus();
    }
    MotorStatus status;
    status.error_code = ERR_DEVICE_OFFLINE;  // 用错误码表达"找不到该电机"
    return status;
}

// 遍历所有电机，收集最新状态快照（调用时会各自加锁）
std::vector<MotorStatus> MotorManager::GetAllMotorStatus() {
    std::vector<MotorStatus> statuses;
    auto motors = GetAllMotors();
    for (auto motor : motors) {
        statuses.push_back(motor->GetStatus());
    }
    return statuses;
}

// 预留的外部触发钩子；当前接收逻辑全部由 ProcessCanData 在后台线程执行
void MotorManager::ProcessReceivedData() {
    // 这个方法由后台线程调用（保留接口，便于未来做单步调试或手动触发）
}

// 完全销毁：先 Stop（结束后台线程并关设备），再清空容器释放所有 unique_ptr
void MotorManager::Shutdown() {
    Stop();

    std::lock_guard<std::mutex> lock(m_mutex);
    m_can_devices.clear();   // unique_ptr 析构 → 调用 CanDevice 的析构 → Shutdown
    m_motors.clear();        // unique_ptr 析构 → Motor 被释放

    printf("[INFO] MotorManager shutdown\n");
}

// 用 "can_port_motor_id" 格式串作为 map 的 key，形式稳定、易于调试
std::string MotorManager::MakeMotorKey(uint8_t can_port, uint8_t motor_id) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%d_%d", can_port, motor_id);
    return std::string(buffer);
}

// 按硬件拓扑批量创建电机：
//   CAN 口 0~3 × 电机 1~3 = 12 个电机
//   rx_id = 50 + motor_id → 51/52/53
//   tx_id = motor_id      → 1/2/3
// 如果硬件拓扑变化（例如每个口 4 个电机），只需改这里的循环范围即可
void MotorManager::CreateMotors() {
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            uint16_t rx_id = 50 + motor_id;   // 51, 52, 53
            uint16_t tx_id = motor_id;        // 1, 2, 3

            auto motor = std::make_unique<Motor>(can_port, motor_id, rx_id, tx_id);
            std::string key = MakeMotorKey(can_port, motor_id);
            m_motors[key] = std::move(motor);
        }
    }
}

// 后台接收线程主循环：
//   轮询每个 CanDevice → 读取所有帧 → 按 (can_port, rx_id) 匹配到 Motor → 更新状态缓存
// 每次轮询后 sleep 10ms，避免空转打满 CPU；灵敏度与 CPU 占用之间的折中
void MotorManager::ProcessCanData() {
    printf("[INFO] CAN data processing thread started\n");

    while (m_is_running) {
        // 依次轮询 4 个 CAN 口；每次最多阻塞 10ms 等待数据
        for (auto& pair : m_can_devices) {
            std::vector<VCI_CAN_OBJ> frames;
            if (pair.second->ReceiveFrames(frames, 10)) {
                // 对每一帧：在所有电机里找到 (can_port 匹配 && rx_id 匹配) 的那一个
                for (const auto& frame : frames) {
                    for (auto& motor_pair : m_motors) {
                        Motor* motor = motor_pair.second.get();
                        if (motor->GetCanPort() == pair.first && motor->GetRxId() == frame.ID) {
                            MotorStatus status = motor->DecodeStatus(frame);
                            motor->UpdateStatus(status);
                            break;   // 已匹配到唯一目标，跳出电机遍历
                        }
                    }
                }
            }
        }

        // 主循环节流；10ms 足够应对大多数电机的状态刷新频率（≤100Hz）
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[INFO] CAN data processing thread stopped\n");
}
