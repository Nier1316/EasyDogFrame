/**
 * @file    motor_manager.cpp
 * @brief   12电机批量管理器实现
 */
#include "motor_manager.h"
#include <cstdio>
#include <chrono>

// 单例实现
MotorManager& MotorManager::GetInstance() {
    static MotorManager instance;
    return instance;
}

MotorManager::MotorManager()
    : m_running(false) {
}

MotorManager::~MotorManager() {
    Stop();
}

// 初始化4路 CANET TCP 连接，创建12个电机对象
bool MotorManager::Initialize() {
    printf("[INFO] MotorManager initializing...\n");

    // 1. 创建并初始化4路 CAN 设备
    const char* server_ip = "192.168.0.178";
    const uint16_t base_port = 4001;

    for (uint8_t i = 0; i < 4; i++) {
        auto can_device = std::make_unique<CanDevice>(i);

        // 配置 TCP 参数
        CanDeviceConfig config;
        config.device_idx = i;
        config.port = base_port + i;
        config.server_ip = server_ip;
        config.work_mode = TCP_CLIENT;  // 客户端模式

        // 初始化设备
        if (!can_device->Initialize(config)) {
            printf("[ERROR] Failed to initialize CAN device %d\n", i);
            return false;
        }

        // 启动设备
        if (!can_device->Start()) {
            printf("[ERROR] Failed to start CAN device %d\n", i);
            return false;
        }

        m_can_devices.push_back(std::move(can_device));
        printf("[INFO] CAN device %d initialized and started\n", i);
    }

    // 2. 创建12个电机对象，配置参数
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            EleMotor& motor = m_motors[can_port][motor_id - 1];
            motor.device_idx = can_port;
            motor.motor_id = motor_id;
            motor.enabled = false;
            motor.error_code = 0;
            motor.current_speed = 0;
            motor.current_torque = 0;
            motor.current_position = 0;
            motor.current_temp = 0;
            motor.target_speed = 0;
            motor.target_torque = 0;
            motor.target_position = 0;
            printf("[INFO] Motor created: CAN%d, motor_id=%d\n", can_port, motor_id);
        }
    }

    // 3. 启动后台接收线程
    m_running = true;
    m_receive_thread = std::thread(&MotorManager::ReceiveThreadFunc, this);
    printf("[INFO] Receive thread started\n");

    printf("[INFO] MotorManager initialized successfully\n");
    return true;
}

// 停止后台线程，关闭设备
void MotorManager::Stop() {
    printf("[INFO] MotorManager stopping...\n");

    // 1. 停止后台线程
    m_running = false;
    if (m_receive_thread.joinable()) {
        m_receive_thread.join();
        printf("[INFO] Receive thread stopped\n");
    }

    // 2. 关闭所有 CAN 设备
    for (auto& can_device : m_can_devices) {
        if (can_device) {
            can_device->Shutdown();
        }
    }
    m_can_devices.clear();

    printf("[INFO] MotorManager stopped\n");
}

// 后台接收线程：每1ms 轮询4个 CAN 口，按 frame.id - 50 = motor_id 路由帧
void MotorManager::ReceiveThreadFunc() {
    printf("[INFO] Receive thread running\n");

    while (m_running) {
        // 轮询4个 CAN 口
        for (uint8_t can_port = 0; can_port < 4; can_port++) {
            if (!m_can_devices[can_port]) {
                continue;
            }

            std::vector<VCI_CAN_OBJ> frames;
            if (m_can_devices[can_port]->ReceiveFrames(frames, 0)) {
                // 处理接收到的帧
                for (const auto& frame : frames) {
                    // 按 frame.id - 50 = motor_id 路由帧
                    if (frame.ID >= 51 && frame.ID <= 53) {
                        uint8_t motor_id = frame.ID - 50;
                        if (motor_id >= 1 && motor_id <= 3) {
                            EleMotor& motor = m_motors[can_port][motor_id - 1];
                            std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
                            // 解包电机状态（这里需要调用 unpack_cmd 或类似函数）
                            // 暂时留空，等待后续实现
                        }
                    }
                }
            }
        }

        // 每1ms 轮询一次
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// 电机控制接口实现
void MotorManager::EnableMotor(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.enabled = true;
    printf("[INFO] Motor enabled: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::DisableMotor(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.enabled = false;
    printf("[INFO] Motor disabled: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::SetZero(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = 0;
    printf("[INFO] Motor zeroed: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::ClearError(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.error_code = 0;
    printf("[INFO] Motor error cleared: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::SendImpedance(uint8_t can_port, uint8_t motor_id,
                                  float pos, float vel, float kp, float kd, float torque) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = pos;
    motor.target_speed = vel;
    // 发送 CAN 帧（这里需要调用 set_motor_para_bt 或类似函数）
}

void MotorManager::SendSpeed(uint8_t can_port, uint8_t motor_id,
                              float vel, float kp, float ki) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_speed = vel;
    // 发送 CAN 帧（这里需要调用 set_motor_para_bt 或类似函数）
}

void MotorManager::SendPosition(uint8_t can_port, uint8_t motor_id,
                                 float pos, float kvp, float kp, float kd, float kvi) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = pos;
    // 发送 CAN 帧（这里需要调用 set_motor_para_bt 或类似函数）
}

MotorStatus MotorManager::GetStatus(uint8_t can_port, uint8_t motor_id) const {
    MotorStatus status;
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return status;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    const EleMotor& motor = m_motors[can_port][motor_id - 1];
    status.motor_id = motor.motor_id;
    status.enable = motor.enabled;
    status.position = motor.current_position;
    status.velocity = motor.current_speed;
    status.torque = motor.current_torque;
    status.error_code = motor.error_code;
    return status;
}
