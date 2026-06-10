/**
 * @file    motor_manager.cpp
 * @brief   12电机批量管理器实现
 */
#include "motor_manager.h"
#include "motor_calibration.h"
#include <cstdio>
#include <chrono>

// 单例实现
MotorManager& MotorManager::GetInstance() {
    static MotorManager instance;
    return instance;
}

MotorManager::MotorManager() {
}

MotorManager::~MotorManager() {
    Stop();
}

// 初始化4路 CANET TCP 连接，创建12个电机对象，并向外部 ThreadManager 注册任务函数
bool MotorManager::Initialize(ThreadManager& thread_mgr) {
    printf("[INFO] MotorManager initializing...\n");

    // 1. 通过 BspCan 初始化4路 CAN 设备
    const char* server_ip = "192.168.0.178";
    const uint16_t base_port = 4001;

    for (uint8_t i = 0; i < 4; i++) {
        // 配置 TCP 参数
        CanDeviceConfig config;
        config.device_idx = i;
        config.port = base_port + i;
        config.server_ip = server_ip;
        config.work_mode = TCP_CLIENT;  // 客户端模式

        // 通过 BspCan 初始化设备
        if (!BspCan::GetInstance().InitDevice(i, config)) {
            printf("[ERROR] Failed to initialize CAN device %d\n", i);
            return false;
        }

        // 启动设备
        if (!BspCan::GetInstance().StartDevice(i)) {
            printf("[ERROR] Failed to start CAN device %d\n", i);
            return false;
        }

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

    // 3. 向外部 ThreadManager 注册任务函数（不在此处启动，由 RobotApp 统一启动）
    thread_mgr.register_thread(
        "motor_receive",
        [this]() { ReceiveThreadFunc(); },
        ThreadMode::LOOP, 1, 80  // 1ms 间隔，SCHED_FIFO 优先级 80
    );
    thread_mgr.register_thread(
        "motor_send",
        [this]() { SendThreadFunc(); },
        ThreadMode::LOOP, 1, 80  // 预留，1ms 间隔
    );

    printf("[INFO] MotorManager initialized successfully\n");
    return true;
}

// 关闭所有 CAN 设备（线程由外部 ThreadManager 统一停止，此处不操作线程）
void MotorManager::Stop() {
    printf("[INFO] MotorManager stopping...\n");

    // 通过 BspCan 关闭所有设备
    for (uint8_t i = 0; i < 4; i++) {
        BspCan::GetInstance().StopDevice(i);
        BspCan::GetInstance().CloseDevice(i);
    }

    printf("[INFO] MotorManager stopped\n");
}

// 单次 CAN 轮询，由 ThreadManager 以 LOOP 模式每 1ms 调用一次
void MotorManager::ReceiveThreadFunc() {
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        std::vector<BspCanFrame> frames;
        // 改用 BspCan 接收帧
        if (BspCan::GetInstance().ReceiveFrames(can_port, frames, 0)) {
            for (const auto& frame : frames) {
                if (frame.id >= 51 && frame.id <= 53) {
                    uint8_t motor_id = frame.id - 50;
                    if (motor_id >= 1 && motor_id <= 3) {
                        std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
                        EleMotor& motor = m_motors[can_port][motor_id - 1];
                        // 直接解包 CAN 帧数据
                        unpack_frame(motor, frame.data, frame.dlc);
                    }
                }
            }
        }
    }
}

// 电机控制接口实现
void MotorManager::EnableMotor(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.enable();
    printf("[INFO] Motor enabled: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::DisableMotor(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.disable();
    printf("[INFO] Motor disabled: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::SetZero(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = 0;
    // 发送归零命令
    float2bag(motor, 0.0f, 1, MOTOR_ANGLE_ZERO);
    printf("[INFO] Motor zeroed: CAN%d, motor_id=%d\n", can_port, motor_id);
}

void MotorManager::ClearError(uint8_t can_port, uint8_t motor_id) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.error_code = 0;
    // 发送清错命令
    float2bag(motor, 0.0f, 1, MOTOR_CLEAR_ERROR);
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
    motor.kp = kp;
    motor.kd = kd;
    motor.target_torque = torque;
    motor.control_mode = IMPEDANCE;
}

void MotorManager::SendSpeed(uint8_t can_port, uint8_t motor_id,
                              float vel, float kp, float ki) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_speed = vel;
    motor.kp = kp;
    motor.ki = ki;
    motor.control_mode = SPEED;
}

void MotorManager::SendPosition(uint8_t can_port, uint8_t motor_id,
                                 float pos, float kvp, float kp, float kd, float kvi) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
    EleMotor& motor = m_motors[can_port][motor_id - 1];
    motor.target_position = pos;
    motor.kvp = kvp;
    motor.kp = kp;
    motor.kd = kd;
    motor.ki = kvi;
    motor.control_mode = POSITION;
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

// 单次发送任务，由外部 ThreadManager 以 LOOP 模式驱动
void MotorManager::SendThreadFunc() {
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            std::lock_guard<std::mutex> lock(m_motor_mutex[can_port][motor_id - 1]);
            EleMotor& motor = m_motors[can_port][motor_id - 1];

            if (!motor.enabled) continue;

            // 将上层统一坐标系的目标值逆标定回电机原始坐标系
            float send_pos = motor.target_position;
            float send_vel = motor.target_speed;
            ApplyMotorCalibrationInverse(can_port, motor_id, send_pos, send_vel);

            switch (motor.control_mode) {
                case IMPEDANCE:
                    set_motor_para_bt(motor,
                        send_pos, send_vel,
                        motor.kp, motor.kd, motor.target_torque, IMPEDANCE);
                    break;
                case SPEED:
                    set_motor_para_bt(motor,
                        send_vel, motor.kvp, 0, 0, motor.ki, SPEED);
                    break;
                case POSITION:
                    set_motor_para_bt(motor,
                        send_pos, motor.kvp, motor.kp,
                        motor.kd, motor.ki, POSITION);
                    break;
            }
        }
    }
}
