/**
 * @file    motor_manager.h
 * @brief   12电机批量管理器
 * @details MotorManager 是单例，管理 4×3=12 个 EleMotor 实例：
 *          - 4 路 CANET TCP 连接（CAN0~CAN3）
 *          - 每路 3 个电机（motor_id=1/2/3）
 *          - 线程由外部 ThreadManager 统一管理，Initialize() 只负责注册任务函数
 *          - 线程安全：每个电机一把 std::mutex，状态读写加锁
 */
#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include "motor_drive/ele_motor.h"
#include "data_types.h"
#include "thread/thread_manager.h"
#include "bsp/bsp_can.h"

class MotorManager {
public:
    static MotorManager& GetInstance();

    // thread_mgr：外部统一 ThreadManager，线程注册到此处，不在内部启动
    bool Initialize(ThreadManager& thread_mgr);
    void Stop();  // 关闭 CAN 设备（线程由外部 ThreadManager 统一停止）

    void EnableMotor(uint8_t can_port, uint8_t motor_id);
    void DisableMotor(uint8_t can_port, uint8_t motor_id);
    void SetZero(uint8_t can_port, uint8_t motor_id);
    void ClearError(uint8_t can_port, uint8_t motor_id);
    void SendImpedance(uint8_t can_port, uint8_t motor_id,
                       float pos, float vel, float kp, float kd, float torque);
    void SendSpeed(uint8_t can_port, uint8_t motor_id,
                   float vel, float kp, float ki);
    void SendPosition(uint8_t can_port, uint8_t motor_id,
                      float pos, float kvp, float kp, float kd, float kvi);

    MotorStatus GetStatus(uint8_t can_port, uint8_t motor_id) const;

private:
    MotorManager();
    ~MotorManager();

    MotorManager(const MotorManager&) = delete;
    MotorManager& operator=(const MotorManager&) = delete;

    // 任务函数（由外部 ThreadManager 以 LOOP 模式驱动）
    void ReceiveThreadFunc();  // 单次 CAN 轮询，1ms 间隔
    void SendThreadFunc();     // 预留：单次发送，1ms 间隔

    EleMotor m_motors[4][3];
    mutable std::mutex m_motor_mutex[4][3];
};

#endif // MOTOR_MANAGER_H_
