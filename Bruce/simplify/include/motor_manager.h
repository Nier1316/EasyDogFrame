/**
 * @file    motor_manager.h
 * @brief   12电机批量管理器
 * @details MotorManager 是单例，管理 4×3=12 个 EleMotor 实例：
 *          - 4 路 CANET TCP 连接（CAN0~CAN3）
 *          - 每路 3 个电机（motor_id=1/2/3）
 *          - 后台接收线程：每1ms 轮询4个 CAN 口，按 frame.id - 50 = motor_id 路由帧
 *          - 线程安全：每个电机一把 std::mutex，状态读写加锁
 */
#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "can_device.h"
#include "motor_drive/ele_motor.h"
#include "data_types.h"

class MotorManager {
public:
    // 单例访问
    static MotorManager& GetInstance();

    // 生命周期管理
    bool Initialize();  // 初始化4路 CANET TCP 连接，创建12个电机对象
    void Stop();        // 停止后台线程，关闭设备

    // 电机控制接口
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

    // 状态查询
    MotorStatus GetStatus(uint8_t can_port, uint8_t motor_id) const;

private:
    MotorManager();
    ~MotorManager();

    // 禁止拷贝和移动
    MotorManager(const MotorManager&) = delete;
    MotorManager& operator=(const MotorManager&) = delete;

    // 后台接收线程
    void ReceiveThreadFunc();

    // 成员变量
    std::vector<std::unique_ptr<CanDevice>> m_can_devices;  // 4 路 CAN 设备
    EleMotor m_motors[4][3];                                 // 4 路 × 3 电机
    std::mutex m_motor_mutex[4][3];                          // 每个电机一把锁
    std::thread m_receive_thread;                            // 后台接收线程
    bool m_running;                                          // 线程运行标志
};

#endif // MOTOR_MANAGER_H_
