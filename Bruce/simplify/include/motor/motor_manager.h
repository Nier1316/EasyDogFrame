/**
 * @file    motor_manager.h
 * @brief   16电机批量管理器
 * @details MotorManager 是单例，管理 4×4=16 个 EleMotor 实例：
 *          - 4 路 CANET TCP 连接（CAN0~CAN3）
 *          - 每路 4 个电机（motor_id=1/2/3/4，1-3 腿关节 + 4 轮电机）
 *          - 线程由外部 ThreadManager 统一管理，Initialize() 只负责注册任务函数
 *          - 线程安全：每个电机一把 std::mutex，状态读写加锁
 */
#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_

#include <cstdint>
#include <mutex>
#include "motor/ele_motor.h"
#include "motor/motor_calibration.h"
#include "common/types.h"
#include "runtime/thread_manager.h"
#include "transport/can_transport.h"

class MotorManager {
public:
    static MotorManager& GetInstance();

    // thread_mgr：外部统一 ThreadManager，线程注册到此处，不在内部启动
    bool Initialize(ThreadManager& thread_mgr);
    void Stop();  // 关闭 CAN 设备（线程由外部 ThreadManager 统一停止）

    /**
     * 注入传输后端（默认 CanetTransport）。换硬件（如 USB2CAN）时调用，
     * 需在 Initialize() 之前。上层代码只依赖 CanTransport 接口。
     */
    void SetTransport(CanTransport* transport);

    // 在使能之前写固件控制模式（IMPEDANCE/SPEED/POSITION）。
    // 必须在 EnableMotor 之前调用：发送线程会跳过未使能的电机，
    // 单靠 SendSpeed/SendImpedance 设字段，模式帧要等使能后才发得出去，
    // 电机会先在固件默认模式下被使能而意外运动。
    void SetControlMode(uint8_t can_port, uint8_t motor_id, int mode);

    // 读固件参数寄存器（type 见 ele_motor_def.h 的 MOTOR_OR_* / MOTOR_WR_*）。
    // 异步：回帧由接收线程打印 "[PARAM] CANx motory type=0xNN value=..."。
    // 使能前也可调用，用于核对固件量程与上电初始状态。
    void ReadParam(uint8_t can_port, uint8_t motor_id, uint8_t type);

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

    // ============ 单次 IO（线程注册移至 runtime/motor_io.h）============
    // MotorManager 只提供单次轮询，不拥有线程生命周期；线程由 runtime 层
    // RegisterMotorIoThreads() 注册到 ThreadManager 驱动。

    /** 单次 CAN 接收轮询（10ms 节拍，对齐 SDK VCI_Receive 粒度） */
    void ReceiveOnce();

    /** 单次发送轮询（1ms 节拍，把 target 字段编帧发出） */
    void SendOnce();

private:
    MotorManager();
    ~MotorManager();

    MotorManager(const MotorManager&) = delete;
    MotorManager& operator=(const MotorManager&) = delete;

    EleMotor m_motors[CAN_PORTS][MOTORS_PER_CAN];
    mutable std::mutex m_motor_mutex[CAN_PORTS][MOTORS_PER_CAN];
    CanTransport* m_transport = nullptr;   // 传输后端（默认 CanetTransport，Initialize 时兜底）
    bool m_initialized = false;   // Initialize() 全部成功后才置真，Stop() 据此避免关未打开的设备
};

#endif // MOTOR_MANAGER_H_
