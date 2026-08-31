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
#include <atomic>
#include <chrono>
#include "motor/ele_motor.h"
#include "motor/motor_calibration.h"
#include "common/types.h"
#include "runtime/thread_manager.h"
#include "transport/can_transport.h"

class MotorManager {
public:
    static MotorManager& GetInstance();

    // 接收线程心跳（诊断"接收链路挂掉"）：ReceiveOnce 每次进入更新。
    // s_start 在 Initialize 时置为当前时刻，心跳值 = 距启动毫秒数。
    inline static std::atomic<int64_t> s_recv_heartbeat_ms{-1};
    inline static std::chrono::steady_clock::time_point s_start{
        std::chrono::steady_clock::now()};

    // thread_mgr：外部统一 ThreadManager，线程注册到此处，不在内部启动
    bool Initialize(ThreadManager& thread_mgr);
    void Stop();  // 关闭 CAN 设备（线程由外部 ThreadManager 统一停止）

    /**
     * 注入传输后端（默认 CanetTransport）。换硬件（如 USB2CAN）时调用，
     * 需在 Initialize() 之前。上层代码只依赖 CanTransport 接口。
     */
    void SetTransport(CanTransport* transport);

    /**
     * 覆盖指定 CAN 路（can_port）的传输后端，其余路保持默认。
     * 例：只把 CAN1 换达妙 USB2CAN：
     *   mm.SetChannelTransport(1, &Usb2CanTransport::GetInstance());
     * 需在 Initialize() 之前调用。
     */
    void SetChannelTransport(uint8_t can_port, CanTransport* transport);

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

    /**
     * 使能前预置零扭矩阻抗控制帧（直发，不受 send 线程 enabled 检查限制）。
     * 覆盖固件残留目标（上一次命令/上电默认），使能瞬间零扭矩不冲。
     * 用法：SetControlMode → PreEnableZeroTorque → EnableMotor。
     * 使能后由后续正常增益的控制帧接管。
     */
    void PreEnableZeroTorque(uint8_t can_port, uint8_t motor_id);

    void SendImpedance(uint8_t can_port, uint8_t motor_id,
                       float pos, float vel, float kp, float kd, float torque);
    void SendSpeed(uint8_t can_port, uint8_t motor_id,
                   float vel, float kp, float ki);
    void SendPosition(uint8_t can_port, uint8_t motor_id,
                      float pos, float kvp, float kp, float kd, float kvi);

    MotorStatus GetStatus(uint8_t can_port, uint8_t motor_id) const;

    // ============ 轮子急停（安全，2026-08-29） ============

    /**
     * 触发轮子急停：置急停标志 + 直发 SPEED 0 速 + 制动增益帧。
     * 触发后 SendOnce 每周期对 4 个轮子强制发 0 速制动帧（忽略上层目标），
     * 直到 ClearWheelEmergency() 解除。轮速超 15 rad/s 也会自动置位。
     * 制动增益 WHEEL_ESTOP_KVP（默认 3.0）= Example23 验证过的控轮增益，
     * 速度反馈正常时固件速度环闭环到 0 速（主动制动，非自由滑行）。
     */
    void WheelEmergencyStop();

    /** 解除轮子急停，恢复上层控制。 */
    void ClearWheelEmergency();

    /** 查询是否处于轮子急停状态（超速自动触发 / 手动触发）。 */
    bool wheelEmergency() const { return m_wheel_estop.load(); }

    // ============ 单次 IO（线程注册移至 runtime/motor_io.h）============
    // MotorManager 只提供单次轮询，不拥有线程生命周期；线程由 runtime 层
    // RegisterMotorIoThreads() 注册到 ThreadManager 驱动。

    /** 单次 CAN 接收轮询（10ms 节拍，对齐 SDK VCI_Receive 粒度） */
    void ReceiveOnce();

    /** 单次发送轮询（1ms 节拍，把 target 字段编帧发出） */
    void SendOnce();

    /** 诊断：ReceiveOnce 最近一次心跳（程序启动后毫秒）。-1 = 从未运行。
     *  主循环用它检测接收线程是否卡死：心跳长时间不前进 + Usb2CanTransport::RxCount() 仍在涨
     *  → 接收线程卡死；RxCount() 也停 → SDK 回调停（USB 接收链路挂）。 */
    static int64_t GetReceiveHeartbeatMs() { return s_recv_heartbeat_ms.load(); }
    static void TouchReceiveHeartbeat() { s_recv_heartbeat_ms.store(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - s_start).count()); }
    static std::atomic<int64_t>& RecvHeartbeat() { return s_recv_heartbeat_ms; }

private:
    MotorManager();
    ~MotorManager();

    MotorManager(const MotorManager&) = delete;
    MotorManager& operator=(const MotorManager&) = delete;

    EleMotor m_motors[CAN_PORTS][MOTORS_PER_CAN];
    mutable std::mutex m_motor_mutex[CAN_PORTS][MOTORS_PER_CAN];
    CanTransport* m_transport[CAN_PORTS] = {nullptr};  // 每路传输后端（Initialize 时默认 Usb2CanTransport，见 motor_manager.cpp）
    const char* m_usb_dev  = "/dev/ttyACM0";   // USB2CAN 设备路径（达妙模块）
    uint8_t     m_usb_baud = 0;                // USB2CAN 波特率索引（0=1000k, 3=500k）
    bool m_initialized = false;   // Initialize() 全部成功后才置真，Stop() 据此避免关未打开的设备
    std::atomic<bool> m_wheel_estop{false};    // 轮子急停标志（手动 WheelEmergencyStop / 超速自动置位）
};

#endif // MOTOR_MANAGER_H_
