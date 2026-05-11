/**
 * @file    motor.h
 * @brief   单个电机的驱动类声明
 * @details Motor 类代表一个物理电机，封装了：
 *          1) 基本控制命令接口（SetSpeed/Stop 等）
 *          2) CAN 帧的编码与解码（命令打包 / 状态解析）
 *          3) 线程安全的状态缓存（GetStatus / UpdateStatus）
 *          Motor 本身不直接发送 CAN 帧，实际 IO 由 MotorManager 协调 CanDevice 完成。
 */
#ifndef MOTOR_H_
#define MOTOR_H_

#include <cstdint>
#include <mutex>             // std::mutex：保护状态字段的并发访问
#include "data_types.h"      // MotorCommand / MotorStatus / ErrorCode
#include "bsp/bsp_can.h"     // BspCanFrame：BSP 层的 CAN 帧结构

class Motor {
public:
    /**
     * @param can_port  所属 CAN 口（0~3）
     * @param motor_id  同一 CAN 口内的电机编号（1~3）
     * @param rx_id     接收帧 CAN ID（电机 → 上位机，如 51/52/53）
     * @param tx_id     发送帧 CAN ID（上位机 → 电机，如 1/2/3）
     */
    Motor(uint8_t can_port, uint8_t motor_id, uint16_t rx_id, uint16_t tx_id);
    ~Motor();

    // ---------------- 命令接口（构造命令结构体，返回是否成功）----------------
    bool SetSpeed(uint16_t speed);       // 设置目标转速
    bool SetTorque(uint16_t torque);     // 设置目标扭矩
    bool SetDirection(uint8_t direction);// 设置旋转方向（0=反向，1=正向）
    bool Stop();                         // 立即停止
    bool Reset();                        // 复位（清除故障）
    bool QueryStatus();                  // 主动查询状态

    // ---------------- 状态查询（线程安全，会加锁）----------------
    MotorStatus GetStatus() const;       // 返回当前缓存的完整状态
    bool IsHealthy() const;              // 是否无错误且非故障态
    uint8_t GetErrorCode() const;        // 最近一次错误码
    const char* GetErrorMessage() const; // 错误码对应的中文描述

    // ---------------- 配置只读访问器 ----------------
    uint8_t  GetCanPort() const { return m_can_port; }
    uint8_t  GetMotorId() const { return m_motor_id; }
    uint16_t GetRxId()    const { return m_rx_id; }
    uint16_t GetTxId()    const { return m_tx_id; }

    // ---------------- 供 MotorManager 内部调用（非应用层 API）----------------
    void UpdateStatus(const MotorStatus& status);           // 后台线程收到新状态后更新缓存
    BspCanFrame EncodeCommand(const MotorCommand& cmd);     // 将命令打包为 CAN 帧（ID 用 m_tx_id）
    MotorStatus DecodeStatus(const BspCanFrame& frame);     // 从接收帧解出状态

private:
    uint8_t     m_can_port;        // 所属 CAN 口（0~3），决定走哪个 CanDevice
    uint8_t     m_motor_id;        // 同一 CAN 口内的电机编号（1~3）
    uint16_t    m_rx_id;           // 接收帧 CAN ID（匹配该值的帧归本电机）
    uint16_t    m_tx_id;           // 发送帧 CAN ID（打包命令时填入）
    MotorStatus m_current_status;  // 最新状态缓存，由后台线程更新，应用层读取
    mutable std::mutex m_status_mutex;  // 保护 m_current_status，mutable 以便 const 方法加锁

    // ---------------- 帧数据区（8 字节）的底层打包/解包 ----------------
    void EncodeMotorCommand(uint8_t* data, const MotorCommand& cmd);   // 命令 → 8 字节
    void DecodeMotorStatus(const uint8_t* data, MotorStatus& status);  // 8 字节 → 状态
};

#endif // MOTOR_H_
