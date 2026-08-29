#ifndef ELE_MOTOR_H
#define ELE_MOTOR_H

#include <mutex>
#include <cstdint>
#include "motor/ele_motor_def.h"

class CanTransport;   // 传输后端（由 MotorManager 按 channel 注入，CANET/USB2CAN 通用）

class EleMotor {
private:
    std::mutex state_mutex;  // 保护状态字段的并发读写

public:
    EleMotor() = default;
    ~EleMotor() = default;

    // 传输后端（MotorManager::Initialize 创建电机时注入 m_transport[device_idx]）。
    // 发送走它 → 换硬件（CANET/USB2CAN）不动电机编解码层。nullptr 时发送静默丢弃。
    CanTransport* transport = nullptr;

    // 配置信息
    uint8_t device_idx;       // CAN设备索引
    uint8_t motor_id;         // 电机ID

    // 当前状态
    float current_speed;      // 当前速度 (rad/s，标定后)
    float current_torque;     // 当前扭矩 (Nm)
    float current_position;   // 当前位置 (rad)
    float current_temp;       // 当前温度 (°C)

    // 轮速低通滤波状态（2026-08-28）：USB2CAN 丢帧/错帧会导致轮速解码跳变
    // （如 FR 轮瞬时 ±4~8 rad/s，物理不可能），跳变会诱导策略过激响应 + 轮控误算。
    // unpack 更新 current_speed 时对轮子做一阶低通，策略观测与 SendOnce 轮控共用滤波值。
    float vel_lp = 0.0f;      // 滤波后轮速（低通状态）
    bool  vel_lp_init = false; // 是否已用首帧对齐

    // 目标状态
    float target_speed;       // 目标速度 (rpm)
    float target_torque;      // 目标扭矩 (Nm)
    float target_position;    // 目标位置 (degree)

    // 控制模式和增益参数
    int control_mode;         // 期望控制模式 (IMPEDANCE/SPEED/POSITION)
    // 电机固件侧当前生效的控制模式。-1 = 未知/未同步。
    // set_motor_para_bt 的字节布局随模式而变，若固件模式与 control_mode 不一致，
    // 电机会按错误的布局解释帧（例如把零速指令读成 -65rad/s + 满负前馈扭矩）。
    // 因此下发控制帧前必须先用 float2bag(.., MOTOR_WR_CONTROL_MODE) 同步固件模式。
    int hw_control_mode;
    int mode_settle_ticks;    // 写模式后暂停下发控制帧的剩余周期数（等固件生效）
    float kp;                 // 刚度/位置环Kp
    float kd;                 // 阻尼/位置环Kd
    float ki;                 // 速度环Ki
    float kvp;                // 位置控制速度环Kp

    // 状态标志
    uint16_t error_code;      // 错误码
    bool enabled;             // 使能状态

    // 轮子自动急停静默窗口（2026-08-29）：使能后剩余周期数（≈2s）。
    // 背景：CAN1 轮使能瞬间速度反馈假偏移 +44 rad/s（固件初始化冲击），若自动急停
    // 立即按此触发会误锁轮子（030148 日志：kvp 扫描全程被劫持，轮几乎不动）。
    // 窗口内不自动触发急停，靠软启动低 kvp 扛住假偏移；手动 WheelEmergencyStop 不受影响。
    int32_t estop_grace_ticks = 0;

    void init();
    void enable();
    void disable();

};

float uint_to_float(int x_int, float x_min, float x_max, int bits);
unsigned int float_to_uint(float x, float x_min, float x_max, int bits);

// 电机参数读写
void float2bag(const EleMotor& motor, float parameter, uint8_t RW, uint8_t type);

// 电机控制指令
void set_motor_para_bt(const EleMotor& motor, float p1, float p2, float p3, float p4, float p5, int model);

// 直接解包 CAN 帧数据（不再接收，只解析）
void unpack_frame(EleMotor& motor, const uint8_t* data, uint8_t dlc);

// 参数回帧详细打印开关。置 true 时连角度/速度/扭矩回帧也打印，
// 用于查看使能前的原始读数；常态运行务必保持 false（1kHz 会淹没终端）。
extern bool g_param_verbose;

#endif // ELE_MOTOR_H

