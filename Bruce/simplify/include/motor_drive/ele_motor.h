#ifndef ELE_MOTOR_H
#define ELE_MOTOR_H

#include <mutex>
#include <cstdint>
#include "ele_motor_def.h"



class EleMotor {
private:
    std::mutex state_mutex;  // 保护状态字段的并发读写

public:
    EleMotor() = default;
    ~EleMotor() = default;

    // 配置信息
    uint8_t device_idx;       // CAN设备索引
    uint8_t motor_id;         // 电机ID

    // 当前状态
    float current_speed;      // 当前速度 (rpm)
    float current_torque;     // 当前扭矩 (Nm)
    float current_position;   // 当前位置 (degree)
    float current_temp;       // 当前温度 (°C)

    // 目标状态
    float target_speed;       // 目标速度 (rpm)
    float target_torque;      // 目标扭矩 (Nm)
    float target_position;    // 目标位置 (degree)

    // 控制模式和增益参数
    int control_mode;         // 当前控制模式 (IMPEDANCE/SPEED/POSITION)
    float kp;                 // 刚度/位置环Kp
    float kd;                 // 阻尼/位置环Kd
    float ki;                 // 速度环Ki
    float kvp;                // 位置控制速度环Kp

    // 状态标志
    uint16_t error_code;      // 错误码
    bool enabled;             // 使能状态

    void init();
    void enable();
    void disable();
    bool has_error() const;
    void clear_error();

};

float uint_to_float(int x_int, float x_min, float x_max, int bits);
unsigned int float_to_uint(float x, float x_min, float x_max, int bits);

// 电机参数读写
void float2bag(const EleMotor& motor, float parameter, uint8_t RW, uint8_t type);

// 电机控制指令
void set_motor_para_bt(const EleMotor& motor, float p1, float p2, float p3, float p4, float p5, int model);

// CAN数据解包
bool unpack_cmd(EleMotor& motor, int timeout_ms = 100);

// 直接解包 CAN 帧数据（不再接收，只解析）
void unpack_frame(EleMotor& motor, const uint8_t* data, uint8_t dlc);

#endif // ELE_MOTOR_H

