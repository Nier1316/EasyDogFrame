#ifndef ELE_MOTOR_H
#define ELE_MOTOR_H

#include <atomic>
#include <cstdint>
#include "ele_motor_def.h"
#include <thread>



class EleMotor {
private:
    // 状态标志
    std::atomic<uint32_t> state_version;  // 状态版本号
    std::atomic<bool> need_sync;          // 是否需要同步

    // 线程相关   
    std::thread sync_thread;
    std::atomic<bool> running;
    std::mutex state_mutex;

    // 同步函数
    void sync_thread_func();              // 后台线程主函数
    void sync_state();                    // 同步状态到电机

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

#endif // ELE_MOTOR_H

