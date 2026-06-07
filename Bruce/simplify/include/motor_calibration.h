#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include <cstdint>

/**
 * @brief 电机标定参数结构
 */
struct MotorCalibrationParam {
    float pos_scale;       // 位置缩放系数（+1 或 -1）
    float vel_scale;       // 速度缩放系数（+1 或 -1）
    float pos_offset;      // 位置偏移值
};

/**
 * @brief 电机标定矩阵
 *
 * 矩阵布局：[4 CAN 端口][3 个电机] = 12 个电机
 *
 * 使用方式：
 *   motor_id = 1, 2, 3 (转换为数组索引: 0, 1, 2)
 *   can_port = 0, 1, 2, 3 (CAN 端口索引)
 *
 * 访问：calibration_matrix[can_port][motor_id - 1]
 */
static const MotorCalibrationParam MOTOR_CALIBRATION[4][3] = {
    // CAN0 端口 (左前腿)
    {
        {1.0f, 1.0f, 0.0f},      // Motor 1 (Hip): 正常
        {1.0f, 1.0f, 0.0f},      // Motor 2 (Thigh): 正常
        {1.0f, 1.0f, 0.0f}       // Motor 3 (Calf): 正常
    },

    // CAN1 端口 (右前腿)
    {
        {1.0f, 1.0f, 0.0f},      // Motor 1 (Hip): 正常
        {1.0f, 1.0f, 0.0f},      // Motor 2 (Thigh): 正常
        {1.0f, 1.0f, 0.0f}       // Motor 3 (Calf): 正常
    },

    // CAN2 端口 (左后腿)
    {
        {1.0f, 1.0f, 0.0f},      // Motor 1 (Hip): 正常
        {1.0f, 1.0f, 0.0f},      // Motor 2 (Thigh): 正常
        {1.0f, 1.0f, 0.0f}       // Motor 3 (Calf): 正常
    },

    // CAN3 端口 (右后腿)
    {
        {1.0f, 1.0f, 0.0f},      // Motor 1 (Hip): 正常
        {1.0f, 1.0f, 0.0f},      // Motor 2 (Thigh): 正常
        {1.0f, 1.0f, 0.0f}       // Motor 3 (Calf): 正常
    }
};

/**
 * @brief 应用标定参数到电机数据
 */
inline void ApplyMotorCalibration(uint8_t can_port, uint8_t motor_id,
                                   float& position, float& velocity, float& torque) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    const MotorCalibrationParam& calib = MOTOR_CALIBRATION[can_port][motor_id - 1];

    // 应用缩放和偏移
    position = position * calib.pos_scale + calib.pos_offset;
    velocity = velocity * calib.vel_scale;

    // 注：扭矩一般不需要反向，保持原值
}

#endif // MOTOR_CALIBRATION_H
