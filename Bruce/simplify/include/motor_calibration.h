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
        {-1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        { 1.0f,  1.0f, 0.441f},      // Motor 2 (Thigh)
        {-1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
    },
    // CAN1 端口 (右前腿)
    {
        { 1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        {-1.0f,  1.0f, 0.841f},      // Motor 2 (Thigh)
        { 1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
    },
    // CAN2 端口 (左后腿)
    {
        { 1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        { 1.0f,  1.0f, 0.441f},      // Motor 2 (Thigh)
        {-1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
    },
    // CAN3 端口 (右后腿)
    {
        {-1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        {-1.0f,  1.0f, 0.441f},      // Motor 2 (Thigh)
        { 1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
    },
};

/**
 * @brief 应用标定参数到电机反馈数据（接收方向）
 */
inline void ApplyMotorCalibration(uint8_t can_port, uint8_t motor_id,
                                   float& position, float& velocity, float& torque) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    const MotorCalibrationParam& calib = MOTOR_CALIBRATION[can_port][motor_id - 1];

    position = position * calib.pos_scale + calib.pos_offset;
    velocity = velocity * calib.vel_scale;
}

/**
 * @brief 对控制指令应用逆标定（发送方向）
 * 上层以统一坐标系给出目标值，逆变换回电机原始坐标系后发送。
 */
inline void ApplyMotorCalibrationInverse(uint8_t can_port, uint8_t motor_id,
                                          float& position, float& velocity) {
    if (can_port >= 4 || motor_id < 1 || motor_id > 3) {
        return;
    }

    const MotorCalibrationParam& calib = MOTOR_CALIBRATION[can_port][motor_id - 1];

    // pos_scale 为 ±1，逆变换等于再乘一次；offset 需先减去再除以 scale
    position = (position - calib.pos_offset) * calib.pos_scale;
    velocity = velocity * calib.vel_scale;
}

#endif // MOTOR_CALIBRATION_H
