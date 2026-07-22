#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include <cstdint>

// =====================================================================
//  CAN 拓扑常量
// =====================================================================
constexpr uint8_t CAN_PORTS      = 4;  // CAN 总线数量（4 条腿）
constexpr uint8_t MOTORS_PER_CAN = 4;  // 每条 CAN 挂载电机数（3 关节 + 1 轮电机）

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
 * 矩阵布局：[CAN_PORTS][MOTORS_PER_CAN] = 16 个电机
 *
 * 使用方式：
 *   motor_id = 1, 2, 3, 4 (转换为数组索引: 0, 1, 2, 3)
 *   can_port = 0, 1, 2, 3 (CAN 端口索引)
 *
 * 访问：calibration_matrix[can_port][motor_id - 1]
 *
 * motor_id=1 (Hip), 2 (Thigh), 3 (Calf), 4 (Wheel)
 */
static const MotorCalibrationParam MOTOR_CALIBRATION[CAN_PORTS][MOTORS_PER_CAN] = {
    // CAN0 端口 (左前腿)
    {
        {-1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        { 1.0f,  1.0f, 0.441f},      // Motor 2 (Thigh)
        {-1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
        { -1.0f,  1.0f, 0.0f  },      // Motor 4 (Wheel) — 待实测
    },
    // CAN1 端口 (右前腿)
    {
        { 1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        {-1.0f,  1.0f, 0.441f},      // Motor 2 (Thigh)
        { 1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
        {-1.0f,  1.0f, 0.0f  },      // Motor 4 (Wheel) — 与其他右后腿(CAN3)方向相反
    },
    // CAN2 端口 (左后腿)
    {
        { 1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        { 1.0f,  1.0f, 0.441f},      // Motor 2 (Thigh)
        {-1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
        { -1.0f,  1.0f, 0.0f  },      // Motor 4 (Wheel) — 待实测
    },
    // CAN3 端口 (右后腿)
    {
        {-1.0f,  1.0f, 0.611f},      // Motor 1 (Hip)
        {-1.0f,  1.0f, 0.441f},      // Motor 2 (Thigh)
        { 1.0f,  1.0f, 0.211f},      // Motor 3 (Calf)
        { 1.0f,  1.0f, 0.0f  },      // Motor 4 (Wheel) — 待实测
    },
};

/**
 * @brief 应用标定参数到电机反馈数据（接收方向）
 */
inline void ApplyMotorCalibration(uint8_t can_port, uint8_t motor_id,
                                   float& position, float& velocity, float& torque) {
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
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
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN) {
        return;
    }

    const MotorCalibrationParam& calib = MOTOR_CALIBRATION[can_port][motor_id - 1];

    // pos_scale 为 ±1，逆变换等于再乘一次；offset 需先减去再除以 scale
    position = (position - calib.pos_offset) * calib.pos_scale;
    velocity = velocity * calib.vel_scale;
}

#endif // MOTOR_CALIBRATION_H
