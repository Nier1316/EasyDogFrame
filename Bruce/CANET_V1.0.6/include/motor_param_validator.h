/**
 * @file    motor_param_validator.h
 * @brief   电机参数验证模块 - 按照 motor_rw_api.h 中的宏定义进行范围检查
 * @details 提供参数范围验证和自动截断功能，确保所有控制命令的参数在允许范围内
 */
#ifndef MOTOR_PARAM_VALIDATOR_H_
#define MOTOR_PARAM_VALIDATOR_H_

#include <cstdint>
#include <cmath>

// 参数范围定义（来自 motor_rw_api.h）
#define P_MIN   -12.5f
#define P_MAX   12.5f
#define V_MIN   -14.0f
#define V_MAX   14.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  100.0f
#define KI_MIN  0.0f
#define KI_MAX  10000.0f
#define T_MIN   -200.0f
#define T_MAX   200.0f

/**
 * @class MotorParamValidator
 * @brief 电机参数验证器 - 提供参数范围检查和截断功能
 */
class MotorParamValidator {
public:
    /**
     * @brief 检查并截断浮点参数到指定范围
     * @param value 输入值
     * @param min 最小值
     * @param max 最大值
     * @return 截断后的值
     */
    static float Clamp(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    /**
     * @brief 验证位置参数
     * @param pos 位置值 (rad)
     * @return 验证后的位置值
     */
    static float ValidatePosition(float pos) {
        return Clamp(pos, P_MIN, P_MAX);
    }

    /**
     * @brief 验证速度参数
     * @param vel 速度值 (rad/s)
     * @return 验证后的速度值
     */
    static float ValidateVelocity(float vel) {
        return Clamp(vel, V_MIN, V_MAX);
    }

    /**
     * @brief 验证Kp参数
     * @param kp Kp值
     * @return 验证后的Kp值
     */
    static float ValidateKp(float kp) {
        return Clamp(kp, KP_MIN, KP_MAX);
    }

    /**
     * @brief 验证Kd参数
     * @param kd Kd值
     * @return 验证后的Kd值
     */
    static float ValidateKd(float kd) {
        return Clamp(kd, KD_MIN, KD_MAX);
    }

    /**
     * @brief 验证Ki参数
     * @param ki Ki值
     * @return 验证后的Ki值
     */
    static float ValidateKi(float ki) {
        return Clamp(ki, KI_MIN, KI_MAX);
    }

    /**
     * @brief 验证扭矩参数
     * @param torque 扭矩值 (Nm)
     * @return 验证后的扭矩值
     */
    static float ValidateTorque(float torque) {
        return Clamp(torque, T_MIN, T_MAX);
    }

    /**
     * @brief 检查参数是否在范围内
     * @param value 参数值
     * @param min 最小值
     * @param max 最大值
     * @return true 如果在范围内，false 否则
     */
    static bool IsInRange(float value, float min, float max) {
        return value >= min && value <= max;
    }

    /**
     * @brief 检查位置是否在范围内
     */
    static bool IsPositionValid(float pos) {
        return IsInRange(pos, P_MIN, P_MAX);
    }

    /**
     * @brief 检查速度是否在范围内
     */
    static bool IsVelocityValid(float vel) {
        return IsInRange(vel, V_MIN, V_MAX);
    }

    /**
     * @brief 检查Kp是否在范围内
     */
    static bool IsKpValid(float kp) {
        return IsInRange(kp, KP_MIN, KP_MAX);
    }

    /**
     * @brief 检查Kd是否在范围内
     */
    static bool IsKdValid(float kd) {
        return IsInRange(kd, KD_MIN, KD_MAX);
    }

    /**
     * @brief 检查Ki是否在范围内
     */
    static bool IsKiValid(float ki) {
        return IsInRange(ki, KI_MIN, KI_MAX);
    }

    /**
     * @brief 检查扭矩是否在范围内
     */
    static bool IsTorqueValid(float torque) {
        return IsInRange(torque, T_MIN, T_MAX);
    }
};

#endif // MOTOR_PARAM_VALIDATOR_H_
