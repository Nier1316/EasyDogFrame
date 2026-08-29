#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include <cstdint>

// =====================================================================
//  CAN 拓扑常量
// =====================================================================
constexpr uint8_t CAN_PORTS = 4;      // CAN 总线数量（4 条腿）
constexpr uint8_t MOTORS_PER_CAN = 4; // 每条 CAN 挂载电机数（3 关节 + 1 轮电机）

/**
 * @brief 电机标定参数结构
 */
struct MotorCalibrationParam
{
    float pos_scale;  // 位置缩放系数（+1 或 -1）
    float vel_scale;  // 速度缩放系数（+1 或 -1）
    float pos_offset; // 位置偏移值
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
constexpr MotorCalibrationParam MOTOR_CALIBRATION[CAN_PORTS][MOTORS_PER_CAN] = {
    // CAN0 端口 (左前腿)
    {
        {-1.0f, 1.0f, 0.611f}, // Motor 1 (Hip)
        {1.0f, 1.0f, 0.441f},  // Motor 2 (Thigh)
        {-1.0f, 1.0f, 0.211f}, // Motor 3 (Calf)
        {-1.0f, -1.0f, 0.0f},  // Motor 4 (Wheel) — pos_scale=-1 实测(2026-08-21 扭矩测向:固件负扭矩=前滚,故策略正扭矩=前滚)；vel_scale=-1 保持
    },
    // CAN1 端口 (右前腿)
    {
        // Motor 1 (Hip) pos_offset 2026-08-22 试改：0.611 → 0.78。
        //   依据：FR hip 下发位置命令偏(RL +0.18 / Example21 +0.108 rad)，反推 offset 应≈0.78~0.80。
        //   若 FR hip 反偏/过调，调回 0.70 或 0.75；若仍不足再往上。
        {1.0f, 1.0f, 0.611f},  // Motor 1 (Hip)
        {-1.0f, 1.0f, 0.441f}, // Motor 2 (Thigh)
        {1.0f, 1.0f, 0.211f},  // Motor 3 (Calf)
        {1.0f, 1.0f, 0.0f},    // Motor 4 (Wheel) — 与其他右后腿(CAN3)方向相反
    },
    // CAN2 端口 (左后腿)
    {
        {1.0f, 1.0f, 0.611f},  // Motor 1 (Hip)
        {1.0f, 1.0f, 0.441f},  // Motor 2 (Thigh)
        {-1.0f, 1.0f, 0.211f}, // Motor 3 (Calf)
        {-1.0f, -1.0f, 0.0f},  // Motor 4 (Wheel) — pos_scale=-1 实测(2026-08-21 扭矩测向:固件负扭矩=前滚,故策略正扭矩=前滚)；vel_scale=-1 保持
    },
    // CAN3 端口 (右后腿)
    {
        {-1.0f, 1.0f, 0.611f}, // Motor 1 (Hip)
        {-1.0f, 1.0f, 0.441f}, // Motor 2 (Thigh)
        {1.0f, 1.0f, 0.211f},
        // Motor 3 (Calf)
        {1.0f, 1.0f, 0.0f}, // Motor 4 (Wheel) — 待实测
    },
};

// =====================================================================
//  关节阻抗控制参数（刚度 / 阻尼 / 重力前馈）
// =====================================================================
/**
 * @brief 单个关节的阻抗控制参数
 *
 * 阻抗模式下电机固件按下式算扭矩：
 *   τ = kp·(θ_target − θ_actual) + kd·(ω_target − ω_actual) + tau_ff
 *
 * tau_ff 为 0 时，全部支撑力矩只能由位置误差换取——kp=150 时要出 30 N·m
 * 就必须先塌 0.2 rad(11.5°)，而塌下去狗就起不来。所以静态保持力矩要
 * 通过 tau_ff 直接给，位置误差只负责修偏差。
 */
struct JointImpedanceParam
{
    float kp;     // 刚度 (N·m/rad)
    float kd;     // 阻尼 (N·m·s/rad)
    float tau_ff; // 重力前馈力矩 (N·m)，上层统一坐标系
};

/**
 * @brief 关节阻抗参数表 [CAN_PORTS][3]，只含 3 个关节，不含轮电机
 *
 * 访问：JOINT_IMPEDANCE[can_port][motor_id - 1]，motor_id ∈ {1,2,3}
 *
 * ---- tau_ff 取值方法 ----
 * 1) 用当前参数让狗尽力站住（哪怕塌着），然后跑：
 *        conda activate dog && python tool/plot_motor_torque.py
 * 2) 统计表 mean 列即该关节的稳态保持扭矩，直接填入（同号，无需换算方向位：
 *    ApplyMotorCalibrationInverse 会按 pos_scale 自动翻转）。
 * 3) 首次只填实测值的 50%，确认位置误差变小（方向对）后再补足；
 *    若误差反而变大，立即断电并把符号翻过来。
 *
 * ---- 实测参考值（2026-08-05，Example19 站稳，recv_20260805_172828）----
 *   Hip ≈ +7~+12   Thigh ≈ −5~−6.6   Calf ≈ −8.7~+4.0（后腿 Calf 变号）
 * 注意量级：站稳时保持扭矩只有个位数到十几 N·m，占限幅 5~9%。
 * 填 40~60 这种量级会让关节直接顶过去——tau_ff 是前馈，不受位置误差约束。
 *
 * kp/kd 与 tau_ff（2026-08-29 更新）：RL 循环腿软改善——thigh/calf 填重力前馈
 * （thigh=8, calf=5 Nm，个位数安全量级），直接给支撑力矩，不再靠位置误差塌换，
 * 同时保留训练 kp/kd（LEG_KP/KD 由 rl_controller.h 下发，表内 kp/kd 仅作参考）。
 * hip tau_ff=10 是站立理想姿态的关键（用户 2026-08-29 调）。
 * 量级规则：站稳保持扭矩个位数~十几 Nm；填 40~60 会顶过关节。
 */
static const JointImpedanceParam JOINT_IMPEDANCE[CAN_PORTS][3] = {
    //                kp      kd   tau_ff
    // CAN0 端口 (左前腿)
    {
        {300.0f, 10.0f, -10.0f}, // Motor 1 (Hip)  tau_ff=-14
        {250.0f, 10.0f, -5.0f},  // Motor 2 (Thigh)  tau_ff=-5
        {250.0f, 10.0f, 12.0f},  // Motor 3 (Calf)   tau_ff=16
    },
    // CAN1 端口 (右前腿)
    {
        {300.0f, 10.0f, -10.0f}, // Motor 1 (Hip)  tau_ff=-14
        {250.0f, 10.0f, -5.0f},  // Motor 2 (Thigh)  tau_ff=-5
        {250.0f, 10.0f, 12.0f},  // Motor 3 (Calf)   tau_ff=16
    },
    // CAN2 端口 (左后腿)
    {
        {300.0f, 10.0f, -10.0f}, // Motor 1 (Hip)  tau_ff=-14
        {250.0f, 10.0f, -5.0f},  // Motor 2 (Thigh)  tau_ff=-5
        {250.0f, 10.0f, 20.0f},  // Motor 3 (Calf)   tau_ff=26
    },
    // CAN3 端口 (右后腿)
    {
        {300.0f, 10.0f, -10.0f}, // Motor 1 (Hip)  tau_ff=-14
        {250.0f, 10.0f, -5.0f},  // Motor 2 (Thigh)  tau_ff=-5
        {250.0f, 10.0f, 20.0f},  // Motor 3 (Calf)   tau_ff=26
    },
};

/**
 * @brief 取关节阻抗参数；越界回退到一组安全默认值
 * @param motor_id 只接受 1~3（关节）；轮电机走速度环，不用这张表
 */
inline const JointImpedanceParam &GetJointImpedance(uint8_t can_port,
                                                    uint8_t motor_id)
{
    static const JointImpedanceParam fallback = {150.0f, 20.0f, 0.0f};
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > 3)
    {
        return fallback;
    }
    return JOINT_IMPEDANCE[can_port][motor_id - 1];
}

/**
 * @brief 应用标定参数到电机反馈数据（接收方向）
 */
inline void ApplyMotorCalibration(uint8_t can_port, uint8_t motor_id,
                                  float &position, float &velocity, float &torque)
{
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN)
    {
        return;
    }

    const MotorCalibrationParam &calib = MOTOR_CALIBRATION[can_port][motor_id - 1];

    position = position * calib.pos_scale + calib.pos_offset;
    velocity = velocity * calib.vel_scale;
    // 反馈扭矩与位置同坐标系，一并翻转，保证收发对称
    torque = torque * calib.pos_scale;
}

/**
 * @brief 对控制指令应用逆标定（发送方向）
 * 上层以统一坐标系给出目标值，逆变换回电机原始坐标系后发送。
 */
inline void ApplyMotorCalibrationInverse(uint8_t can_port, uint8_t motor_id,
                                         float &position, float &velocity,
                                         float *torque = nullptr)
{
    if (can_port >= CAN_PORTS || motor_id < 1 || motor_id > MOTORS_PER_CAN)
    {
        return;
    }

    const MotorCalibrationParam &calib = MOTOR_CALIBRATION[can_port][motor_id - 1];

    // pos_scale 为 ±1，逆变换等于再乘一次；offset 需先减去再除以 scale
    position = (position - calib.pos_offset) * calib.pos_scale;
    velocity = velocity * calib.vel_scale;
    // 前馈扭矩与位置同坐标系：pos_scale 翻转方向时，扭矩必须一起翻，
    // 否则上层基于标定后反馈算出的扭矩会朝反方向使劲，形成正反馈发散。
    if (torque)
    {
        *torque = *torque * calib.pos_scale;
    }
}

#endif // MOTOR_CALIBRATION_H
