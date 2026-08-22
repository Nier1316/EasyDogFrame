/**
 * @file    rl_controller.h
 * @brief   dogurdf 轮足策略的观测构建与控制律（真机部署）
 *
 * 与 dogurdf_sim2sim_deploy/src/sim2sim.py 的 _build_observation /
 * _compute_torques_policy 严格一致，是 C++ 侧的等价实现。
 *
 * 关键约定：
 *  - 本模块所有数组均工作在 POLICY 顺序（12 腿关节 + 4 轮），
 *    与 CAN 顺序（per-leg: hip/thigh/calf/wheel）通过 POLICY_TO_MJX 互转。
 *  - 收发的标定由 MotorManager 自动处理（GetStatus 已标定、SendImpedance 自动逆标定），
 *    本模块只面对「标定后统一坐标系」。
 *  - 零位对齐已做：真机 GetStatus 角 ↔ URDF 角的每关节差异（大腿符号相反 + 偏移）
 *    由 CONV_A/CONV_B 转换吸收（见下）。观测/动作均工作在 URDF 约定，
 *    DEFAULT_POSE 是策略训练默认姿态（URDF 约定），下发前经 urdf_to_status 转回真机指令角。
 */
#pragma once

namespace rl {

constexpr int NUM_JOINTS      = 16;
constexpr int NUM_LEG_JOINTS  = 12;
constexpr int NUM_WHEELS      = 4;
constexpr int OBS_DIM         = 64;

// ---- 控制参数（sim2sim.py 静态常量）----
constexpr float ACTION_SCALE        = 0.25f;
constexpr float WHEEL_VEL_SCALE     = 12.5f;
constexpr float LEG_KP              = 250.0f;   // 与 iteration_450 训练配置一致（stiffness=250）
constexpr float LEG_KD              = 4.0f;
constexpr float WHEEL_KD            = 2.0f;
constexpr float LEG_TORQUE_LIMIT    = 150.0f;
constexpr float WHEEL_TORQUE_LIMIT  = 53.0f;
constexpr float CONTROL_DT          = 0.02f;   // 50 Hz
constexpr float GAIT_CYCLE          = 0.6f;

// ---- 常量数组（定义见 rl_controller.cpp）----

// 名义站姿（POLICY order，占位：hip=0, thigh=0.20, calf=-0.35, wheel=0）
extern const float DEFAULT_POSE[NUM_JOINTS];
// 关节限位（POLICY order；轮子用大哨兵使 clip 空操作）
extern const float JOINT_LOWER[NUM_JOINTS];
extern const float JOINT_UPPER[NUM_JOINTS];
// 步态相位偏移（FL, FR, RL, RR）
extern const float GAIT_OFFSET[4];
// POLICY -> MJX/CAN 映射：can[mjx] 对应 policy[POLICY_TO_MJX[mjx]]
extern const int POLICY_TO_MJX[NUM_JOINTS];
// MJX/CAN -> POLICY 映射：policy[i] 对应 can[MJX_TO_POLICY[i]]
extern const int MJX_TO_POLICY[NUM_JOINTS];

// ---- 真机 GetStatus 角 ↔ URDF 角的每关节映射（2026-08-20 真机 L 形测量）----
// 背景：策略在 URDF/MuJoCo 约定下训练（正向=后摆等），但真机 GetStatus 返回的
//       指令角用了电机标定约定，两者存在每关节的符号/偏移差异。
//       实测：大腿符号相反（真机正向=前摆），髋/小腿符号相同。
// 关系：URDF = CONV_A[joint]*GetStatus + CONV_B[joint]（角度 rad）
//   hip   A=+1 B=+0.0297   （L 形：GetStatus -1.7° ↔ URDF 0°）
//   thigh A=-1 B=-1.0524   （L 形：GetStatus -91.8° ↔ URDF +31.5°）
//   calf  A=+1 B=-1.3832   （L 形：GetStatus +89.0° ↔ URDF +9.75°）
//   wheel A=+1 B=0         （不参与位置 obs，速度仅用符号）
// ⚠ 基于一次 L 形目测，数值待真机低增益验证后再微调。
extern const float CONV_A[NUM_JOINTS];   // ±1 符号
extern const float CONV_B[NUM_JOINTS];   // 偏移 (rad)

/** GetStatus 角 → URDF 角（位置用，含偏移 B） */
inline float status_to_urdf(float angle_gs, int joint) {
    return CONV_A[joint] * angle_gs + CONV_B[joint];
}
/** GetStatus 速度 → URDF 速度（B 是常数，只保留符号） */
inline float status_vel_to_urdf(float vel_gs, int joint) {
    return CONV_A[joint] * vel_gs;
}
/** URDF 角 → GetStatus 角（动作下发用） */
inline float urdf_to_status(float angle_urdf, int joint) {
    return (angle_urdf - CONV_B[joint]) / CONV_A[joint];
}

/**
 * @brief 将世界系向量 v 通过四元数 q 旋转到机体坐标系
 * @param q    姿态四元数 [w, x, y, z]（body 相对 world）
 * @param v    世界系向量 [3]
 * @param out  机体系结果 [3]
 *
 * 等价于 sim2sim.py 的 brax rotate(v, quat_inv(q))，即 demo.py 的 world2self。
 */
void world2self(const float* q, const float* v, float* out);

/**
 * @brief 构建 64 维策略观测（POLICY order）
 * @param gyro        机体系角速度 [3]（rad/s，IMU 直接输出）
 * @param quat        IMU 四元数 [4]（w,x,y,z）
 * @param pos_policy  关节位置 [16]（POLICY order，标定后）
 * @param vel_policy  关节速度 [16]（POLICY order，标定后）
 * @param last_action 上一控制步动作 [16]（POLICY order）
 * @param cmd         指令 [3] = [vx, vy, wz]
 * @param step        episode 内控制步计数（gait_phase 时钟，reset 归 0）
 * @param obs         输出观测 [64]
 */
void build_observation(const float* gyro, const float* quat,
                       const float* pos_policy, const float* vel_policy,
                       const float* last_action, const float* cmd,
                       int step, float* obs);

/**
 * @brief 腿关节目标位置（POLICY order）：clip(default + action*scale, 限位)
 */
inline float leg_pos_target(float action, int policy_idx) {
    float q = DEFAULT_POSE[policy_idx] + action * ACTION_SCALE;
    float lo = JOINT_LOWER[policy_idx];
    float hi = JOINT_UPPER[policy_idx];
    return q < lo ? lo : (q > hi ? hi : q);
}

/**
 * @brief 轮关节前馈扭矩：kd * (wheel_vel_scale * action - vel)，clip 到限幅
 */
inline float wheel_torque(float action, float vel) {
    float tau = WHEEL_KD * (WHEEL_VEL_SCALE * action - vel);
    float lim = WHEEL_TORQUE_LIMIT;
    return tau < -lim ? -lim : (tau > lim ? lim : tau);
}

} // namespace rl
