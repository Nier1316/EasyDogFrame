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
 *  - 零位对齐暂未做：DEFAULT_POSE 用仿真占位值，后续足端对齐后替换。
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
constexpr float LEG_KP              = 150.0f;
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
