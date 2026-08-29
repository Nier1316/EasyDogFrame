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
 *  - 零位对齐 / 真机↔URDF 转换见 strategy/sim2real_conv.h（CONV_A/B、DEFAULT_POSE、
 *    urdf_to_status 等已拆出）。观测/动作均工作在 URDF 约定。
 */
#pragma once

#include "strategy/sim2real_conv.h"   // DEFAULT_POSE / CONV_A/B / urdf 转换

namespace rl {

constexpr int NUM_JOINTS      = 16;
constexpr int NUM_LEG_JOINTS  = 12;
constexpr int NUM_WHEELS      = 4;
constexpr int OBS_DIM         = 64;

// ---- 控制参数（对齐 RL_Train/code/src/sim2sim.py 默认值，traj_v28 权威）----
constexpr float ACTION_SCALE        = 0.25f;
constexpr float WHEEL_VEL_SCALE     = 12.5f;
// LEG_KP/LEG_KD = 250/4：sim2sim.py 默认（traj_v28 训练 stiffness=250, damping=4）。
// ⚠ 2026-08-29 真机调整：LEG_KD 4 → 10——hip 逐渐外翻漂移（180601 日志 hip +0.06→+0.15），
//   阻尼 4 不足抑制外部负载漂移（电机内收力矩拉不回）。真机执行器有延迟，训练 damping=4 的
//   等效阻尼被延迟吃掉一部分，10 是补偿真机延迟的常规做法（历史 250/40 也验证更稳）。
//   若腿变"硬"影响策略动态，可退回 6~8。
constexpr float LEG_KP              = 250.0f;//（）
constexpr float LEG_KD              = 4.0f;//（）
// WHEEL_KD = 2.0：sim2sim.py 默认。⚠ 历史：真机曾 2→1 抑制解除挂钩振荡，现按
// sim2sim 默认回 2.0；若真机振荡复现需再评估。
constexpr float WHEEL_KD            = 1.0f;
// LEG_TORQUE_LIMIT = 250：sim2sim.py 默认（v28）。⚠ 历史曾用 150。
constexpr float LEG_TORQUE_LIMIT    = 250.0f;
constexpr float WHEEL_TORQUE_LIMIT  = 53.0f;
constexpr float CONTROL_DT          = 0.02f;   // 50 Hz
constexpr float GAIT_CYCLE          = 0.6f;

// ---- 轮子固件 SPEED 速度环（2026-08-29 迁移，见 FACT.md）----
// 固件阻抗模式忽略 vel_des（tau = kp×(pos_des−pos) + kd×(0−vel) + tau_ff），轮子速度
// 控制必须走 SPEED 模式（SendSpeed 下发 vel/kvp/ki，固件内部 1kHz 速度环）。
// KVP/KVI 对齐 robot_calibration.h 的 WHEEL_KVP/WHEEL_KVI（Example23 实测 3.0/0.3：
//   速度环收敛无振荡；0.5 起始有 33% 超调，2~3 是性价比平衡点）。
// WHEEL_SOFT_KVP：起立/回位期间轮子 0 速弱增益（软启动，假速度偏移窗口内力矩小）。
// WHEEL_CMD_ALPHA：轮速目标一阶低通（@50Hz τ≈100ms），抑制推杆/松杆 cmd 骤变导致的
//   轮子振荡。作用于轮速目标（执行平滑），不影响策略观测的 cmd（保持训练分布）。
constexpr float WHEEL_KVP        = 3.0f;    // 固件速度环比例增益
// WHEEL_KVI = 0.05：固件速度环积分增益，取 Example49 悬空单轮稳定档（ki=0.05）。
// ⚠ 历史 0.3（Example23 键盘控轮值）在 RL 上积分过强：50Hz 变化目标 + 反馈延迟 →
//   速度环超调振荡疯转（031608 日志：target ±0.5 实际 +6.5 rad/s）。
constexpr float WHEEL_KVI        = 0.05f;   // 固件速度环积分增益
// WHEEL_SOFT_KVP：起立/回位期间轮子 0 速弱增益（软启动）。
// ⚠ 2026-08-30 0.3→0.1：使能瞬间假速度偏移（CAN1 个体 +44 rad/s）会被速度环追，
//   kvp=0.3 产生 ~13 Nm 驱动力致起立前轮子疯转；0.1 降到 ~4.4 Nm 可控。
constexpr float WHEEL_SOFT_KVP   = 0.1f;    // 起立/回位期间轮子 0 速弱增益（软启动）
constexpr float WHEEL_CMD_ALPHA  = 0.2f;    // 轮速目标低通系数（50Hz）
// WHEEL_CMD_DEADZONE：轮速目标死区 (rad/s)。
// ⚠ 2026-08-30 已弃用：死区 0.5 会削减转向差速 action（target 0.5~1.0），导致真机转向严重
//   不到位（双开对比 035837 实证）。站立锁轮已由 WHEEL_CMD_MOVE_THR 门控接管（无移动指令
//   强制 0 速），移动时策略轮 action 完整执行、不再削死区。常量保留仅作历史参考。
constexpr float WHEEL_CMD_DEADZONE  = 0.5f;  // ⚠ 已弃用（门控取代），保留历史值
// WHEEL_CMD_MOVE_THR：移动指令阈值 (m/s 或 rad/s)。|cmd vx/wz| 都低于此 = 静止意图，
//   轮速目标强制 0。背景：策略站立时常给后轮正向微调 action（0.09~0.14 → target 1.1~1.75
//   rad/s，033222 日志后轮持续前转溜车），死区 0.5 挡不住；真机固件速度环会精确执行成
//   物理转。只有明确要移动才放开轮控。cmd 含 CMD_BIAS_VX（-0.05）< 阈值，站立正确锁轮。
constexpr float WHEEL_CMD_MOVE_THR  = 0.1f;  // 移动/静止判定阈值

// ---- 常量数组（定义见 rl_controller.cpp）----
// （DEFAULT_POSE / CONV_A / CONV_B / urdf 转换已移至 strategy/sim2real_conv）

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

// 轮子摩擦前馈（Nm）：[wheel_idx][0]=正向静摩擦, [1]=负向静摩擦。
// wheel_idx 0..3 = FL,FR,RL,RR（POLICY 轮顺序 == CAN 顺序）。
// 由 Example35_WheelFFCalibrate 实测（2026-08-21）：CAN2 阻力明显较大。
extern const float WHEEL_FF[4][2];

// 摩擦前馈总开关。⚠ 2026-08-21 定位轮电机乱转：策略站立时 wheel action 接近 0 但带噪声，
// 符号在 0 附近抖动会让前馈在 ±静摩擦间跳变 → 轮子被反复推正推负。先关掉验证。
constexpr bool WHEEL_FF_ENABLE = false;

// ---- 轮子速度软限位（安全兜底，2026-08-21）----
// 背景：RL 轮子曾被带到 48 rad/s 饱和（≈10.8 m/s，危险）。软限位在轮速超阈值时
// 速度软限位：轮速超阈值 → 把扭矩夹在 ±WHEEL_SOFT_LIMIT_TORQUE 内（限幅式，不是硬制动）。
// ⚠ 阈值 10 rad/s = 1.13 m/s 线速（v = vx/0.113）。满 action 目标轮速 12.5 rad/s 会超限，
//   即满摇杆/满 action 行驶时会触发保护（限加速方向，仍保留自然制动）。若需高速行驶再放宽。
constexpr bool   WHEEL_SOFT_LIMIT_ENABLE = true;   // 软限位开关
constexpr float  WHEEL_VEL_SOFT_LIMIT    = 5.0f;  // 异常速度阈值 (rad/s)
constexpr float  WHEEL_SOFT_LIMIT_TORQUE = 10.0f;   // 超限时扭矩限幅 (Nm)

/**
 * @brief 轮关节前馈扭矩：kd * (wheel_vel_scale * action - vel) [+ 摩擦前馈]，clip 到限幅
 * @param action      策略输出的轮动作（POLICY 轮索引 12..15 对应 idx=0..3）
 * @param vel         轮速反馈（URDF 约定）
 * @param wheel_idx   0..3 = FL,FR,RL,RR（== CAN0..3）
 *
 * 摩擦前馈（WHEEL_FF_ENABLE 时）：按期望运动方向（action 目标速度符号）叠加对应静摩擦，
 * 消除低速死区。开启后需加阈值/滞后避免 action 噪声导致前馈符号抖动。
 */
inline float wheel_torque(float action, float vel, int wheel_idx) {
    float tau = WHEEL_KD * (WHEEL_VEL_SCALE * action - vel);
    if (WHEEL_FF_ENABLE) {
        float tau_ff = 0.0f;
        if      (WHEEL_VEL_SCALE * action > 0.0f) tau_ff =  WHEEL_FF[wheel_idx][0];
        else if (WHEEL_VEL_SCALE * action < 0.0f) tau_ff =  WHEEL_FF[wheel_idx][1];
        tau += tau_ff;
    }

    // 速度软限位：轮速超阈值 → 扭矩限幅到 ±WHEEL_SOFT_LIMIT_TORQUE 内。
    // 限幅式（非硬制动）：PD 自然制动力（与超速方向相反）照常通过，只夹住加速方向的扭矩，
    // 避免超速瞬间强制反向制动的冲击。正超速禁正扭矩、负超速禁负扭矩。
    if (WHEEL_SOFT_LIMIT_ENABLE) {
        if      (vel >  WHEEL_VEL_SOFT_LIMIT) tau = (tau >  WHEEL_SOFT_LIMIT_TORQUE) ?  WHEEL_SOFT_LIMIT_TORQUE : tau;
        else if (vel < -WHEEL_VEL_SOFT_LIMIT) tau = (tau < -WHEEL_SOFT_LIMIT_TORQUE) ? -WHEEL_SOFT_LIMIT_TORQUE : tau;
    }

    float lim = WHEEL_TORQUE_LIMIT;
    return tau < -lim ? -lim : (tau > lim ? lim : tau);
}

} // namespace rl
