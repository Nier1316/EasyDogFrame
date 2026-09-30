/**
 * @file    rl_controller.h
 * @brief   dogurdf 轮足策略的观测构建与控制律（真机部署）
 *
 * 与 dogurdf_sim2sim_deploy/src/sim2sim.py 的 _build_observation /
 * _compute_torques_policy 严格一致，是 C++ 侧的等价实现。
 *
 * 关键约定：
 *  - 本模块所有数组均工作在 POLICY 顺序（12 腿关节 + 4 轮），
 *    与 CAN 顺序（per-leg: hip/thigh/calf/wheel）通过 CAN_TO_POLICY 互转。
 *  - 收发的标定由 MotorManager 自动处理（GetStatus 已标定、SendImpedance 自动逆标定），
 *    本模块只面对「标定后统一坐标系」。
 *  - 零位对齐 / 真机↔URDF 转换见 strategy/sim2real_conv.h（CONV_A/B、DEFAULT_POSE、
 *    urdf_to_status 等已拆出）。观测/动作均工作在 URDF 约定。
 */
#pragma once

#include <cmath>                       // std::tanh（摩擦前馈）
#include "strategy/sim2real_conv.h"   // DEFAULT_POSE / CONV_A/B / urdf 转换

namespace rl {

constexpr int NUM_JOINTS      = 16;
constexpr int NUM_LEG_JOINTS  = 12;
constexpr int NUM_WHEELS      = 4;
constexpr int OBS_DIM         = 64;

// ---- 控制参数（与 RL_Train/code 训练侧、dogurdf_sim2sim_deploy/src/sim2sim.py 保持一致）----
// 当前部署权重 = weights/iteration_9754.pkl（经 tool/export_policy.py 导出到 policy_weights.h）；
// sim2sim 也指向同一份（dogurdf_sim2sim_deploy/run_sim2sim.sh），保证对比是同策略。
constexpr float ACTION_SCALE        = 0.25f;
constexpr float WHEEL_VEL_SCALE     = 12.5f;
// LEG_KP/LEG_KD = 250/4：对齐 sim2sim.py 默认（训练 stiffness=250, damping=4）。
// ⚠ 2026-08-29 曾评估 LEG_KD 提至 10（hip 外翻漂移 180601 日志 +0.06→+0.15，阻尼 4 偏弱、
//   真机延迟吃掉部分阻尼），但未落地——当前仍 4.0，对齐训练。若真机 hip 仍漂移，
//   可现场试提 6~10（历史 250/40 也验证更稳）再定。
constexpr float LEG_KP              = 250.0f;
constexpr float LEG_KD              = 4.0f;
// WHEEL_KD = 1.0：轮速阻尼（RL 阻抗诊断路径用）。⚠ 历史 2.0（sim2sim 默认）→ 1.0 抑制
// 解除挂钩振荡；SPEED 迁移后轮子走固件速度环（kvp/ki），此量仅诊断用。
constexpr float WHEEL_KD            = 1.0f;
// ⚠ 扭矩限幅不在本文件设——真机由固件阻抗环 + ele_motor_def.h 的 TORQUE_CMD_LIMIT 决定：
//   MOTOR_LIMITS 协议量程 = TORQUE_CMD_LIMIT 命令上限 = Hip/Thigh±120、Calf±200、Wheel±52
//   （2026-08-30 用户改固件限幅，2026-09-04 Hip/Thigh 110→120；编码前 clamp 防越界，反馈按量程解析）。
//   sim2sim 侧默认 LEG_TORQUE_LIMIT=250 / WHEEL_TORQUE_LIMIT=53；加 --real_actuator 才对齐真机的
//   120/120/200/52。即真机 Calf 封顶 ±200 < sim 的 250 → calf 大扭矩动作在真机被削顶，
//   做 sim2real 对比时应带 --real_actuator 以消除该饱和域差异。
constexpr float CONTROL_DT          = 0.02f;   // 50 Hz
constexpr float GAIT_CYCLE          = 0.6f;

// ---- 轮子固件 SPEED 速度环（2026-08-29 迁移，见 FACT.md）----
// 固件阻抗模式忽略 vel_des（tau = kp×(pos_des−pos) + kd×(0−vel) + tau_ff），轮子速度
// 控制必须走 SPEED 模式（SendSpeed 下发 vel/kvp/ki，固件内部 1kHz 速度环）。
// KVP = 3.0 对齐 robot_calibration.h 的 WHEEL_KVP（Example23 实测 3.0：速度环收敛无振荡；
//   0.5 起始有 33% 超调，2~3 是性价比平衡点）。KVI 取 0.05（见下，非 0.3）。
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
// ---- 顺序置换（2026-09-30 更名：原 POLICY_TO_MJX / MJX_TO_POLICY 两个名字与实际语义**正好相反**）----
//   CAN 顺序 i = can_port*4 + (motor_id-1)：00=FL-hip 01=FL-thigh 02=FL-calf 03=FL-wheel, 04=FR-…, 15=RR-wheel
//   POLICY 顺序：0..11 = 四条腿的 hip/thigh/calf（FL,FR,RL,RR），12..15 = 四个轮。
// 用法（按"我要什么"选名字，别凭直觉猜）：
//   · 第 p 个 policy 关节对应哪个 CAN 索引 → `POLICY_TO_CAN[p]`
//   · 第 i 个 CAN 电机对应哪个 policy 关节 → `CAN_TO_POLICY[i]`
// 例：`pos_policy[p] = pos_can[rl::POLICY_TO_CAN[p]];` / `int p = rl::CAN_TO_POLICY[mjx];`
extern const int POLICY_TO_CAN[NUM_JOINTS];
extern const int CAN_TO_POLICY[NUM_JOINTS];
// ⚠ 兼容别名：下面两个是**旧名**，其字面含义与所指数组相反（历史上极易踩坑），
//   仅为不破坏既有调用点而保留；新代码一律用上面的 POLICY_TO_CAN / CAN_TO_POLICY。
extern const int (&POLICY_TO_MJX)[NUM_JOINTS];   // == CAN_TO_POLICY（不要按名字理解！）
extern const int (&MJX_TO_POLICY)[NUM_JOINTS];   // == POLICY_TO_CAN（不要按名字理解！）

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

// ---- 轮子摩擦前馈 / 轮子扭矩软限位：已删除（2026-09-29）----
// 历史实现 rl::wheel_torque()（kd·(WHEEL_VEL_SCALE·action − vel) + WHEEL_FF 摩擦前馈 + 速度软限位）
// 在 2026-08-29 轮控迁移到固件 SPEED 速度环之后**再无任何调用者**，本次连同
// WHEEL_FF[4][2] / WHEEL_FF_ENABLE / WHEEL_SOFT_LIMIT_ENABLE / WHEEL_VEL_SOFT_LIMIT /
// WHEEL_SOFT_LIMIT_TORQUE / WHEEL_TORQUE_LIMIT 一并删除（历史数值见 memory/FACT.md、UPDATE.md）。
// 轮速保护现由 MotorManager 的 WHEEL_ESTOP_*（15 rad/s 超速自动急停 + 固件速度环制动）承担。

// ---- 腿摩擦前馈（Coulomb + Viscous，参考 project_5_Matrix_deploy/friction_model.py）----
// 模型：τ_ff = fc·tanh(τ_pd / 2) + fv·dq
//   - 方向用 PD 扭矩方向 tanh(τ_pd/2)：始终帮 PD、绝不抵抗，避免速度方向在站立/静止时
//     符号抖动（当年轮子摩擦前馈乱转的教训）。τ_pd 为当前 PD 输出（kp·(q_t−q) − kd·q̇，URDF 约定）。
//   - fv·dq：粘性阻尼前馈。Example54/47 辨识 b（粘性）不可靠（速度反馈延迟），暂置 0。
//   - fc：每关节库仑摩擦，来自 Example54 吊装摩擦辨识（2026-09-04）。
// ⚠ 实测值并不小（1.64~6.17 Nm，见 rl_controller.cpp 的 LEG_FF_FC），已接近部分关节的
//   PD 输出量级；若真机出现"越动越快/振荡"，先关闭 LEG_FF_ENABLE 复核再折半。
constexpr bool   LEG_FF_ENABLE  = true;
constexpr float  LEG_FF_TANH_K  = 0.5f;   // tanh(τ_pd/K) 方向光滑参数
extern const float LEG_FF_FC[12];          // 每关节库仑摩擦 (Nm)，POLICY order（12 腿）
extern const float LEG_FF_FV[12];          // 每关节粘性阻尼 (Nm·s/rad)，POLICY order（12 腿）

/** 腿摩擦前馈扭矩（URDF 约定，叠加到 SendImpedance 的 tau_ff 上） */
inline float leg_friction_ff(float tau_pd, float dq, int policy_idx) {
    if (!LEG_FF_ENABLE) return 0.0f;
    return LEG_FF_FC[policy_idx] * std::tanh(tau_pd * LEG_FF_TANH_K)
         + LEG_FF_FV[policy_idx] * dq;
}

} // namespace rl
