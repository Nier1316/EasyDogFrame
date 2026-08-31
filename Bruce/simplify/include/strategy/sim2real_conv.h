/**
 * @file    sim2real_conv.h
 * @brief   sim2real 标定层：真机 GetStatus 角 ↔ URDF 约定（L5）
 * @details 从 rl_controller 拆出（阶段5）。职责：真机零位对齐 —— 策略在 URDF/MuJoCo
 *          约定下训练，真机 GetStatus 指令角存在每关节符号/偏移差异，由本层吸收。
 *          观测/动作工作区（策略侧）见 rl_controller.h；本层只做坐标系转换。
 *          名义站姿 DEFAULT_POSE 也归此处（它是 URDF 约定下的策略训练默认姿态）。
 */
#pragma once

namespace rl {

// 名义站姿（POLICY order：12 腿 + 4 轮）：hip=0, thigh=0.20, calf=-0.35, wheel=0。
// 与 dogurdf.py 的 NOMINAL_HIP/THIGH/CALF 一致，即策略训练默认姿态。
extern const float DEFAULT_POSE[16];

// 真机 GetStatus 角 ↔ URDF 角每关节映射（2026-08-20 真机 L 形测量）：
//   URDF = CONV_A[joint]*GetStatus + CONV_B[joint]（rad）
//   hip   A=+1 B=+0.0297   thigh A=-1 B=-0.9624   calf A=+1 B=-1.2832   wheel A=+1 B=0
// ⚠ 基于一次 L 形目测，数值待真机低增益验证后再微调。
extern const float CONV_A[16];   // ±1 符号
extern const float CONV_B[16];   // 偏移 (rad)

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

} // namespace rl
