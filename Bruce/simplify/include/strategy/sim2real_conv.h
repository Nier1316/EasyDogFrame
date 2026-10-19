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

// 名义站姿（POLICY order：12 腿 + 4 轮）：hip=0, thigh=-0.03, calf=-0.116, wheel=0。
// 与 RL_Train/code/src/robots/dogurdf.py 的 NOMINAL_HIP/THIGH/CALF 一致
// （2026-09-15 站高 0.52 m 标定），即策略训练默认姿态。
// ✅ 2026-10-17：本组数值**已由训练侧真值核实**，取代此前的 thigh=0.20 / calf=-0.35。
//   历史疑问（已闭环）：旧 calf=-0.35 与 dogurdf.py 的 NOMINAL_CALF 差 0.18 rad=10.31°，
//   FK 下旧值会让轮心陷地约 50 mm ⇒ 旧值确实是错的，现以 RL_Train 的 0.52 m 站高标定为准。
extern const float DEFAULT_POSE[16];

/**
 * @brief 真机**起立/摆放目标**姿态（URDF，16：POLICY 序 12 腿 + 4 轮）
 *
 * 与 `DEFAULT_POSE` 的区别（重要）：
 *   · `DEFAULT_POSE` = **训练侧**名义姿态 ⇒ 参与策略观测 `joint_pos_rel = q − DEFAULT_POSE`，
 *     **不要为了调站高去改它**（会让策略观测带上恒定偏置）。
 *   · `stand_pose()` = **真机**起立/摆放目标，只为机械上站得舒服（轮子刚好触地）。
 *     默认值 = `DEFAULT_POSE`；可用环境变量实时覆盖（4 条腿相同）：
 *         STAND_THIGH_DEG=-8   STAND_CALF_DEG=-3   ./run.sh 62
 *     实测（MJCF FK，轮半径 113 mm）：每档 thigh −1.7°/calf +1.7° 抬高约 8~10 mm。
 * @return 指向静态数组的指针（首次调用时按环境变量初始化）
 */
const float* stand_pose();

// 真机 GetStatus 角 ↔ URDF 角每关节映射（2026-08-20 真机 L 形测量）：
//   URDF = CONV_A[joint]*GetStatus + CONV_B[joint]（rad）
//   hip   默认 A=+1 B=+0.0297（**例外：FR hip 为 A=-1 B=-0.0297**，2026-10-13 现场确认
//         观测方向反了，修法=整个映射取反以保零位，见 sim2real_conv.cpp 注释）
//   thigh A=-1 B=-0.9624   calf A=+1 B=-1.2832   wheel A=+1 B=0
// ⚠ 基于一次 L 形目测（08-20）；**hip 的符号还叠加了电机层 pos_scale 的对角模式**
//   （FL −1 / FR +1 / RL +1 / RR −1，自 57cd63b 起未复核），FR 已于 10-13 现场纠正，
//   RL 待现场确认。判据：腿远端向机器人左侧推 ⇒ URDF 读数应变大。
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
