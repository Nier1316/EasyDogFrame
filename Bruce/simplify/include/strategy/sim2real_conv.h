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

// 真机 GetStatus 角 ↔ URDF 角每关节映射（**以 sim2real_conv.cpp 的数组为唯一准据**）：
//   URDF = CONV_A[joint]*GetStatus + CONV_B[joint]（rad）
//   hip   : 左侧 FL/RL  A=+1 B=+0.0297 ；右侧 FR/RR  A=-1 B=-0.0297   （左右镜像）
//   thigh : 四腿        A=-1 B=-0.9624
//   calf  : 四腿        A=+1 B=-1.2832
//   wheel : 四腿        A=+1 B=0
// 叠加电机层 MOTOR_CALIBRATION.pos_scale（FL −1 / FR +1 / RL +1 / RR −1，对角模式，
// 自 57cd63b 引入）后的合成关系 q_urdf = a*raw + b，逐腿数值见
// docs/JOINT_FRAME_CHAIN.md：
//   hip   四腿 b = +0.6407 / −0.6407 / +0.6407 / −0.6407（左右镜像）
//   thigh 四腿 b = −1.4034 ； calf 四腿 b = −1.0722（四腿同值）
//
// ✅ 2026-10-26 状态一：**符号已由用户现场确认为正确**，不再回退。
//    硬理由：PD 符号错 ⇒ 正反馈 ⇒ 立刻失控；"机器人能走"本身就证明 12 个腿关节符号
//    全部正确 ⇒ 不要把"腿间符号不整齐"当成可"统一"的遗留问题。
// ✅ 2026-10-26 状态二：**零位已用 SolidWorks 几何复核**。判据：把腿卡在限位块
//    （= 编码器 0）处，该姿态的 SW 关节角必须等于上面的 b。结果：hip 左右镜像正常、
//    calf 正常、thigh 与"装配零位后倾 52°（竖直 ⇔ −52.41°）"自洽
//    ⇒ **CONV_A / CONV_B / pos_scale / pos_offset 全部无需修改**（零位这条线已关闭）。
// ⚠ 历史：本条曾因"手推腿判方向"反复翻转（10-13 翻 FR、10-17 补翻 RR）并留下自相矛盾
//    的注释。今后判方向**只做运动验证**（给该关节已知 +Δ 指令，看腿实际往哪边走），
//    不要靠手推 —— 见 sim2real_conv.cpp 的教训段。
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
