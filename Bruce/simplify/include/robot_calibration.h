#ifndef ROBOT_CALIBRATION_H
#define ROBOT_CALIBRATION_H

/**
 * @file    robot_calibration.h
 * @brief   整机标定参数集中处 —— 运动解算 / 运动控制 / 参数辨识
 *
 * 与 motor_calibration.h 的分工：
 *   motor_calibration.h  单电机层面：方向位、零位偏移、阻抗增益（按 CAN 口 × 电机号）
 *   robot_calibration.h  整机层面：连杆尺寸、关节限位、步态与控制参数、动力学参数
 *
 * 本文件只放"需要按实物调整的数值"，不放算法。
 * 解算实现在 leg_kinematics.h，它包含本文件取参数。
 *
 * ---- 坐标系与角度约定（改任何符号前必读）----
 *   髋关节坐标系:  X+ 向后    Y+ 向外翻   Z+ 向上
 *   身体坐标系:    X+ 向前    Y+ 向左     Z+ 向上
 *   关节角用户约定: θ₁ 正=外翻  θ₂ 正=后摆  θ₃ 正=后弯
 *
 *   物理角 = ZERO_OFFSET + 指令角
 *   指令角 ∈ [LOWER_LIMIT, UPPER_LIMIT]
 */

#include <cmath>
#include <cstdint>
#include "motor_calibration.h"

// =====================================================================
//  §0  角度工具
// =====================================================================
constexpr float rad2deg(float rad) { return rad * (180.0f / M_PI); }
constexpr float deg2rad(float deg) { return deg * (M_PI / 180.0f); }

inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// =====================================================================
//  §1  腿与关节编号（对应 CAN 拓扑）
// =====================================================================
enum LegIndex : uint8_t {
    FL = 0,  // 左前 (CAN0)
    FR = 1,  // 右前 (CAN1)
    RL = 2,  // 左后 (CAN2)
    RR = 3,  // 右后 (CAN3)
};

enum JointIndex : uint8_t {
    HIP   = 0,  // θ₁ 髋外摆 (motor_id=1)
    THIGH = 1,  // θ₂ 大腿   (motor_id=2)
    CALF  = 2,  // θ₃ 小腿   (motor_id=3)
};

// =====================================================================
//  §2  连杆参数 (m) —— 按实物测量修改
//
//  IK 可达半径 = |L2−L3| ~ (L2+L3)。当前 L2==L3，故理论可完全收拢。
//  改这三个值会直接改变 FK/IK 结果，改完务必跑往返校验（见 §7）。
// =====================================================================
constexpr float LEG_L1 = 0.12f;   // 髋侧向偏移量（髋外摆轴 → 大腿轴的 Y 向距离）
constexpr float LEG_L2 = 0.35f;   // 大腿长度
constexpr float LEG_L3 = 0.35f;   // 小腿长度

// =====================================================================
//  §3  机身尺寸 (m) 与腿安装位置
// =====================================================================
constexpr float BODY_LENGTH = 0.30f;   // 机身长 (X 方向，前后)
constexpr float BODY_WIDTH  = 0.12f;   // 机身宽 (Y 方向，左右)
// 注意：BODY_SIZE_HEIGHT 目前不参与任何解算 —— LEG_MOUNT 的 z 全为 0，
// 即髋关节被视作与机身中心同高。需要考虑髋轴高度差时改 LEG_MOUNT 的 z 分量。
constexpr float BODY_SIZE_HEIGHT = 0.06f;   // 机身高 (Z 方向)

// 腿安装位置（身体坐标系）：x+ 向前, y+ 向左, z+ 向上
constexpr float LEG_MOUNT[4][3] = {
    { BODY_LENGTH / 2,  BODY_WIDTH / 2, 0 },   // FL 左前
    { BODY_LENGTH / 2, -BODY_WIDTH / 2, 0 },   // FR 右前
    {-BODY_LENGTH / 2,  BODY_WIDTH / 2, 0 },   // RL 左后
    {-BODY_LENGTH / 2, -BODY_WIDTH / 2, 0 },   // RR 右后
};

// =====================================================================
//  §4  关节零位偏移与限位 (deg)
//
//  物理角 = ZERO_OFFSET + 指令角，指令角必须落在 [LOWER, UPPER] 内。
//
//  ⚠ 当前站立姿态（§5 STAND_*）有两个角压在限位边界上：
//     θ₁ 指令 0° == UPPER_LIMIT_THETA1_DEG，髋朝正方向零余量
//     θ₃ 指令 60° == LOWER_LIMIT_THETA3_DEG，小腿朝负方向零余量
//  IK 反解只要略微越界就会被 clamp 削掉，表现为该关节"跟不上指令"。
//  调站立姿态时优先把这两个角挪进区间内部。
// =====================================================================

// ---- 零位偏移的唯一真值来源 ----
// 零位偏移由电机层 motor_calibration.h 的 MOTOR_CALIBRATION[].pos_offset 定义，
// 收发每一帧都过那条路径。此处必须与其保持一致，否则 FK/IK 预测的足端位置
// 与电机实际到达的位置会有偏差（曾出现 Hip 差 5°、Thigh 差 25°、Calf 差 12°）。
// 直接引用电机层 MOTOR_CALIBRATION 的零位偏移，保持单一真值来源，
// 改电机层时此处自动跟随，无需再手动同步。
constexpr float HIP_POS_OFFSET_RAD   = MOTOR_CALIBRATION[0][0].pos_offset;
constexpr float THIGH_POS_OFFSET_RAD = MOTOR_CALIBRATION[0][1].pos_offset;
constexpr float CALF_POS_OFFSET_RAD  = MOTOR_CALIBRATION[0][2].pos_offset;

// --- θ1 髋外摆 ---
constexpr float ZERO_OFFSET_THETA1_DEG = rad2deg(HIP_POS_OFFSET_RAD);    // 35.01°
constexpr float LOWER_LIMIT_THETA1_DEG = -60.0f;
constexpr float UPPER_LIMIT_THETA1_DEG =   0.0f;

// --- θ2 大腿 ---
// 下界原为 −45°，但 Example19 实测站稳的姿态需要 −60°（见 §5 STAND_THIGH_DEG）。
// 若仍用 −45°，Example21 主循环的 clamp（example.cpp:1822）会把站立指令削掉 15°，
// 站姿被悄悄改成一个站不住的姿态。放宽到 −70° 给 IK 留调节余量。
constexpr float ZERO_OFFSET_THETA2_DEG = rad2deg(THIGH_POS_OFFSET_RAD);  // 25.27°
constexpr float LOWER_LIMIT_THETA2_DEG = -70.0f;
constexpr float UPPER_LIMIT_THETA2_DEG =  90.0f;

// --- θ3 小腿 ---
constexpr float ZERO_OFFSET_THETA3_DEG = rad2deg(CALF_POS_OFFSET_RAD);   // 12.09°
constexpr float LOWER_LIMIT_THETA3_DEG =  60.0f;
constexpr float UPPER_LIMIT_THETA3_DEG = 180.0f;

// 弧度版本（解算内部使用，勿单独修改）
constexpr float THETA1_OFFSET = deg2rad(ZERO_OFFSET_THETA1_DEG);
constexpr float THETA2_OFFSET = deg2rad(ZERO_OFFSET_THETA2_DEG);
constexpr float THETA3_OFFSET = deg2rad(ZERO_OFFSET_THETA3_DEG);

// =====================================================================
//  §5  运动控制参数
// =====================================================================

// ---- 控制周期 ----
constexpr int   CONTROL_HZ = 100;                 // 主控制循环频率 (Hz)

// ---- 站立姿态指令角 (deg) ----
// 四条腿共用。取自 Example19_ReadAndStand 实测能站稳的一组值
// （2026-08-05 日志 recv_20260805_172828：Thigh 仅塌 1.6~1.8°，扭矩 5~6.6 N·m，
//   占限幅 5~6%，四腿高度对称）。
// 注意 Thigh 是 −60° 而非 −30°：Example19 的 printf 里误写成 −30，
// 但 TGT_PHYS 实际下发 −60，日志已证实。照 printf 抄会得到站不起来的姿态。
// θ₁ 仍压在 §4 的 UPPER_LIMIT_THETA1_DEG=0 边界上，Hip 扭矩 8~12 N·m
// 是三关节里最吃力的一个，属已知待改项。
constexpr float STAND_HIP_DEG   =   0.0f;
constexpr float STAND_THIGH_DEG = -60.0f;
constexpr float STAND_CALF_DEG  =  60.0f;

// ---- 起立过程 ----
// Example19 用 10 s 慢速插值能站稳；2 s 过快，惯量冲击大容易失稳。
constexpr int   STAND_INTERP_FRAMES = 1000;       // 1000/100Hz = 10 s

// ---- 机身高度调节（相对站立姿态的偏移量，m）----
// 与 §3 的 BODY_SIZE_HEIGHT 无关：这里是运行时可调的离地高度偏移。
constexpr float BODY_HEIGHT_MIN    = -0.15f;      // 最低（深蹲）
constexpr float BODY_HEIGHT_MAX    =  0.10f;      // 最高（站立）
constexpr float HEIGHT_ADJUST_RATE =  0.10f;      // 扳机满程时调节速率 (m/s)

// ---- 轮电机（速度环，由电机固件闭环）----
constexpr float WHEEL_MAX_SPEED = 3.0f;           // 满摇杆目标角速度 (rad/s)
constexpr float WHEEL_KVP       = 3.0f;           // 速度环 Kp —— 与 Example23 实测对齐（1.0 太小，轮子起不来）
constexpr float WHEEL_KVI       = 0.3f;           // 速度环 Ki —— 加积分克服摩擦/稳态误差

// ---- 已排除的方向：使能瞬间轮子转动 ----
// 现象：CAN1 轮电机在使能瞬间转起来，速度反馈冲到 v_max(48 rad/s)。
// 曾尝试"软启动"（使能时把速度环 Kp 设 0，等 100ms 再恢复）——无效，
// Kp=0 期间轮子照样转，说明驱动力矩不来自速度环，而是固件内部的
// 上电/使能初始化动作（疑似转子对齐）直接驱动电流环，绕过速度环。
// 该方向已证伪，勿再尝试从速度环参数入手。
// Example24 亦已确认 24 个固件寄存器四路逐项相同，不是单台参数差异。

// ---- 差速转向 ----
// 左右两侧轮速给不同值实现原地/行进转向：
//   左侧(CAN0 FL, CAN2 RL) = 前进分量 + 转向分量
//   右侧(CAN1 FR, CAN3 RR) = 前进分量 − 转向分量
// 方向位由 MOTOR_CALIBRATION 的 vel_scale 处理，故此处四轮同号即同向。
constexpr float WHEEL_MAX_TURN  = 2.0f;           // 满摇杆转向分量 (rad/s)
// 前进+转向叠加后可能超过单轮上限，按比例缩回以保住转向意图
// （直接钳位会让转弯半径随速度漂移）。
constexpr float WHEEL_SPEED_CAP = 4.0f;           // 单轮合成速度上限 (rad/s)

// =====================================================================
//  §6  参数辨识 —— 动力学与驱动参数
//
//  ⚠ 以下数值目前全部是占位符，尚未实测。
//    在填入真实值之前，不要用它们做重力补偿或动力学前馈计算：
//    错误的质量/质心会让前馈朝反方向使劲。
//    当前重力前馈走的是 motor_calibration.h 的 JOINT_IMPEDANCE.tau_ff
//    实测填值路线，不依赖本节。
// =====================================================================

constexpr float GRAVITY_ACC = 9.80665f;    // 重力加速度 (m/s²)

/** @brief 连杆动力学参数（用于重力补偿 / 动力学模型） */
struct LinkDynamics {
    float mass;        // 质量 (kg)
    float com;         // 质心距关节轴距离 (m)，沿连杆方向
    float inertia;     // 绕关节轴转动惯量 (kg·m²)
};

// 索引: [JointIndex]  —— 三个连杆共用一套（四腿对称）
// TODO: 实测填入。称重 + 悬挂法测质心 + 摆动法测惯量。
constexpr LinkDynamics LINK_DYNAMICS[3] = {
    /* HIP   */ { 0.0f, 0.0f, 0.0f },
    /* THIGH */ { 0.0f, 0.0f, 0.0f },
    /* CALF  */ { 0.0f, 0.0f, 0.0f },
};

constexpr float BODY_MASS = 0.0f;          // 机身质量（不含腿）(kg)，TODO: 实测

/**
 * @brief 电机驱动参数
 *
 * 这些量在电机固件里也有对应寄存器（见 ele_motor_def.h）：
 *   gear_ratio → MOTOR_WR_GR (0x44)      kt → MOTOR_WR_KT_OUT (0x49)
 *   inertia    → MOTOR_WR_J  (0x45)      damping → MOTOR_WR_B (0x46)
 *   friction   → MOTOR_WR_Tf (0x48)
 * 用 float2bag(motor, 0.0f, 0, <寄存器>) 可读回固件当前值来核对。
 */
struct MotorDriveParam {
    float gear_ratio;   // 减速比
    float kt;           // 电磁转矩系数 (A/N·m)
    float inertia;      // 转子转动惯量 (kg·m²)
    float damping;      // 粘滞阻尼系数
    float friction;     // 静摩擦力矩 (N·m)
};

// 索引: [JointIndex] —— TODO: 用读参数帧从固件读回后填入
constexpr MotorDriveParam MOTOR_DRIVE[3] = {
    /* HIP   */ { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    /* THIGH */ { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    /* CALF  */ { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
};

#endif // ROBOT_CALIBRATION_H
