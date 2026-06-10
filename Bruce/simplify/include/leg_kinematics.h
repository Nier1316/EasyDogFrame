/**
 * @file    leg_kinematics.h
 * @brief   四足机器狗 三关节腿正逆运动学解算库（纯头文件）
 *
 * 坐标系定义（与 MATLAB 仿真 leg_kinematics.m 完全一致）:
 *
 * 髋关节坐标系:
 *   X+ = 向后 (狗体后方)
 *   Y+ = 向外翻 (髋外翻同向)
 *   Z+ = 向上
 *
 * 身体坐标系:
 *   X+ = 向前
 *   Y+ = 向左
 *   Z+ = 向上
 *
 * 关节角用户约定:
 *   θ₁ 髋外摆: 正=向外翻
 *   θ₂ 大腿:   正=向后摆
 *   θ₃ 小腿:   正=向后弯
 *
 * 物理角 = ZERO_OFFSET + 指令角
 * 指令角 ∈ [LOWER_LIMIT, UPPER_LIMIT]
 */
#pragma once

#include <cmath>
#include <cstdint>

// =====================================================================
//  连杆参数 (单位: 米) — 根据实际硬件修改
// =====================================================================
constexpr float LEG_L1 = 0.05f;   // 髋侧向偏移量
constexpr float LEG_L2 = 0.20f;   // 大腿长度
constexpr float LEG_L3 = 0.20f;   // 小腿长度

// =====================================================================
//  身体尺寸 (单位: 米)
// =====================================================================
constexpr float BODY_LENGTH = 0.30f;   // 身体长度 (X方向)
constexpr float BODY_WIDTH  = 0.12f;   // 身体宽度 (Y方向)
constexpr float BODY_HEIGHT = 0.06f;   // 身体高度 (Z方向)

// =====================================================================
//  腿安装位置 (在身体坐标系中)
//  每行: [x, y, z]   x正=向前, y正=向左, z正=向上
// =====================================================================
constexpr float LEG_MOUNT[4][3] = {
    { BODY_LENGTH/2,  BODY_WIDTH/2, 0 },   // FL (左前)
    { BODY_LENGTH/2, -BODY_WIDTH/2, 0 },   // FR (右前)
    {-BODY_LENGTH/2,  BODY_WIDTH/2, 0 },   // RL (左后)
    {-BODY_LENGTH/2, -BODY_WIDTH/2, 0 },   // RR (右后)
};

// =====================================================================
//  关节零位偏移 & 限位宏
//  可直接用于电机控制：物理角 = ZERO_OFFSET + 指令角
//  设置电机时传物理角，ApplyMotorCalibrationInverse 会自动转换
// =====================================================================

// --- θ1 髋外摆 ---
constexpr float ZERO_OFFSET_THETA1_DEG = 30.0f;
constexpr float LOWER_LIMIT_THETA1_DEG = -60.0f;
constexpr float UPPER_LIMIT_THETA1_DEG =   0.0f;

// --- θ2 大腿 ---
constexpr float ZERO_OFFSET_THETA2_DEG = 0.0f;
constexpr float LOWER_LIMIT_THETA2_DEG = -45.0f;
constexpr float UPPER_LIMIT_THETA2_DEG =  90.0f;

// --- θ3 小腿 ---
constexpr float ZERO_OFFSET_THETA3_DEG = 0.0f;
constexpr float LOWER_LIMIT_THETA3_DEG = 60.0f;
constexpr float UPPER_LIMIT_THETA3_DEG = 180.0f;

// 弧度版本（解算内部使用）
constexpr float THETA1_OFFSET = ZERO_OFFSET_THETA1_DEG * (M_PI / 180.0f);
constexpr float THETA2_OFFSET = ZERO_OFFSET_THETA2_DEG * (M_PI / 180.0f);
constexpr float THETA3_OFFSET = ZERO_OFFSET_THETA3_DEG * (M_PI / 180.0f);

// =====================================================================
//  角度工具
// =====================================================================
constexpr float rad2deg(float rad) { return rad * (180.0f / M_PI); }
constexpr float deg2rad(float deg) { return deg * (M_PI / 180.0f); }

inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// =====================================================================
//  腿编号
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
//  单腿正运动学
//
//  输入: q_cmd[3] — 指令角 [θ₁, θ₂, θ₃] (rad)
//        L1, L2, L3 — 连杆长度 (m)
//        off1, off2, off3 — 零位偏移 (rad)
//  输出: p[3] — 足端位置 [px, py, pz] (髋关节坐标系, m)
// =====================================================================
inline void leg_fk(const float q_cmd[3],
                   float L1, float L2, float L3,
                   float off1, float off2, float off3,
                   float p[3]) {
    // 指令角 → 物理角
    float t1 = q_cmd[0] + off1;
    float t2 = q_cmd[1] + off2;
    float t3 = q_cmd[2] + off3;

    // 用户约定→内部公式约定 (正值=向前 需取反)
    float t2_int = -t2;
    float t3_int = -t3;

    // 腿平面内的位置分量
    float A = std::sin(t2_int) * L2 + std::sin(t2_int + t3_int) * L3;  // X₁ (前向)
    float B = L1;                                                        // Y₁ (外翻偏移)
    float C = -std::cos(t2_int) * L2 - std::cos(t2_int + t3_int) * L3;  // Z₁ (竖直, 向下负)

    // 基座标 X+ = 向后, 故取负; 绕 X 轴旋转 θ₁
    p[0] = -A;
    p[1] = B * std::cos(t1) - C * std::sin(t1);
    p[2] = B * std::sin(t1) + C * std::cos(t1);
}

// =====================================================================
//  单腿逆运动学
//
//  输入: p[3] — 足端位置 [px, py, pz] (髋关节坐标系, m)
//        L1, L2, L3 — 连杆长度 (m)
//        off1, off2, off3 — 零位偏移 (rad)
//  输出: q_cmd[3] — 指令角 [θ₁, θ₂, θ₃] (rad)
//
//  注意: θ₃ 取负解对应 "小腿向后弯"
//        非交叉腿解 (腿在身体外侧)
// =====================================================================
inline void leg_ik(const float p[3],
                   float L1, float L2, float L3,
                   float off1, float off2, float off3,
                   float q_cmd[3]) {
    float px = p[0], py = p[1], pz = p[2];

    // ---- Step 1: 求 θ₁_phys ----
    float r_yz = std::sqrt(py * py + pz * pz);
    if (r_yz < std::abs(L1) + 1e-9f) {
        r_yz = std::abs(L1) + 1e-6f;
    }
    // 非交叉解: 腿在身体外侧
    float t1_phys = M_PI - std::asin(L1 / r_yz) - std::atan2(py, pz);

    // ---- Step 2: 还原腿平面内分量 ----
    float D = std::sqrt(std::max(r_yz * r_yz - L1 * L1, 0.0f));
    float A = -px;  // 基座标 X+→髋坐标 X₁+

    // ---- Step 3: 余弦定理求 θ₃_internal ----
    float numerator   = A * A + D * D - L2 * L2 - L3 * L3;
    float denominator = 2.0f * L2 * L3;

    float t3_int;
    if (std::abs(denominator) < 1e-12f) {
        t3_int = 0;
    } else {
        float cos_t3 = numerator / denominator;
        cos_t3 = clamp(cos_t3, -1.0f, 1.0f);
        t3_int = -std::acos(cos_t3);  // 负解 = 向后弯
    }

    // ---- Step 4: 求 θ₂_internal ----
    float k1 = L2 + L3 * std::cos(t3_int);
    float k2 = L3 * std::sin(t3_int);
    float denom = k1 * k1 + k2 * k2;

    float t2_int;
    if (denom < 1e-12f) {
        t2_int = 0;
    } else {
        float sin_t2 = (A * k1 - D * k2) / denom;
        float cos_t2 = (A * k2 + D * k1) / denom;
        t2_int = std::atan2(sin_t2, cos_t2);
    }

    // 内部约定 → 用户约定 → 物理角 → 指令角
    float t2_phys = -t2_int;
    float t3_phys = -t3_int;
    q_cmd[0] = t1_phys - off1;
    q_cmd[1] = t2_phys - off2;
    q_cmd[2] = t3_phys - off3;
}

// =====================================================================
//  髋→身体坐标系旋转矩阵
//  髋坐标系: X+向后, Y+向外翻, Z+向上
//  身体坐标系: X+向前, Y+向左, Z+向上
// =====================================================================
inline void hip_rotation_matrix(LegIndex leg, float R[3][3]) {
    switch (leg) {
        case FL:
        case RL:
            // 左腿: 向外 = 向左 = +Ybody
            R[0][0] = -1; R[0][1] =  0; R[0][2] = 0;
            R[1][0] =  0; R[1][1] =  1; R[1][2] = 0;
            R[2][0] =  0; R[2][1] =  0; R[2][2] = 1;
            break;
        case FR:
        case RR:
            // 右腿: 向外 = 向右 = -Ybody
            R[0][0] = -1; R[0][1] =  0; R[0][2] = 0;
            R[1][0] =  0; R[1][1] = -1; R[1][2] = 0;
            R[2][0] =  0; R[2][1] =  0; R[2][2] = 1;
            break;
    }
}

// =====================================================================
//  四腿正运动学: 12关节角 → 4足端位置 (身体坐标系)
//
//  输入: q_all[12] — 12 个关节指令角 [FLθ1,FLθ2,FLθ3, FR... RL... RR...] (rad)
//  输出: foot_body[4][3] — 4 足端位置 (身体坐标系, m)
// =====================================================================
inline void leg_fk_all(const float q_all[12], float foot_body[4][3]) {
    for (int leg = 0; leg < 4; leg++) {
        const float* q = q_all + leg * 3;

        // 单腿 FK (髋坐标系)
        float p_hip[3];
        leg_fk(q, LEG_L1, LEG_L2, LEG_L3,
               THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET,
               p_hip);

        // 髋→身体坐标变换
        float R[3][3];
        hip_rotation_matrix(static_cast<LegIndex>(leg), R);

        float p_body[3];
        for (int i = 0; i < 3; i++) {
            p_body[i] = LEG_MOUNT[leg][i];
            for (int j = 0; j < 3; j++) {
                p_body[i] += R[i][j] * p_hip[j];
            }
        }

        foot_body[leg][0] = p_body[0];
        foot_body[leg][1] = p_body[1];
        foot_body[leg][2] = p_body[2];
    }
}