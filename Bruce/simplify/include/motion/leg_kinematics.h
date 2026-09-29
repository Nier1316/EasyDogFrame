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
 *
 * 本文件只放解算算法。连杆尺寸、关节限位、控制参数等所有可调数值
 * 都在 robot_calibration.h。
 */
#pragma once

#include <cmath>
#include <cstdint>

#include "motion/robot_calibration.h"   // 连杆/限位/控制参数 + 角度工具 + 腿关节编号

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

// =====================================================================
//  单腿解析雅可比（髋坐标系）—— 关节角速度 → 轮心线速度
//
//  J[row][col] = ∂p_row / ∂q_col，p 为髋坐标系下的轮心位置（与 leg_fk 完全同约定）
//  用途：① 接触力反解 τ = Jᵀ·f  ⇒  f = (Jᵀ)⁻¹·τ（接触检测/GRF 估计）
//        ② 力控前馈 τ = Jᵀ·f_des（把足端期望力映射成关节力矩）
//        ③ 腿式里程计（足端速度 = J·q̇）
//
//  解析式由 leg_fk 逐项求导得到，记
//      t1 = q1+off1,  u = −(q2+off2),  v = −(q3+off3)
//      A  = sin(u)·L2 + sin(u+v)·L3        （leg_fk 里的前向分量，p0 = −A）
//      C  = −cos(u)·L2 − cos(u+v)·L3       （leg_fk 里的竖直分量）
//  则
//      ∂A/∂t2 =  C     ∂C/∂t2 = −A
//      ∂A/∂t3 = −cos(u+v)·L3     ∂C/∂t3 = −sin(u+v)·L3
//  三列分别对应 θ1/θ2/θ3（off 为常数，∂/∂q = ∂/∂t）。
//
//  ⚠ 末端点取"轮心"（L3 的末端），与 leg_fk 一致；不是轮底接触点。
//     需要轮底点时把结果沿轮半径方向平移即可（接触点 = 轮心 + r·(−n̂_ground)）。
// =====================================================================
inline void leg_jacobian(const float q_cmd[3],
                         float L1, float L2, float L3,
                         float off1, float off2, float off3,
                         float J[3][3]) {
    const float t1 = q_cmd[0] + off1;
    const float u  = -(q_cmd[1] + off2);   // = t2_int
    const float v  = -(q_cmd[2] + off3);   // = t3_int
    const float uv = u + v;

    const float su = std::sin(u), cu = std::cos(u);
    const float suv = std::sin(uv), cuv = std::cos(uv);
    const float st1 = std::sin(t1), ct1 = std::cos(t1);

    const float A = su * L2 + suv * L3;
    const float C = -cu * L2 - cuv * L3;

    // 列 1：∂p/∂θ1（只有 p1/p2 受影响）
    J[0][0] = 0.0f;
    J[1][0] = -L1 * st1 - C * ct1;
    J[2][0] =  L1 * ct1 - C * st1;

    // 列 2：∂p/∂θ2
    J[0][1] = -C;
    J[1][1] =  A * st1;
    J[2][1] = -A * ct1;

    // 列 3：∂p/∂θ3
    J[0][2] =  cuv * L3;
    J[1][2] =  suv * L3 * st1;
    J[2][2] = -suv * L3 * ct1;
}

// =====================================================================
//  单腿雅可比（身体坐标系）：J_body = R_hip→body · J_hip
//  （轮心 = LEG_MOUNT + R·p_hip，LEG_MOUNT 为常数，不进入雅可比）
// =====================================================================
inline void leg_jacobian_body(LegIndex leg, const float q_cmd[3], float Jb[3][3]) {
    float Jh[3][3];
    leg_jacobian(q_cmd, LEG_L1, LEG_L2, LEG_L3,
                 THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET, Jh);

    float R[3][3];
    hip_rotation_matrix(leg, R);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            Jb[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) Jb[i][j] += R[i][k] * Jh[k][j];
        }
}

// =====================================================================
//  3×3 矩阵求逆（伴随矩阵法）—— 仅在行列式数值上≈0 时拒绝
//  ⚠ 这里**不做**"是否病态"的业务判断：行列式的绝对大小带量纲（m³/rad³），
//     不能跨位形比较。腿的奇异性判据见 leg_cond_proxy() / leg_foot_force_body()。
// =====================================================================
inline bool mat3_inv(const float M[3][3], float Minv[3][3]) {
    const float det =
        M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
      - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
      + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

    if (std::fabs(det) < 1e-12f) return false;
    const float id = 1.0f / det;

    Minv[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * id;
    Minv[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) * id;
    Minv[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * id;
    Minv[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) * id;
    Minv[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * id;
    Minv[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) * id;
    Minv[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * id;
    Minv[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) * id;
    Minv[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * id;
    return true;
}

/**
 * @brief 腿雅可比的**无量纲条件数代理**：cond = ‖c1‖‖c2‖‖c3‖ / |det|
 *        1 = 正交（最好），→ ∞ = 奇异（腿伸直，膝角 t3_int→0）。
 * @return 代理条件数；矩阵奇异时返回 1e30
 *
 * 标定记录（2026-09-29 实测，FL 腿，身体坐标系）：
 *   膝伸直 t3_int=0（q3≈−0.211）  cond ≈ 1e5+（|det| 7.5e-9）
 *   距伸直 0.02 rad              cond ≈ 103
 *   距伸直 0.05 rad              cond ≈ 41
 *   DEFAULT_POSE (0,0.20,−0.35)   cond ≈ 15
 *   STAND (0,−1.047,1.047)        cond ≈ 1.8
 * → **接触力估计的误差放大倍数就是这个 cond**（力矩误差 ×cond ≈ 力误差）。
 *   这也是"必须先把重力项与摩擦项扣掉"的量化理由：本项目摩擦 1.64~6.17 N·m，
 *   在 cond≈15 的位形下会带来 25~90 N 的力误差。
 */
inline float leg_cond_proxy(const float J[3][3]) {
    const float det =
        J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1])
      - J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0])
      + J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
    float n[3] = {0, 0, 0};
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) n[j] += J[i][j] * J[i][j];
        n[j] = std::sqrt(n[j]);
    }
    const float scale = n[0] * n[1] * n[2];
    if (scale < 1e-12f || std::fabs(det) < 1e-12f) return 1e30f;
    return scale / std::fabs(det);
}

/**
 * @brief 关节力矩 → 轮端力（身体坐标系）：f = (Jᵀ)⁻¹·τ
 * @param leg    腿编号（决定髋→身体坐标变换）
 * @param q_cmd  3 个关节指令角 (rad)
 * @param tau    3 个关节力矩 (N·m，**须已扣掉重力项与摩擦项**，否则结果不可用)
 * @param f_body 输出轮端力 (N，身体坐标系)
 * @param cond   可选输出：本次位形的条件数代理（见 leg_cond_proxy），供调用方门控/加权
 * @return false 表示该位形过于病态（默认 |cond| > 50，即力误差可能被放大 50 倍以上）
 *
 * ⚠ 前提：τ 必须只是"外力产生的力矩残差"。本函数不做重力/摩擦补偿——
 *   调用方应先用重力项 g_est(q) 与摩擦前馈表扣掉后再调用。
 */
inline bool leg_foot_force_body(LegIndex leg, const float q_cmd[3],
                                const float tau[3], float f_body[3],
                                float* cond = nullptr) {
    float Jb[3][3];
    leg_jacobian_body(leg, q_cmd, Jb);

    const float c = leg_cond_proxy(Jb);
    if (cond) *cond = c;
    if (c > 50.0f) return false;   // 近伸直位形：拒绝，不给发散结果

    float Jinv[3][3];
    if (!mat3_inv(Jb, Jinv)) return false;

    // f = (J⁻¹)ᵀ·τ  ⇒  f[i] = Σ_j Jinv[j][i]·τ[j]
    for (int i = 0; i < 3; i++) {
        f_body[i] = 0.0f;
        for (int j = 0; j < 3; j++) f_body[i] += Jinv[j][i] * tau[j];
    }
    return true;
}

/**
 * @brief 轮端力 → 关节力矩（身体坐标系）：τ = Jᵀ·f
 *        力控前馈用；也是 leg_foot_force_body 的逆运算（可用于自检）
 */
inline void leg_foot_force_to_torque(LegIndex leg, const float q_cmd[3],
                                     const float f_body[3], float tau[3]) {
    float Jb[3][3];
    leg_jacobian_body(leg, q_cmd, Jb);
    for (int j = 0; j < 3; j++) {
        tau[j] = 0.0f;
        for (int i = 0; i < 3; i++) tau[j] += Jb[i][j] * f_body[i];
    }
}
