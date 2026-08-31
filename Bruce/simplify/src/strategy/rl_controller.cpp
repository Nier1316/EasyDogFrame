/**
 * @file    rl_controller.cpp
 * @brief   dogurdf 策略观测构建与控制律的实现
 */
#include "strategy/rl_controller.h"
#include <cmath>

namespace rl {

// DEFAULT_POSE / CONV_A / CONV_B 已移至 strategy/sim2real_conv.cpp

// 关节限位（POLICY order；轮子用大哨兵）
const float JOINT_LOWER[NUM_JOINTS] = {
    -0.6f, -0.7f, -1.0f,
    -0.6f, -0.7f, -1.0f,
    -0.6f, -0.7f, -1.0f,
    -0.6f, -0.7f, -1.0f,
    -1.0e6f, -1.0e6f, -1.0e6f, -1.0e6f,
};
const float JOINT_UPPER[NUM_JOINTS] = {
    0.6f, 1.75f, 0.35f,
    0.6f, 1.75f, 0.35f,
    0.6f, 1.75f, 0.35f,
    0.6f, 1.75f, 0.35f,
    1.0e6f, 1.0e6f, 1.0e6f, 1.0e6f,
};

// 步态相位偏移（对角步态：FL+RR 同相 0，FR+RL 同相 0.5）
const float GAIT_OFFSET[4] = {0.0f, 0.5f, 0.5f, 0.0f};

// POLICY <-> MJX/CAN 顺序置换（与 dogurdf.py 的 _policy_to_mjx_permutation 一致）
const int POLICY_TO_MJX[NUM_JOINTS] = {
    0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15
};
const int MJX_TO_POLICY[NUM_JOINTS] = {
    0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15
};

// 轮子摩擦前馈（Example35 实测 2026-08-21）：wheel_idx 0..3 = FL,FR,RL,RR == CAN0..3。
// 含义：轮子悬空时"恰好克服静摩擦开始转动"的正/负扭矩。CAN2(RL) 明显偏大。
const float WHEEL_FF[4][2] = {
    { +0.600f, -0.600f },  // FL(C0)
    { +0.500f, -0.400f },  // FR(C1)
    { +0.800f, -0.800f },  // RL(C2) — 阻力较大
    { +0.500f, -0.400f },  // RR(C3)
};

// 腿摩擦前馈库仑摩擦 fc（Example47 辨识，RL_TRAINING_REFERENCE §2.1 绝对值，
// POLICY order：FL,FR,RL,RR × hip/thigh/calf，共 12）。异常值（RR calf=4.6）置 0。
// ⚠ 方向由 tanh(τ_pd) 决定，此处只存幅值；真机验证后按需微调。
const float LEG_FF_FC[12] = {
    0.55f, 0.29f, 0.07f,   // FL hip/thigh/calf
    0.03f, 0.09f, 0.15f,   // FR
    0.01f, 0.36f, 0.04f,   // RL
    0.24f, 0.64f, 0.00f,   // RR（calf 辨识异常置 0）
};
// 粘性阻尼 fv：Example47 辨识 B 不可靠（速度反馈延迟，多数负值），暂全 0。
const float LEG_FF_FV[12] = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
};

void world2self(const float* q, const float* v, float* out) {
    // q = [w, x, y, z]；等价于 demo.py/sim2sim.py 的 world2self：
    //   out = v*(2w²-1) - cross(q_vec,v)*2w + q_vec*(q_vec·v)*2
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const float s = 2.0f * qw * qw - 1.0f;

    const float cx = qy * v[2] - qz * v[1];
    const float cy = qz * v[0] - qx * v[2];
    const float cz = qx * v[1] - qy * v[0];
    const float w2 = qw * 2.0f;

    const float d = (qx * v[0] + qy * v[1] + qz * v[2]) * 2.0f;

    out[0] = v[0] * s - cx * w2 + qx * d;
    out[1] = v[1] * s - cy * w2 + qy * d;
    out[2] = v[2] * s - cz * w2 + qz * d;
}

void build_observation(const float* gyro, const float* quat,
                       const float* pos_policy, const float* vel_policy,
                       const float* last_action, const float* cmd,
                       int step, float* obs) {
    // 1) base_lin_vel = 0（训练端恒置零）
    obs[0] = obs[1] = obs[2] = 0.0f;

    // 2) base_ang_vel = 机体系角速度（IMU gyro，rad/s）
    obs[3] = gyro[0];
    obs[4] = gyro[1];
    obs[5] = gyro[2];

    // 3) projected_gravity = world2self(quat, [0,0,-1])
    float g[3];
    const float down[3] = {0.0f, 0.0f, -1.0f};
    world2self(quat, down, g);
    obs[6] = g[0];
    obs[7] = g[1];
    obs[8] = g[2];

    // 4) joint_pos_rel[12] = pos_policy[:12] - default_pose[:12]
    for (int i = 0; i < NUM_LEG_JOINTS; ++i) {
        obs[9 + i] = pos_policy[i] - DEFAULT_POSE[i];
    }

    // 5) joint_vel[16]（全部关节，含轮）
    for (int i = 0; i < NUM_JOINTS; ++i) {
        obs[21 + i] = vel_policy[i];
    }

    // 6) last_action[16]
    for (int i = 0; i < NUM_JOINTS; ++i) {
        obs[37 + i] = last_action[i];
    }

    // 7) command[3]
    obs[53] = cmd[0];
    obs[54] = cmd[1];
    obs[55] = cmd[2];

    // 8) gait_phase[8] = [sin(2πφ), cos(2πφ)] × 4
    const float two_pi = 2.0f * 3.14159265358979323846f;
    for (int foot = 0; foot < 4; ++foot) {
        float phi = std::fmod(step * CONTROL_DT / GAIT_CYCLE + GAIT_OFFSET[foot], 1.0f);
        if (phi < 0.0f) phi += 1.0f;
        obs[56 + foot * 2 + 0] = std::sin(two_pi * phi);
        obs[56 + foot * 2 + 1] = std::cos(two_pi * phi);
    }

    // clip 到 [-100, 100]（与 sim2sim.py 一致）
    for (int i = 0; i < OBS_DIM; ++i) {
        if (obs[i] < -100.0f) obs[i] = -100.0f;
        else if (obs[i] > 100.0f) obs[i] = 100.0f;
    }
}

} // namespace rl
