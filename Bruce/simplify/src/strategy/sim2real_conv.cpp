/**
 * @file    sim2real_conv.cpp
 * @brief   sim2real 标定常量定义（真机↔URDF 转换，见 sim2real_conv.h）
 */
#include "strategy/sim2real_conv.h"

namespace rl {

// 名义站姿（POLICY order，URDF 约定）。与 dogurdf.py 的
// NOMINAL_HIP/NOMINAL_THIGH/NOMINAL_CALF 一致（hip=0, thigh=0.20, calf=-0.35,
// wheel=0），即策略训练默认姿态。真机下发指令角由
// urdf_to_status(DEFAULT_POSE, joint) 转换得到，不要在此手填真机标定角。
const float DEFAULT_POSE[16] = {
    // FL  FR  RL  RR 各 hip/thigh/calf
    0.0f, 0.20f, -0.35f,
    0.0f, 0.20f, -0.35f,
    0.0f, 0.20f, -0.35f,
    0.0f, 0.20f, -0.35f,
    // 4 轮
    0.0f, 0.0f, 0.0f, 0.0f,
};

// 真机 GetStatus ↔ URDF 每关节转换（POLICY order：12 腿 + 4 轮）。
// 见 sim2real_conv.h 注释，基于 2026-08-20 真机 L 形测量。
// 每腿 hip/thigh/calf：髋(+1,+0.0297) 大腿(-1,-0.9624) 小腿(+1,-1.2832)
const float CONV_A[16] = {
    +1, -1, +1,    // FL
    +1, -1, +1,    // FR
    +1, -1, +1,    // RL
    +1, -1, +1,    // RR
    +1, +1, +1, +1 // 4 轮
};
const float CONV_B[16] = {
// FL  FR  RL  RR 各 hip/thigh/calf
    +0.0297f, -0.9624f, -1.2832f,
    +0.0297f, -0.9624f, -1.2832f,
    +0.0297f, -0.9624f, -1.2832f,
    +0.0297f, -0.9624f, -1.2832f,
    0.0f, 0.0f, 0.0f, 0.0f
};

} // namespace rl
