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
//
// ⚠ 2026-10-13 现场确认：**右前(FR) hip 的观测方向是反的** —— 用手把 FR 腿远端往机器人
//   左侧推，URDF 读数反而变小（其余腿正常）。
//   模型侧真值（已实测核对）：MJCF 里四条 hip 的世界系轴全是 (+1,0,0)，FK 实测
//   「hip 正 ⇒ 该腿远端向机器人左侧(+y)移动」四条腿完全一致；`dogurdf.py` 四条腿共用
//   同一个 NOMINAL_HIP。所以 hip 的 URDF 语义必须**四条腿统一**，而折算前的净符号
//   （MOTOR_CALIBRATION.pos_scale × CONV_A）是**对角同号**：FL −1 / FR +1 / RL +1 / RR −1。
//   那个对角模式自 57cd63b(2026-06-13 "补充物理0位") 引入后从未复核 ⇒ 本条修 FR。
//
// 修法：把该关节的**整个 URDF 映射取反**（A 与 B 同时翻号），于是
//       urdf_new(raw) ≡ −urdf_old(raw) ⇒ **零位一点不动、方向翻转**，而且
//       urdf_to_status 仍精确互逆 ⇒ **观测与动作同步翻转，不会互相打架**。
//   ⚠ 只翻 A 不翻 B 会让零位偏 2×CONV_B = 0.0594 rad (3.4°)，别那么改。
//   ⚠ 电机层 MOTOR_CALIBRATION 的 pos_scale 保持不动：那是「物理编码器正方向」的实测
//     声明，且只改它同样会把零位挪 2×CONV_B。
//   ⚠ **RL(左后) hip 尚未现场确认**（它在电机层与 FR 同号，是首要嫌疑）；确认同样反向后
//     按同样方式改（A: +1→−1、B: +0.0297→−0.0297）。thigh/calf 也建议一并核对
//     （判据：远端**向后**推 ⇒ URDF 读数应**变大**）。
const float CONV_A[16] = {
    +1, -1, +1,    // FL
    -1, -1, +1,    // FR  ← hip 翻号（2026-10-13 现场确认），见上方说明
    +1, -1, +1,    // RL
    +1, -1, +1,    // RR
    +1, +1, +1, +1 // 4 轮
};
const float CONV_B[16] = {
// FL  FR  RL  RR 各 hip/thigh/calf
    +0.0297f, -0.9624f, -1.2832f,
    -0.0297f, -0.9624f, -1.2832f,   // FR hip：随 A 一起翻号（保零位）
    +0.0297f, -0.9624f, -1.2832f,
    +0.0297f, -0.9624f, -1.2832f,
    0.0f, 0.0f, 0.0f, 0.0f
};

} // namespace rl
