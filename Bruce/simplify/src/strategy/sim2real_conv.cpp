/**
 * @file    sim2real_conv.cpp
 * @brief   sim2real 标定常量定义（真机↔URDF 转换，见 sim2real_conv.h）
 */
#include "strategy/sim2real_conv.h"
#include <cstdio>
#include <cstdlib>

namespace rl {

// 名义站姿（POLICY order，URDF 约定）。与 RL_Train/code/src/robots/dogurdf.py 的
// NOMINAL_HIP/NOMINAL_THIGH/NOMINAL_CALF 一致（hip=0, thigh=-0.03, calf=-0.116,
// wheel=0，站高 0.52 m），即策略训练默认姿态。真机下发指令角由
// urdf_to_status(DEFAULT_POSE, joint) 转换得到，不要在此手填真机标定角。
const float DEFAULT_POSE[16] = {
    // FL  FR  RL  RR 各 hip/thigh/calf
    0.0f, -0.03f, -0.116f,
    0.0f, -0.03f, -0.116f,
    0.0f, -0.03f, -0.116f,
    0.0f, -0.03f, -0.116f,
    // 4 轮
    0.0f, 0.0f, 0.0f, 0.0f,
};

// 真机 GetStatus ↔ URDF 每关节转换（POLICY order：12 腿 + 4 轮）。
// 见 sim2real_conv.h 注释，基于 2026-08-20 真机 L 形测量。
//
// ⚠ 2026-10-13 曾"现场确认 FR hip 反了"并把映射取反；**2026-10-17 现场复核推翻该结论**
//   （取反后反而与真实运动相反）⇒ 已回退为 A=+1, B=+0.0297，即与其余三条腿同号。
//   教训：手动推腿判方向容易被"哪边是左/参考姿态"搞混，**两个方向相反的结论就是这么来的**。
//   要彻底定案应改用**运动验证**：给该关节一个已知的 +Δ 指令，看腿实际往哪边走。
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
    -1, -1, +1,    // FR  ← 2026-10-17 定案：**右侧两条腿（FR/RR）的 hip 都要翻**
                   //   依据：MJCF 四条 hip 世界轴全是 (+1,0,0) ⇒ 模型里"左右对称外八"
                   //   对应的是镜像数值 (FL +a, FR −a, RL +a, RR −a)。而实测四条全为 +，
                   //   MuJoCo 画面因此显示"左侧外撇、右侧内收"。
                   //   历史教训：10-13 只翻了 FR、没翻 RR ⇒ 永远对不上 ⇒ 反复回退。
    +1, -1, +1,    // RL
    -1, -1, +1,    // RR  ← hip 翻号（与 FR 同时改，见 FR 处说明）
    +1, +1, +1, +1 // 4 轮
};
const float CONV_B[16] = {
// FL  FR  RL  RR 各 hip/thigh/calf
    +0.0297f, -0.9624f, -1.2832f,
    -0.0297f, -0.9624f, -1.2832f,   // FR hip：随 A 一起回正（零位不变）   // FR hip：随 A 翻号（保零位）
    +0.0297f, -0.9624f, -1.2832f,
    -0.0297f, -0.9624f, -1.2832f,   // RR hip：随 A 翻号（保零位）
    0.0f, 0.0f, 0.0f, 0.0f
};

// ---------------------------------------------------------------------
//  真机起立/摆放目标：默认 = DEFAULT_POSE，可用环境变量覆盖 thigh/calf（4 腿相同）
//  为什么不直接改 DEFAULT_POSE：它参与策略观测 joint_pos_rel = q − DEFAULT_POSE，
//  改它会引入恒定偏置；而站高只关心机械上轮子刚好触地。
// ---------------------------------------------------------------------
const float* stand_pose() {
    static float p[16];
    static bool init = false;
    if (!init) {
        for (int i = 0; i < 16; i++) p[i] = DEFAULT_POSE[i];
        const char* ts = std::getenv("STAND_THIGH_DEG");
        const char* cs = std::getenv("STAND_CALF_DEG");
        if ((ts && *ts) || (cs && *cs)) {
            const float pi = 3.14159265358979323846f;
            const float th = (ts && *ts) ? (float)std::atof(ts) * pi / 180.0f : p[1];
            const float ca = (cs && *cs) ? (float)std::atof(cs) * pi / 180.0f : p[2];
            for (int leg = 0; leg < 4; leg++) {
                p[leg * 3 + 1] = th;
                p[leg * 3 + 2] = ca;
            }
            std::printf("[STAND] 起立姿态被环境变量覆盖：thigh %+.1f° / calf %+.1f°"
                        "（默认 %+.1f° / %+.1f°）\n",
                        th * 180.0f / pi, ca * 180.0f / pi,
                        DEFAULT_POSE[1] * 180.0f / pi, DEFAULT_POSE[2] * 180.0f / pi);
        }
        init = true;
    }
    return p;
}

} // namespace rl
