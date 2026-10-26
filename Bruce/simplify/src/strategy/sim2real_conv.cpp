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
// **下面的数组是唯一准据**；本段只记结论与教训，不再记"待确认/已回退"之类的过程状态。
//
// 当前状态（2026-10-26 定案）：
//   · **符号正确**（用户现场确认）。硬理由：PD 符号错 ⇒ 正反馈 ⇒ 立刻失控，
//     "机器人能走"本身即证明 12 个腿关节符号全部正确。
//     注意：CONV_A（左右镜像）与 MOTOR_CALIBRATION.pos_scale（对角模式，自 57cd63b 引入）
//     相乘后的净符号按**前后**分组（hip: FL/FR=−, RL/RR=+）——这是电机安装/接线在
//     前后方向上的真实差异，**不是遗留错误，不要"统一"**。
//   · **零位已用 SolidWorks 几何复核**：把腿卡在限位块（= 编码器 0）处，该姿态的
//     SW 关节角必须等于 b = CONV_A*pos_offset + CONV_B：
//        hip  ±0.6407（左 + / 右 −）； thigh −1.4034（四腿）； calf −1.0722（四腿）
//     SW 结果：hip 左右镜像正常、calf 正常、thigh 与"装配零位后倾 52°"自洽
//     ⇒ **CONV_A / CONV_B / pos_scale / pos_offset 全部无需修改**（零位线关闭）。
//     由此推论：PACE 辨识里膝 bias 顶在 −0.5 界上**不是零点问题**，而是拟合的零方向
//     （详见 docs/JOINT_FRAME_CHAIN.md §7 与 PACE 相关记录）。
//
// 保留的技术教训（以后改这类映射必须先读）：
//   · 要翻方向就**整个 URDF 映射取反**（A 与 B 同时翻号）⇒ 零位一点不动、方向翻转，
//     且 urdf_to_status 仍精确互逆 ⇒ 观测与动作同步翻转，不会互相打架。
//   · ⚠ 只翻 A 不翻 B 会把零位挪 2*CONV_B = 0.0594 rad (3.4°)，别那么改。
//   · ⚠ 不要用电机层 pos_scale 去"修"URDF 方向：那是「物理编码器正方向」的实测声明，
//     而且只改它同样会把零位挪 2*CONV_B。
//   · 判方向**只做运动验证**（给该关节已知 +Δ 指令，看腿实际往哪边走）；手推腿判方向
//     容易被"哪边是左/参考姿态"搞混 —— 历史上 10-13 与 10-17 两次相反结论就是这么来的。
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
    -0.0297f, -0.9624f, -1.2832f,   // FR hip：与 A 同步翻号（零位不变）
    +0.0297f, -0.9624f, -1.2832f,
    -0.0297f, -0.9624f, -1.2832f,   // RR hip：与 A 同步翻号（零位不变）
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
        // ---- 真机站高标定（2026-10-19 现场定：站高 460 mm）----
        // 用 MJCF（dogurdf.xml，轮半径 113 mm）按两个约束反解：
        //   ① 轮子刚触地时的机身高度 = 460 mm
        //   ② 轮心正好在髋关节正下方（与 dogurdf.py 的名义姿态定义一致）
        // 解得 thigh = +0.89°、calf = -19.00°（膝更弯 ⇒ 机身更低）。
        // ⚠ 机身固定在架上时，**站高越小 ⇒ 轮子相对机身越高 ⇒ 腿的离地余量越大**，
        //   故 PACE 采集要把腿收得多一些（不要为了"余量"去抬高站高，那是反的）。
        // 站高一览：449 mm(旧 0.20/-0.35) → 460(本次) → 480(上一档) → 520(训练默认)。
        //   thigh  +0.89° / calf -19.00°  → 460 mm
        //   thigh  -1.09° / calf -15.03°  → 480 mm
        //   thigh  -1.70° / calf  -6.60°  → 520 mm
        // ⚠ 只改 stand_pose()，不动 DEFAULT_POSE（后者是策略观测基准）。
        constexpr float STAND_THIGH_RAD = +0.0156f;   // +0.89°
        constexpr float STAND_CALF_RAD  = -0.3316f;   // -19.00°
        for (int leg = 0; leg < 4; leg++) {
            p[leg * 3 + 0] = 0.0f;                    // hip 保持 0（左右对称）
            p[leg * 3 + 1] = STAND_THIGH_RAD;
            p[leg * 3 + 2] = STAND_CALF_RAD;
        }
        const char* ts = std::getenv("STAND_THIGH_DEG");
        const char* cs = std::getenv("STAND_CALF_DEG");
        if ((ts && *ts) || (cs && *cs)) {
            const float pi = 3.14159265358979323846f;
            const float dflt_t = p[1], dflt_c = p[2];          // stand_pose 自身默认（不是 DEFAULT_POSE）
            const float th = (ts && *ts) ? (float)std::atof(ts) * pi / 180.0f : dflt_t;
            const float ca = (cs && *cs) ? (float)std::atof(cs) * pi / 180.0f : dflt_c;
            for (int leg = 0; leg < 4; leg++) {
                p[leg * 3 + 1] = th;
                p[leg * 3 + 2] = ca;
            }
            std::printf("[STAND] 起立姿态被环境变量覆盖：thigh %+.1f° / calf %+.1f°"
                        "（本机默认 %+.1f° / %+.1f°，站高 460 mm；DEFAULT_POSE %+.1f° / %+.1f° 未动）\n",
                        th * 180.0f / pi, ca * 180.0f / pi,
                        dflt_t * 180.0f / pi, dflt_c * 180.0f / pi,
                        DEFAULT_POSE[1] * 180.0f / pi, DEFAULT_POSE[2] * 180.0f / pi);
        }
        init = true;
    }
    return p;
}

} // namespace rl
