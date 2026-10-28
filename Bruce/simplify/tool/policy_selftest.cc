// ============================================================================
//  tool/policy_selftest.cc —— 策略权重离线数值回归（**不碰 CAN / 不开 USB**）
//
//  为什么单独做一个可执行文件：验证"新策略是否正确部署进框架"只需要 MLP 前向，
//  没必要、也不应该启动真机程序（bin/can_motor_app 会去打开 USB2CAN 设备）。
//  本目标只链接 policy_set.cpp（纯权重数据 + 变体表）与 rl_controller.cpp
//  （纯观测构建，无 CAN/USB 依赖），因此可以在任何没有硬件的时候跑，也适合放进 CI。
//
//  原理：tool/export_policy.py 导出权重时会同时写一份 REF_OBS(64)/REF_ACTION(16)
//  （来自 Python 侧 flax 推理的真实输入/输出）。这里对**每一套**变体跑
//  mlp_forward(REF_OBS)，与 REF_ACTION 逐元素比较，最大绝对误差应 < 1e-3。
//
//  另外做一次 **nominal 站姿离线前向自检**（真实站姿观测 → 动作必须有限、|a|≤1）。
//
//  用法：cmake --build build -j && ./bin/policy_selftest
//  退出码：0 = 全部通过；1 = 有变体超差。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "strategy/mlp.h"
#include "strategy/policy_set.h"
#include "strategy/rl_controller.h"   // build_observation / DEFAULT_POSE（纯 CPU）

int main() {
    printf("=== 策略权重离线数值回归（不碰 CAN）===\n");
    printf("    mlp 结构 %d -> %d -> %d -> %d -> %d，ELU 隐藏层，输出无激活\n\n",
           rl::ACTOR_DIM, rl::HIDDEN_0, rl::HIDDEN_1, rl::HIDDEN_2, rl::ACTION_DIM);

    bool all_ok = true;
    // 上界取"当前最后一档"。新增策略时只需在这里补一行（否则新变体会被静默跳过）。
    const int last_vid = rl::POLICY_DOGURDF_PACE_STG4_6503;
    for (int vid = rl::POLICY_SMALLLIFT_9754; vid <= last_vid; vid++) {
        const rl::PolicyWeights& w = rl::PolicyOf(vid);
        rl::SetPolicyVariant(vid);

        float act[rl::ACTION_DIM];
        rl::mlp_forward(w.ref_obs, act);

        float max_err = 0.0f;
        int   max_idx = -1;
        for (int i = 0; i < rl::ACTION_DIM; i++) {
            const float e = std::fabs(act[i] - w.ref_action[i]);
            if (e > max_err) { max_err = e; max_idx = i; }
        }
        const bool ok = max_err < 1e-3f;
        all_ok = all_ok && ok;

        printf("  [变体 %d] %-36s 最大误差 %.3e  idx=%2d  %s\n",
               vid, w.name, max_err, max_idx, ok ? "✅" : "❌");
        if (!ok)
            for (int i = 0; i < rl::ACTION_DIM; i++)
                printf("        a[%2d]  got=%+.6f  ref=%+.6f\n",
                       i, act[i], w.ref_action[i]);
    }

    // 额外：最新变体的 action_clip 路径也要能跑（截断到 [-1,1]）
    {
        rl::SetPolicyVariant(last_vid);
        const rl::PolicyWeights& w = rl::PolicyOf(last_vid);
        float act[rl::ACTION_DIM];
        rl::mlp_forward_clipped(w.ref_obs, act);
        bool in_range = true;
        for (int i = 0; i < rl::ACTION_DIM; i++)
            if (act[i] < -1.0f || act[i] > 1.0f) in_range = false;
        printf("\n  mlp_forward_clipped(变体 %d)：全部动作 ∈ [-1,1] %s\n", last_vid,
               in_range ? "✅" : "❌");
        all_ok = all_ok && in_range;
    }

    // 额外：**逐变体观测通道**自检（2026-10-28 引入）。
    // 同一组已知输入分别用系数 1.0/1.0（变体 5 = RAW legacy）与 0.25/0.05
    // （变体 6 = pace 缩放通道）走**真实的** build_observation，断言：
    //   · 只有 ang_vel[3..6) 与 joint_vel[21..37) 两段被按比例缩放（含轮速 [33..37)）；
    //   · 其余 48 维**逐位不变**（bitwise）——旧变体的行为必须与改动前完全相同。
    {
        float gyro[3] = {1.0f, -2.0f, 0.5f};
        float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // 机身水平
        float pos[rl::NUM_JOINTS], vel[rl::NUM_JOINTS], la[rl::NUM_JOINTS];
        for (int i = 0; i < rl::NUM_JOINTS; i++) {
            pos[i] = rl::DEFAULT_POSE[i] + 0.01f * (float)(i + 1);
            vel[i] = 1.5f + 0.25f * (float)i;       // 腿 1.50..4.25，轮 4.50..5.25
            la[i]  = 0.5f - 0.05f * (float)i;
        }
        float cmd[3] = {0.5f, 0.0f, 0.3f};

        float raw[rl::OBS_DIM], scaled[rl::OBS_DIM];
        rl::SetPolicyVariant(rl::POLICY_DOGURDF_LEGACY_S4_1500);   // 1.0 / 1.0
        rl::build_observation(gyro, quat, pos, vel, la, cmd, 3, raw);
        rl::SetPolicyVariant(rl::POLICY_DOGURDF_PACE_STG4_6503);   // 0.25 / 0.05
        rl::build_observation(gyro, quat, pos, vel, la, cmd, 3, scaled);

        bool untouched_ok = true, ang_ok = true, jvel_ok = true, wheel_ok = true;
        int  first_bad = -1;
        for (int i = 0; i < rl::OBS_DIM; i++) {
            const bool is_ang  = (i >= 3  && i < 6);
            const bool is_jvel = (i >= 21 && i < 37);
            if (!is_ang && !is_jvel) {
                if (scaled[i] != raw[i]) { untouched_ok = false; if (first_bad < 0) first_bad = i; }
            } else if (is_ang) {
                if (scaled[i] != raw[i] * 0.25f) { ang_ok = false; if (first_bad < 0) first_bad = i; }
            } else if (scaled[i] != raw[i] * 0.05f) {
                jvel_ok = false; if (first_bad < 0) first_bad = i;
            }
        }
        // 轮速 4 维（obs[33..37)）单独判一次，防"只乘 12 腿"这个经典错位。
        for (int i = 33; i < 37; i++)
            if (scaled[i] != raw[i] * 0.05f) wheel_ok = false;

        const bool ch_ok = untouched_ok && ang_ok && jvel_ok && wheel_ok;
        printf("\n  逐变体观测通道自检（真实 build_observation：变体 5 = 1.0/1.0 vs 变体 6 = 0.25/0.05）：\n");
        printf("    ang_vel[3..6)      ×0.25 %s（raw[3]=%+.6f → %+.6f）\n",
               ang_ok ? "✅" : "❌", raw[3], scaled[3]);
        printf("    joint_vel[21..37)  ×0.05 %s（raw[21]=%+.6f → %+.6f）\n",
               jvel_ok ? "✅" : "❌", raw[21], scaled[21]);
        printf("    轮速[33..37)       ×0.05 %s（raw[33]=%+.6f → %+.6f）\n",
               wheel_ok ? "✅" : "❌", raw[33], scaled[33]);
        printf("    其余 48 维逐位不变   %s%s\n",
               untouched_ok ? "✅" : "❌",
               first_bad >= 0 ? "（首个不符维度见上）" : "");
        all_ok = all_ok && ch_ok;
    }

    // 额外：**nominal 站姿离线前向自检**（新策略当默认部署场景）。
    // REF_OBS 是 [-1,1] 均匀随机数，只证明"数值链路一致"，不证明"物理上合理"。
    // 这里按 strategy/rl_controller.cpp 的 64 维布局手工构造一个**真实站姿**观测
    // （不链接 rl_controller，保持本目标零依赖）：
    //   base_lin_vel=0 | ang_vel=0 | projected_gravity=[0,0,-1]（机身水平）
    //   joint_pos_rel=0（全部关节正好在 DEFAULT_POSE） | joint_vel=0
    //   last_action=0 | cmd=[0,0,0]（站立） | gait_phase(step=0)=[sin×4, cos×4]
    // 判据：动作全为有限值，且 |a| ≤ 1（不长期饱和，落到 action_clip 之后就无需削顶）。
    {
        rl::SetPolicyVariant(last_vid);
        const rl::PolicyWeights& w = rl::PolicyOf(last_vid);

        float obs[rl::ACTOR_DIM] = {0.0f};
        obs[8] = -1.0f;                      // projected_gravity z
        // gait_phase：step=0，offset=(0,0.5,0.5,0)，cycle=0.6s → sin 全 ~0、cos=[1,-1,-1,1]
        obs[56] = 0.0f; obs[57] = 0.0f; obs[58] = 0.0f; obs[59] = 0.0f;
        obs[60] = 1.0f; obs[61] = -1.0f; obs[62] = -1.0f; obs[63] = 1.0f;

        float act[rl::ACTION_DIM];
        rl::mlp_forward(obs, act);

        bool finite = true;
        float amax = 0.0f;
        int   amax_idx = -1;
        for (int i = 0; i < rl::ACTION_DIM; i++) {
            if (!std::isfinite(act[i])) finite = false;
            if (std::fabs(act[i]) > amax) { amax = std::fabs(act[i]); amax_idx = i; }
        }
        const bool bounded = amax <= 1.0f;
        printf("\n  nominal 站姿前向自检（变体 %d, %s）：\n", last_vid, w.name);
        printf("    obs = 单位四元数 + 全关节在 DEFAULT_POSE + 零速 + 零指令\n");
        for (int i = 0; i < rl::ACTION_DIM; i++)
            printf("    a[%2d] = %+.6f\n", i, act[i]);
        printf("    有限值 %s ；max|a| = %.6f（idx=%d）%s\n",
               finite ? "✅" : "❌", amax, amax_idx,
               bounded ? "≤1 未饱和 ✅" : ">1 会饱和 ❌");
        all_ok = all_ok && finite && bounded;
    }

    printf("\n%s\n", all_ok
        ? "✅ 全部通过：全部权重与网络结构正确，最新策略已正确接入。"
        : "❌ 有变体超差：权重/结构需用 tool/export_policy.py 重新导出。");
    return all_ok ? 0 : 1;
}
