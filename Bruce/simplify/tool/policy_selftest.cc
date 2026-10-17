// ============================================================================
//  tool/policy_selftest.cc —— 策略权重离线数值回归（**不碰 CAN / 不开 USB**）
//
//  为什么单独做一个可执行文件：验证"新策略是否正确部署进框架"只需要 MLP 前向，
//  没必要、也不应该启动真机程序（bin/can_motor_app 会去打开 USB2CAN 设备）。
//  本目标只链接 policy_set.cpp（纯权重数据 + 变体表），因此可以在任何没有硬件的
//  时候跑，也适合放进 CI。
//
//  原理：tool/export_policy.py 导出权重时会同时写一份 REF_OBS(64)/REF_ACTION(16)
//  （来自 Python 侧 flax 推理的真实输入/输出）。这里对**每一套**变体跑
//  mlp_forward(REF_OBS)，与 REF_ACTION 逐元素比较，最大绝对误差应 < 1e-3。
//
//  用法：cmake --build build -j && ./bin/policy_selftest
//  退出码：0 = 全部通过；1 = 有变体超差。
// ============================================================================
#include <cmath>
#include <cstdio>

#include "strategy/mlp.h"
#include "strategy/policy_set.h"

int main() {
    printf("=== 策略权重离线数值回归（不碰 CAN）===\n");
    printf("    mlp 结构 %d -> %d -> %d -> %d -> %d，ELU 隐藏层，输出无激活\n\n",
           rl::ACTOR_DIM, rl::HIDDEN_0, rl::HIDDEN_1, rl::HIDDEN_2, rl::ACTION_DIM);

    bool all_ok = true;
    for (int vid = rl::POLICY_SMALLLIFT_9754; vid <= rl::POLICY_RW9_PLAIN_V11_S4_8001; vid++) {
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

    // 额外：新策略(8001) 的 action_clip 路径也要能跑（截断到 [-1,1]）
    {
        rl::SetPolicyVariant(rl::POLICY_RW9_PLAIN_V11_S4_8001);
        const rl::PolicyWeights& w = rl::PolicyOf(rl::POLICY_RW9_PLAIN_V11_S4_8001);
        float act[rl::ACTION_DIM];
        rl::mlp_forward_clipped(w.ref_obs, act);
        bool in_range = true;
        for (int i = 0; i < rl::ACTION_DIM; i++)
            if (act[i] < -1.0f || act[i] > 1.0f) in_range = false;
        printf("\n  mlp_forward_clipped(变体 3)：全部动作 ∈ [-1,1] %s\n",
               in_range ? "✅" : "❌");
        all_ok = all_ok && in_range;
    }

    printf("\n%s\n", all_ok
        ? "✅ 全部通过：四套权重与网络结构正确，新策略 8001 已正确接入。"
        : "❌ 有变体超差：权重/结构需用 tool/export_policy.py 重新导出。");
    return all_ok ? 0 : 1;
}
