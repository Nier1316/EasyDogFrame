/**
 * @file    mlp.h
 * @brief   手写 MLP 前向推理（dogurdf 策略 actor 网络）
 *
 * 网络结构（与 dogurdf_sim2sim_deploy/src/networks/mlp.py 的 ActorCriticMLP.act 一致）:
 *   64 -> 512 -> 256 -> 128 -> 16，中间层 ELU 激活，输出层无激活。
 *
 * 权重由 tool/export_policy.py 从 flax checkpoint 导出到 policy_weights.h，
 * 布局为 x @ W + b（W 形状 [in_dim][out_dim]，row-major）。
 *
 * 零外部依赖（不用 libtorch / onnxruntime），20 万参数、50 Hz 下 CPU 无压力。
 */
#pragma once

#include <cmath>
#include "strategy/policy_weights.h"

namespace rl {

constexpr int ACTOR_DIM   = 64;
constexpr int ACTION_DIM  = 16;
constexpr int HIDDEN_0    = 512;
constexpr int HIDDEN_1    = 256;
constexpr int HIDDEN_2    = 128;

/** ELU: x > 0 ? x : exp(x) - 1（expm1f 精度更好） */
inline float elu(float x) {
    return x > 0.0f ? x : std::expm1f(x);
}

/**
 * @brief 单层全连接 + 可选 ELU 激活
 * @param in      输入向量
 * @param W       权重，形状 [in_dim][out_dim]（row-major: W[i*out_dim + j]）
 * @param b       偏置 [out_dim]
 * @param in_dim  输入维度
 * @param out_dim 输出维度
 * @param out     输出向量 [out_dim]
 * @param act     是否施加 ELU
 */
inline void dense(const float* in, const float* W, const float* b,
                  int in_dim, int out_dim, float* out, bool act) {
    for (int j = 0; j < out_dim; ++j) {
        float acc = b[j];
        for (int i = 0; i < in_dim; ++i) {
            acc += in[i] * W[i * out_dim + j];
        }
        out[j] = act ? elu(acc) : acc;
    }
}

/**
 * @brief 策略前向：obs(64) -> action(16)，确定性输出（无噪声）
 */
inline void mlp_forward(const float* obs, float* action) {
    float h0[HIDDEN_0];
    dense(obs, ACTOR_W0, ACTOR_B0, ACTOR_DIM, HIDDEN_0, h0, true);

    float h1[HIDDEN_1];
    dense(h0, ACTOR_W1, ACTOR_B1, HIDDEN_0, HIDDEN_1, h1, true);

    float h2[HIDDEN_2];
    dense(h1, ACTOR_W2, ACTOR_B2, HIDDEN_1, HIDDEN_2, h2, true);

    dense(h2, ACTOR_WO, ACTOR_BO, HIDDEN_2, ACTION_DIM, action, false);
}

} // namespace rl
