// ============================================================================
//  运行时策略变体：多套 actor 权重同时编译进程序
//
//  各权重头文件的符号名完全相同（ACTOR_W0/…/ACTOR_BO 与 REF_OBS/REF_ACTION），
//  因此必须各自包进独立命名空间；`#pragma once` 按“文件”去重，不同文件
//  都能正常展开，不会互相顶掉。
//
//  权重来源（由 tool/export_policy.py 从 weights/*.pkl 导出，勿手改权重头）：
//    wt_smalllift       : weights/iteration_9754.pkl   （smalllift_s45 stage4.5，可前进）
//    wt_standturn       : iteration_10000.pkl          （standstep_s4，站立/原地转向）
//    wt_5350            : weights/iteration_5350.pkl   （h52_s45_v4，演示稳定）
//    wt_rw9_plain_v11_s4: weights/iteration_8001.pkl（rw9_plain_v1.1_s4，上一版平地速度）
//    wt_rw9_plain_v12_s4_hipy25: weights/iteration_9002.pkl（rw9_plain_v1.2_s4_hipy25，最新平地速度）
// ============================================================================
#include "strategy/policy_set.h"

namespace rl { namespace wt_smalllift {
#include "strategy/policy_weights.h"
#include "strategy/policy_test_ref.h"
} }  // namespace rl::wt_smalllift

namespace rl { namespace wt_standturn {
#include "strategy/policy_weights_standturn.h"
#include "strategy/policy_test_ref_standturn.h"
} }  // namespace rl::wt_standturn

namespace rl { namespace wt_5350 {
#include "strategy/policy_weights_5350.h"
#include "strategy/policy_test_ref_5350.h"
} }  // namespace rl::wt_5350

namespace rl { namespace wt_rw9_plain_v11_s4 {
#include "strategy/policy_weights_rw9_plain_v11_s4.h"
#include "strategy/policy_test_ref_rw9_plain_v11_s4.h"
} }  // namespace rl::wt_rw9_plain_v11_s4

namespace rl { namespace wt_rw9_plain_v12_s4_hipy25 {
#include "strategy/policy_weights_rw9_plain_v12_s4_hipy25.h"
#include "strategy/policy_test_ref_rw9_plain_v12_s4_hipy25.h"
} }  // namespace rl::wt_rw9_plain_v12_s4_hipy25

namespace rl {

static const PolicyWeights kSmalllift = {
    wt_smalllift::ACTOR_W0, wt_smalllift::ACTOR_B0,
    wt_smalllift::ACTOR_W1, wt_smalllift::ACTOR_B1,
    wt_smalllift::ACTOR_W2, wt_smalllift::ACTOR_B2,
    wt_smalllift::ACTOR_WO, wt_smalllift::ACTOR_BO,
    wt_smalllift::REF_OBS,  wt_smalllift::REF_ACTION,
    "smalllift_s45/iteration_9754",
    "checkpoints_20260919_115617_smalllift_s45/iteration_9754.pkl",
    true, false,
};

// iteration_5350 = h52_s45_v4（宽摩擦 DR / 地形适应）—— 当前演示/稳定策略。
// checkpoint 2026-09-29 清理时删除，2026-10-06 由 git 历史 0d0257c 恢复（blob 逐字节一致）。
static const PolicyWeights k5350 = {
    wt_5350::ACTOR_W0, wt_5350::ACTOR_B0,
    wt_5350::ACTOR_W1, wt_5350::ACTOR_B1,
    wt_5350::ACTOR_W2, wt_5350::ACTOR_B2,
    wt_5350::ACTOR_WO, wt_5350::ACTOR_BO,
    wt_5350::REF_OBS,  wt_5350::REF_ACTION,
    "h52_s45_v4/iteration_5350",
    "checkpoints_20260917_193400_h52_s45_v4/iteration_5350.pkl",
    true, true,
};

static const PolicyWeights kStandturn = {
    wt_standturn::ACTOR_W0, wt_standturn::ACTOR_B0,
    wt_standturn::ACTOR_W1, wt_standturn::ACTOR_B1,
    wt_standturn::ACTOR_W2, wt_standturn::ACTOR_B2,
    wt_standturn::ACTOR_WO, wt_standturn::ACTOR_BO,
    wt_standturn::REF_OBS,  wt_standturn::REF_ACTION,
    "standstep_s4/iteration_10000",
    "checkpoints_20261002_110719_standstep_s4/iteration_10000.pkl",
    false, false,
};

// rw9_plain_v1.1_s4 / iteration_8001：上一版平地速度策略（plain，不含 HIM 估计器）。
// obs 64 / action 16，训练时 base_lin_vel 恒 0；仅训练过平地，不得用于楼梯/崎岖地面。
// 推理必须用动作均值（deterministic，不采样；std 仅约 0.36）。
static const PolicyWeights kRw9PlainV11S4 = {
    wt_rw9_plain_v11_s4::ACTOR_W0, wt_rw9_plain_v11_s4::ACTOR_B0,
    wt_rw9_plain_v11_s4::ACTOR_W1, wt_rw9_plain_v11_s4::ACTOR_B1,
    wt_rw9_plain_v11_s4::ACTOR_W2, wt_rw9_plain_v11_s4::ACTOR_B2,
    wt_rw9_plain_v11_s4::ACTOR_WO, wt_rw9_plain_v11_s4::ACTOR_BO,
    wt_rw9_plain_v11_s4::REF_OBS,  wt_rw9_plain_v11_s4::REF_ACTION,
    "rw9_plain_v1.1_s4/iteration_8001",
    "checkpoints_20261015_181212_rw9_plain_v1.1_s4/iteration_8001.pkl",
    true, false,
};

// rw9_plain_v1.2_s4_hipy25 / iteration_9002：最新平地速度策略（plain，不含 HIM 估计器）。
// obs 64 / action 16，训练时 base_lin_vel 恒 0；仅训练过平地，不得用于楼梯/崎岖地面。
// 推理必须用动作均值（deterministic，不采样；std 仅约 0.35）。
static const PolicyWeights kRw9PlainV12S4Hipy25 = {
    wt_rw9_plain_v12_s4_hipy25::ACTOR_W0, wt_rw9_plain_v12_s4_hipy25::ACTOR_B0,
    wt_rw9_plain_v12_s4_hipy25::ACTOR_W1, wt_rw9_plain_v12_s4_hipy25::ACTOR_B1,
    wt_rw9_plain_v12_s4_hipy25::ACTOR_W2, wt_rw9_plain_v12_s4_hipy25::ACTOR_B2,
    wt_rw9_plain_v12_s4_hipy25::ACTOR_WO, wt_rw9_plain_v12_s4_hipy25::ACTOR_BO,
    wt_rw9_plain_v12_s4_hipy25::REF_OBS,  wt_rw9_plain_v12_s4_hipy25::REF_ACTION,
    "rw9_plain_v1.2_s4_hipy25/iteration_9002",
    "checkpoints_20261018_104932_rw9_plain_v1.2_s4_hipy25/iteration_9002.pkl",
    true, false,
};

const PolicyWeights& PolicyOf(int id) {
    switch (id) {
        case POLICY_SMALLLIFT_9754:        return kSmalllift;
        case POLICY_STANDTURN_10000:       return kStandturn;
        case POLICY_H52S45_V4_5350:        return k5350;
        case POLICY_RW9_PLAIN_V11_S4_8001:        return kRw9PlainV11S4;
        case POLICY_RW9_PLAIN_V12_S4_HIPY25_9002: return kRw9PlainV12S4Hipy25;
        default:                                  return PolicyOf(POLICY_VARIANT);   // 越界 → 编译期默认
    }
}

static int g_variant = POLICY_VARIANT;   // 默认 = 编译期设置（policy_variant.h）

int PolicyVariant() { return g_variant; }

void SetPolicyVariant(int id) {
    if (id == POLICY_SMALLLIFT_9754 || id == POLICY_STANDTURN_10000 ||
        id == POLICY_H52S45_V4_5350 || id == POLICY_RW9_PLAIN_V11_S4_8001 ||
        id == POLICY_RW9_PLAIN_V12_S4_HIPY25_9002)
        g_variant = id;
}

const PolicyWeights& CurrentPolicy() { return PolicyOf(g_variant); }

} // namespace rl
