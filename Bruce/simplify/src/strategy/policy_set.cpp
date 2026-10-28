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
//    wt_dogurdf_legacy_s4_it1500: weights/dogurdf_legacy_s4_it1500.pkl
//                                 （repro_s4_legacy / iteration_1500，stage-4 + **RAW 观测**）
//    wt_dogurdf_pace_stg4_it6503: weights/dogurdf_pace_stg4_it6503.pkl
//                                 （pace stg4 / iteration_6503，stage-4 + **缩放观测 0.25/0.05**）
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

namespace rl { namespace wt_dogurdf_legacy_s4_it1500 {
#include "strategy/policy_weights_dogurdf_legacy_s4_it1500.h"
#include "strategy/policy_test_ref_dogurdf_legacy_s4_it1500.h"
} }  // namespace rl::wt_dogurdf_legacy_s4_it1500

namespace rl { namespace wt_dogurdf_pace_stg4_it6503 {
#include "strategy/policy_weights_dogurdf_pace_stg4_it6503.h"
#include "strategy/policy_test_ref_dogurdf_pace_stg4_it6503.h"
} }  // namespace rl::wt_dogurdf_pace_stg4_it6503

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
    1.0f, 1.0f,   // 观测通道 = RAW（训练时 ang_vel/joint_vel 未缩放）
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
    1.0f, 1.0f,   // 观测通道 = RAW（训练时 ang_vel/joint_vel 未缩放）
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
    1.0f, 1.0f,   // 观测通道 = RAW（训练时 ang_vel/joint_vel 未缩放）
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
    1.0f, 1.0f,   // 观测通道 = RAW（训练时 ang_vel/joint_vel 未缩放）
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
    1.0f, 1.0f,   // 观测通道 = RAW（训练时 ang_vel/joint_vel 未缩放）
};

// dogurdf_legacy_s4 / iteration_1500：2026-10-27 的 repro_s4_legacy 重现 run（stage 4 DR）。
// plain actor（不含 HIM 估计器），obs 64 / action 16，训练时 base_lin_vel 恒 0。
// ⚠ 观测约定 = **RAW**（ang_vel / joint_vel 通道不乘 0.25 / 0.05）——该 run 以
//   --obs_ang_vel_scale 1.0 --obs_joint_vel_scale 1.0 训练，即 2026-10-22 之前的通道约定；
//   本框架的 build_observation 一直是 RAW，两者一致（用缩放通道会把原地转打成 200%+）。
// 仅在平地/stage-4 DR 下训练，不得用于楼梯/崎岖地面。推理用动作均值（deterministic）。
// 训练侧 nominal 姿态与 PD 增益和框架完全一致：hip/thigh/calf = 0/−0.03/−0.116、
// Kp/Kd = 250/4、action_scale 0.25、wheel_vel_scale 12.5。
static const PolicyWeights kDogurdfLegacyS4It1500 = {
    wt_dogurdf_legacy_s4_it1500::ACTOR_W0, wt_dogurdf_legacy_s4_it1500::ACTOR_B0,
    wt_dogurdf_legacy_s4_it1500::ACTOR_W1, wt_dogurdf_legacy_s4_it1500::ACTOR_B1,
    wt_dogurdf_legacy_s4_it1500::ACTOR_W2, wt_dogurdf_legacy_s4_it1500::ACTOR_B2,
    wt_dogurdf_legacy_s4_it1500::ACTOR_WO, wt_dogurdf_legacy_s4_it1500::ACTOR_BO,
    wt_dogurdf_legacy_s4_it1500::REF_OBS,  wt_dogurdf_legacy_s4_it1500::REF_ACTION,
    "dogurdf_legacy_s4/iteration_1500",
    "checkpoints_20261027_195149_repro_s4_legacy/iteration_1500.pkl",
    true, false,
    1.0f, 1.0f,   // 观测通道 = RAW（训练时 ang_vel/joint_vel 未缩放）
};

// dogurdf_pace_stg4 / iteration_6503：2026-10-28 的 PACE 分阶段链（stage 1→4）终点。
// plain actor（不含 HIM 估计器），obs 64 / action 16，训练时 obs[0:3] 恒零。
// ⚠ 观测通道 = **缩放**（ang_vel ×0.25、joint_vel ×0.05，joint_vel 是 12 腿 + 4 轮
//   共 16 维一起乘）—— 即 2026-10-22 之后的 Table-I 约定。依据：
//     · src/components/tasks/velocity/observations.py:129-130（缺省 0.25 / 0.05）
//       与 :203 `his_ang_vel * ang_vel_scale`、:206 `his_dof_vel * joint_vel_scale`；
//     · src/components/tasks/velocity/__init__.py:255-260 从 config 注入（缺省同上）；
//     · src/cfg/dogurdf_config.py 未设置 obs_ang_vel_scale / obs_joint_vel_scale；
//     · 该 run 由 scripts/train_stage_chain.sh:72-74 启动，未传 --obs_* 覆盖。
//   反向佐证：该链从 RAW 的 legacy it1500 续训，切到 pace_s1_warm it1500 时 actor
//   首层 ang_vel 输入行 |W| ×2.43、joint_vel 行 ×8.29，其余行持平 —— 正是输入被
//   0.25/0.05 缩放后的补偿方向（理想 ×4 / ×20，1500 iter 仅部分重拟合）。
// 训练边界：平地 + stage-4 参数 DR，带 PACE 辨识的 6.25 ms 力矩延迟与关节阻尼；
// 不得用于楼梯/崎岖地面。推理必须用动作均值（deterministic，std≈0.369）。
// 训练侧 nominal 姿态与 PD 增益和框架完全一致：hip/thigh/calf = 0/−0.03/−0.116、
// Kp/Kd = 250/4、action_scale 0.25、wheel_vel_scale 12.5、action_clip 1.0。
static const PolicyWeights kDogurdfPaceStg4It6503 = {
    wt_dogurdf_pace_stg4_it6503::ACTOR_W0, wt_dogurdf_pace_stg4_it6503::ACTOR_B0,
    wt_dogurdf_pace_stg4_it6503::ACTOR_W1, wt_dogurdf_pace_stg4_it6503::ACTOR_B1,
    wt_dogurdf_pace_stg4_it6503::ACTOR_W2, wt_dogurdf_pace_stg4_it6503::ACTOR_B2,
    wt_dogurdf_pace_stg4_it6503::ACTOR_WO, wt_dogurdf_pace_stg4_it6503::ACTOR_BO,
    wt_dogurdf_pace_stg4_it6503::REF_OBS,  wt_dogurdf_pace_stg4_it6503::REF_ACTION,
    "dogurdf_pace_stg4/iteration_6503",
    "checkpoints_20261028_071627_stg4_pace/iteration_6503.pkl",
    true, false,
    0.25f, 0.05f,   // 缩放观测通道（2026-10-22 之后的训练约定）
};

const PolicyWeights& PolicyOf(int id) {
    switch (id) {
        case POLICY_SMALLLIFT_9754:        return kSmalllift;
        case POLICY_STANDTURN_10000:       return kStandturn;
        case POLICY_H52S45_V4_5350:        return k5350;
        case POLICY_RW9_PLAIN_V11_S4_8001:        return kRw9PlainV11S4;
        case POLICY_RW9_PLAIN_V12_S4_HIPY25_9002: return kRw9PlainV12S4Hipy25;
        case POLICY_DOGURDF_LEGACY_S4_1500:       return kDogurdfLegacyS4It1500;
        case POLICY_DOGURDF_PACE_STG4_6503:       return kDogurdfPaceStg4It6503;
        default:                                  return PolicyOf(POLICY_VARIANT);   // 越界 → 编译期默认
    }
}

static int g_variant = POLICY_VARIANT;   // 默认 = 编译期设置（policy_variant.h）

int PolicyVariant() { return g_variant; }

void SetPolicyVariant(int id) {
    if (id == POLICY_SMALLLIFT_9754 || id == POLICY_STANDTURN_10000 ||
        id == POLICY_H52S45_V4_5350 || id == POLICY_RW9_PLAIN_V11_S4_8001 ||
        id == POLICY_RW9_PLAIN_V12_S4_HIPY25_9002 ||
        id == POLICY_DOGURDF_LEGACY_S4_1500 ||
        id == POLICY_DOGURDF_PACE_STG4_6503)
        g_variant = id;
}

const PolicyWeights& CurrentPolicy() { return PolicyOf(g_variant); }

} // namespace rl
