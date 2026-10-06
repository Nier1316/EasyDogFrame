#pragma once
// ============================================================================
//  运行时策略变体选择
//
//  两套 actor 权重（smalllift_s45 / iteration_9754 与 standstep_s4 / iteration_10000）
//  同时编译进程序（见 src/strategy/policy_set.cpp），由本模块在运行时按 id 选择。
//
//  默认变体 = policy_variant.h 的 POLICY_VARIANT；示例可在进入 RL 循环前覆盖，例如：
//      rl::SetPolicyVariant(rl::POLICY_H52S45_V4_5350);   // Example37：稳定演示策略（可前进）
//      rl::SetPolicyVariant(rl::POLICY_STANDTURN_10000);  // Example61：站立/原地转向
//
//  ⚠ 只在进入 RL 循环**之前**调用一次；运行中切换会让动作不连续。
// ============================================================================

#include "strategy/policy_variant.h"   // 默认变体 POLICY_VARIANT

namespace rl {

enum PolicyVariantId : int {
    POLICY_SMALLLIFT_9754  = 0,   // iteration_9754：可前进 + 转向
    POLICY_STANDTURN_10000 = 1,   // iteration_10000：站立 / 原地转向（不支持前进）
    POLICY_H52S45_V4_5350  = 2,   // iteration_5350：h52_s45_v4，宽摩擦 DR / 地形适应（可前进）
                                  //   ↑ 当前**演示/稳定**策略，Example37 使用
};

/** 一套完整的策略权重 + 元数据（不拥有内存，指向编译进程序的静态数组） */
struct PolicyWeights {
    // actor 网络：64 -> 512 -> 256 -> 128 -> 16；W 形状 [in][out]（row-major）
    // 字段用小写：避开 <termios.h> 的 B0（波特率宏）等宏名冲突
    const float* w0; const float* b0;   // 64  -> 512
    const float* w1; const float* b1;   // 512 -> 256
    const float* w2; const float* b2;   // 256 -> 128
    const float* wo; const float* bo;   // 128 -> 16
    const float* ref_obs;               // [64] 离线回归输入
    const float* ref_action;            // [16] 离线回归期望输出
    const char*  name;                  // 显示名（如 smalllift_s45/iteration_9754）
    const char*  ckpt;                  // 训练侧 checkpoint 相对路径
    bool         has_forward_vx;        // 该策略是否支持 vx 前进
};

/** 取指定变体的权重表（id 越界时返回默认变体） */
const PolicyWeights& PolicyOf(int id);

/** 当前运行时变体 id / 切换 / 当前权重表 */
int  PolicyVariant();
void SetPolicyVariant(int id);
const PolicyWeights& CurrentPolicy();

} // namespace rl
