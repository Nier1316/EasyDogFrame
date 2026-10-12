#pragma once
// ============================================================================
//  策略变体 —— **编译期默认值**（运行时切换见 strategy/policy_set.h）
//
//  两套权重现在**同时编译进程序**，本文件只决定“程序启动时的默认变体”。
//  任何示例都能在进入 RL 循环前调用 rl::SetPolicyVariant() 覆盖它：
//
//    0 = smalllift_s45 / iteration_9754（2026-09-19）
//        可前进/后退（vx≠0）+ 转向；Example37 手柄遥操作、Example36 站立、
//        Example53 重力前馈测量等都基于它。
//    1 = standstep_s4 / iteration_10000（2026-10-02）
//        只有两种行为：**静止站立 [0,0,0]** 与 **原地迈步转向 [0,0,wz]**（vx 恒 0）。
//        专为"站立↔原地转向"训练（diagonal light-lift stepping + 轮差速偏航，平地）。
//        喂 vx≠0 属于训练分布之外，**不要**用它做前进。
//    2 = h52_s45_v4 / iteration_5350（2026-09-17）【默认 · 演示/稳定】
//        宽摩擦 DR、地形适应，可前进 + 转向；Example37 手柄遥操作使用。
//        该权重于 2026-09-29 清理时删除、2026-10-06 从 git 历史恢复（逐字节一致）。
//
//  ⚠ 无论选哪套，都建议跑一次 **Example30_RLPolicyLinkTest**（离线、不碰 CAN）：
//    它会**同时校验三套权重**的 MLP 数值回归，确认导出权重与推理链路正确。
//  导出方法（训练环境，一条命令写完，注意不要用行尾反斜杠续行——会触发 -Wcomment）：
//    <python> tool/export_policy.py --ckpt <...>/iteration_10000.pkl --out-weights include/strategy/policy_weights_standturn.h --out-ref include/strategy/policy_test_ref_standturn.h
// ============================================================================

#define POLICY_VARIANT 2   // 默认变体（0=9754 / 1=10000 / 2=5350；示例可用 rl::SetPolicyVariant 覆盖）
