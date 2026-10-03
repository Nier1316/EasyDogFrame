#pragma once
// ============================================================================
//  策略变体选择 —— **全工程唯一的权重开关**
//
//  改下面 POLICY_VARIANT 一个数字即可切换编译进固件的策略；两套权重都留在仓库里，
//  随时切回，不需要重新导出。
//
//    0 = smalllift_s45 / iteration_9754（2026-09-19）
//        可前进/后退（vx≠0）+ 转向；Example37 手柄遥操作、Example36 站立、
//        Example53 重力前馈测量等都基于它。
//    1 = standstep_s4 / iteration_10000（2026-10-02，当前最新）
//        只有两种行为：**静止站立 [0,0,0]** 与 **原地迈步转向 [0,0,wz]**（vx 恒 0）。
//        专为"站立↔原地转向"训练（diagonal light-lift stepping + 轮差速偏航，平地）。
//        喂 vx≠0 属于训练分布之外，**不要**用它做前进。
//
//  ⚠ 切换变体后必须重新编译，并跑一次 **Example30_OfflineLinkCheck**（离线 MLP 数值回归，
//    不碰 CAN）确认导出权重与推理链路正确。
//  导出方法（训练环境，一条命令写完，注意不要用行尾反斜杠续行——会触发 -Wcomment）：
//    /home/sysu/miniconda3/envs/MJX/bin/python tool/export_policy.py --ckpt <...>/iteration_10000.pkl --out-weights include/strategy/policy_weights_standturn.h --out-ref include/strategy/policy_test_ref_standturn.h
// ============================================================================

#define POLICY_VARIANT 1

#if POLICY_VARIANT == 1
  #include "strategy/policy_weights_standturn.h"
  #include "strategy/policy_test_ref_standturn.h"
  #define POLICY_VARIANT_NAME     "standstep_s4/iteration_10000"
  #define POLICY_VARIANT_CKPT     "checkpoints_20261002_110719_standstep_s4/iteration_10000.pkl"
  #define POLICY_HAS_FORWARD_VX   0   // 该策略不支持 vx 前进（只有站立/原地转向）
#else
  #include "strategy/policy_weights.h"
  #include "strategy/policy_test_ref.h"
  #define POLICY_VARIANT_NAME     "smalllift_s45/iteration_9754"
  #define POLICY_VARIANT_CKPT     "checkpoints_20260919_115617_smalllift_s45/iteration_9754.pkl"
  #define POLICY_HAS_FORWARD_VX   1
#endif
