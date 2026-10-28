#pragma once
// ============================================================================
//  策略变体 —— **编译期默认值**（运行时切换见 strategy/policy_set.h）
//
//  多套权重现在**同时编译进程序**，本文件只决定“程序启动时的默认变体”。
//  任何示例都能在进入 RL 循环前调用 rl::SetPolicyVariant() 覆盖它：
//
//    0 = smalllift_s45 / iteration_9754（2026-09-19）
//        可前进/后退（vx≠0）+ 转向；Example37 手柄遥操作、Example36 站立、
//        Example53 重力前馈测量等都基于它。
//    1 = standstep_s4 / iteration_10000（2026-10-02）
//        只有两种行为：**静止站立 [0,0,0]** 与 **原地迈步转向 [0,0,wz]**（vx 恒 0）。
//        专为"站立↔原地转向"训练（diagonal light-lift stepping + 轮差速偏航，平地）。
//        喂 vx≠0 属于训练分布之外，**不要**用它做前进。
//    2 = h52_s45_v4 / iteration_5350（2026-09-17）【演示/稳定】
//        宽摩擦 DR、地形适应，可前进 + 转向；Example37 手柄遥操作使用。
//        该权重于 2026-09-29 清理时删除、2026-10-06 从 git 历史恢复（逐字节一致）。
//    3 = rw9_plain_v1.1_s4 / iteration_8001（2026-10-15）【上一版平地速度】
//        plain checkpoint（不含 HIM 估计器），obs 64 / action 16。仅训练过平地，
//        **不得用于楼梯/崎岖地面**。推理必须用动作均值（deterministic），不采样。
//    4 = rw9_plain_v1.2_s4_hipy25 / iteration_9002（2026-10-18）【默认 · 最新平地速度】
//        plain checkpoint（不含 HIM 估计器），obs 64 / action 16。仅训练过平地，
//        **不得用于楼梯/崎岖地面**。推理必须用动作均值（deterministic），不采样。
//    5 = dogurdf_legacy_s4 / iteration_1500（2026-10-27）【旧模型线 · stage-4 + RAW 观测】
//        checkpoint: checkpoints_20261027_195149_repro_s4_legacy/iteration_1500.pkl。
//        plain actor（不含 HIM），obs 64 / action 16；**观测通道是 RAW**
//        （ang_vel / joint_vel 不乘 0.25 / 0.05，= 2026-10-22 之前的训练约定），
//        与本框架 build_observation 的原始通道一致 —— 反过来用缩放通道喂它，
//        实测原地转会从 ~92% 打成 266%~281%（超转狂转）。
//        nominal 姿态与 PD 增益和框架一致（0/−0.03/−0.116、Kp/Kd 250/4、0.25、12.5）。
//        仅平地 + stage-4 参数 DR 训练，**不得用于楼梯/崎岖地面**。推理用动作均值。
//
//    6 = dogurdf_pace_stg4 / iteration_6503（2026-10-28）【默认 · 新模型线 · stage-4 + 缩放观测】
//        checkpoint: checkpoints_20261028_071627_stg4_pace/iteration_6503.pkl（PACE 分阶段链
//        stage 1→4 的终点，链由 scripts/train_stage_chain.sh 启动）。
//        plain actor（不含 HIM），obs 64 / action 16；**观测通道是缩放**：
//        ang_vel × 0.25、joint_vel（12 腿 + 4 轮共 16 维一起）× 0.05 —— 即 2026-10-22
//        之后的 Table-I 约定（velocity/observations.py:129-130 缺省、:203/:206 施加，
//        velocity/__init__.py:255-260 注入，dogurdf_config.py 未覆盖）。框架用
//        PolicyWeights.obs_ang_vel_scale / obs_joint_vel_scale **逐变体**实现该通道，
//        变体 0~5 的系数仍为 1.0（RAW），数值逐位不变。
//        nominal 与 PD 同框架（0/−0.03/−0.116、Kp/Kd 250/4、0.25、12.5、clip 1.0）。
//        训练带 PACE 辨识的 6.25 ms 力矩延迟与关节阻尼；仅平地 + stage-4 DR，
//        **不得用于楼梯/崎岖地面**。推理必须用动作均值（deterministic，std≈0.369）。
//
//    7 = dogurdf_pace_s45 / iteration_6800（2026-10-28）【默认 · 新模型线 · 6 的续训】
//        checkpoint: checkpoints_20261028_162903_pace_s45/iteration_6800.pkl。
//        来历：**直接从变体 6 的 6503 续训**（run 名 pace_s45，16:29 启动，iteration
//        6504→6800；训练侧日志 logs/pace_s45_20261028_162859.log 首段
//        "Resuming from .../stg4_pace/iteration_6503.pkl → 6504"，末段停在 6800）。
//        plain actor（不含 HIM），obs 64 / action 16 / critic 88；**观测通道是缩放**
//        （ang_vel × 0.25、joint_vel 12 腿 + 4 轮共 16 维一起 × 0.05），与变体 6 相同。
//        依据：该 run 没有启动脚本（scripts/ 下无 pace_s45），日志里无 --obs_* 覆盖；
//        train.py:244-261 的 --obs_* 缺省 None → velocity/__init__.py:255-260 从 config
//        注入 → dogurdf_config.py 未设 obs_ang_vel_scale / obs_joint_vel_scale（只设了
//        旧的 config.obs_scales.*=1.0，那是另一套字段）→ 落到 observations.py:129-130
//        的缺省 0.25 / 0.05。旁证：actor 首层行 |W| 均值 6503→6800 几乎不动
//        （ang_vel 5.2251→5.2297 ×1.001、腿 joint_vel 8.3547→8.3070 ×0.994、
//        轮速 4.0730→3.8064 ×0.935），而 legacy RAW→6503 是 ×2.64 / ×8.49 / ×10.16
//        —— 297 iter 不可能把同一通道的量级搬走，故 6800 与 6503 同通道。
//        **与变体 6 的区别 = 训练地形**：7 是 stage 4.5（stage-4 参数 DR + 整图连续
//        Perlin 崎岖地，hfield all_rough、rough_range 0.075；日志里有 hfield 加载行），
//        6 是平地（stage 4）。其余（网络/PD/动作缩放/通道）逐项相同。
//        nominal 与 PD 同框架（0/−0.03/−0.116、Kp/Kd 250/4、0.25、12.5、clip 1.0）。
//        **台阶/楼梯仍未训练**。推理必须用动作均值（deterministic，std≈0.365）。
//
//  ⚠ 无论选哪套，都建议跑一次 **Example30_RLPolicyLinkTest**（离线、不碰 CAN）：
//    它会**同时校验全部八套权重**的 MLP 数值回归，确认导出权重与推理链路正确。
//  导出方法（训练环境，一条命令写完，注意不要用行尾反斜杠续行——会触发 -Wcomment）：
//    <python> tool/export_policy.py --ckpt <...>/iteration_10000.pkl --out-weights include/strategy/policy_weights_standturn.h --out-ref include/strategy/policy_test_ref_standturn.h
// ============================================================================

#define POLICY_VARIANT 7   // 默认变体（0=9754 / 1=10000 / 2=5350 / 3=8001 / 4=9002 / 5=dogurdf_legacy_it1500 / 6=dogurdf_pace_stg4_it6503 / 7=dogurdf_pace_s45_it6800；示例可用 rl::SetPolicyVariant 覆盖）
