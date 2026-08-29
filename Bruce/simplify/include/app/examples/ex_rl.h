#pragma once
// RL/前馈/延迟辨识示例 25, 30~32, 35~38
void Example25_RLPolicyControl();
void Example30_RLPolicyLinkTest();
void Example31_RLZeroAlign();
void Example32_RLPoseCheck();
void Example35_WheelFFCalibrate();
void Example36_RLStandLoop();
void Example37_RLTeleopControl();
void Example38_ActionDelayMeasure();
// 起立 → RL 站立循环 → 回车缓慢趴下（完整流程）
void Example51_StandRLThenLieDown();
// 固定转向命令 RL（yaw=0.5，5s）——sim2real 对比真机侧数据采集
void Example52_FixedCmdYaw();
// RL 站立下重力前馈测量（读稳态 cal_torque，标定 JOINT_IMPEDANCE.tau_ff）
void Example53_MeasureGravityFF();
