#pragma once
// RL/前馈/延迟辨识示例：25, 30~32, 35~38（策略·链路回归·零位·姿态·轮前馈·站立·遥操作·延迟）
//                      51~53, 56（站立后趴下·固定 yaw·重力前馈·固定 yaw 遥测录制）
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
// 固定 yaw 转向真机遥测录制（静置→yaw=0.5 转5s→停，每步落盘 s2r_fixedyaw_*.csv，sim2real 对比）
void Example56_FixedYawRecord();
// 【2026-10-02 新增】站立 / 原地转向 专精策略（standstep_s4）手柄遥操作：
// 右摇杆=原地转向（死区内=精确站立 [0,0,0]）、A=强制站立、B=急停、START=趴下、q=退出；
// 默认开 500Hz 统一数据集录制，用于与 sim2sim 对比 gap。要求 POLICY_VARIANT==1。
void Example61_RLStandTurnTeleop();
