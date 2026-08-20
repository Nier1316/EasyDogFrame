#include <stdio.h>
#include <unistd.h>       // sleep()
#include <signal.h>       // signal() / SIGINT / SIGTERM
#include <cmath>
#include "bsp/bsp_can.h"   // 示例 1 直接使用 BSP 层
#include "motor_manager.h"
#include "motor_calibration.h"
#include "thread/thread_manager.h"


void Example1_RawCanFrameTest();
void Example2_OnceMode();
void Example3_LoopMode();
void Example4_SharedData();
void Example5_DuplicateRegister();
void Example6_ThreadRestart();
void Example7_AutoCleanup();
void Example8_StateQuery();
void Example9_BasicMotorCtr();
void Example10_MultiMotorTorqueControl();
void Example11_MoveAll();
void Example12_SinusoidalMotion();
void Example13_MultiMotorSinusoidalMotion();
void Example14_CalibrationDetect();
void Example15_CalibrationVerify();
void Example16_MotorTest();
void Example17_SimSyncIntegration();
void Example18_LegIKControl();
void Example19_ReadAndStand();
void Example20_MoveToPhysicalZero();
void Example21_XboxControllerControl();
void Example22_StandAndWheelTest();
void Example23_SingleCanKeyboardControl();
// 只读诊断：不使能任何电机，读固件量程/模式/上电初始反馈
void Example24_ReadMotorParams();
// RL 策略控制（dogurdf sim2real 部署，50 Hz）
void Example25_RLPolicyControl();
// 键盘输入接收测试（纯诊断，不碰电机）：验证集成终端/调试器下的输入通路
void Example26_KeyboardInputTest();
// CANET 接收频率探针（不使能电机）：纯读取测设备往返延迟与吞吐
void Example27_CANetFrequencyProbe();
// CANET 批量发送探针（电机下电，不使能电机）：量化批量 vs 逐帧发送耗时
void Example28_CANetBatchProbe();
// 控制环频率测试（拆锁后）：使能 CAN1 电机（可下电）测主循环帧间隔
void Example29_MainLoopCadenceTest();
// RL 策略链路离线验证（不碰 CAN）：REF_OBS→mlp→REF_ACTION 回归门
void Example30_RLPolicyLinkTest();
// RL 零位对齐（摆腿读角，不使能电机）：测仿真默认姿态对应的真机指令角
void Example31_RLZeroAlign();
// RL 默认姿态验证（命令到 DEFAULT_POSE，低增益）：验证 GetStatus↔URDF 转换
void Example32_RLPoseCheck();
