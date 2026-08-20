#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "RoboTasks/robot_app.h"
#include "example.h"

static RobotApp g_app;
static volatile bool g_running = true;

static void signal_handler(int) {
    g_running = false;
}

int main() {
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    // 运行示例9 - 基础电机控制
    // printf("[INFO] Running Example9_BasicMotorCtr...\n");
    // Example9_BasicMotorCtr();
    // printf("[INFO] Example9 completed.\n");

    // 运行示例10 - 多电机扭矩控制
    // printf("[INFO] Running Example10_MultiMotorTorqueControl...\n");
    // Example10_MultiMotorTorqueControl();
    // printf("[INFO] Example10 completed.\n");

    // 运行示例11 - 全电机扭矩控制（CAN0~3）
    // printf("[INFO] Running Example11_MoveAll...\n");
    // Example11_MoveAll();
    // printf("[INFO] Example11 completed.\n");

    // 运行示例12 - 正弦周期运动控制
    // printf("[INFO] Running Example12_SinusoidalMotion...\n");
    // Example12_SinusoidalMotion();
    // printf("[INFO] Example12 completed.\n");

    // 运行示例13 - 多电机正弦运动
    // printf("[INFO] Running Example13_MultiMotorSinusoidalMotion...\n");
    // Example13_MultiMotorSinusoidalMotion();
    // printf("[INFO] Example13 completed.\n");

    // 运行示例14 - 标定检测
    // printf("[INFO] Running Example14_CalibrationDetect...\n");
    // Example14_CalibrationDetect();
    // printf("[INFO] Example14 completed.\n");

    // 运行示例15 - 标定验证
    // printf("[INFO] Running Example15_CalibrationVerify...\n");
    // Example15_CalibrationVerify();
    // printf("[INFO] Example15 completed.\n");

    // 运行示例16 - 电机测试
    // printf("[INFO] Running Example16_MotorTest...\n");
    // Example16_MotorTest();
    // printf("[INFO] Example16 completed.\n");

    // 运行示例17 - SimSync 仿真集成
    // printf("[INFO] Running Example17_SimSyncIntegration...\n");
    // Example17_SimSyncIntegration();
    // printf("[INFO] Example17 completed.\n");

    // 运行示例18 - LEG IK Control
    // printf("[INFO] Running Example18_LegIKControl...\n");
    // Example18_LegIKControl();
    // printf("[INFO] Example18 completed.\n");

    // 运行示例19 - 读取当前姿态并缓慢移动到站立
    // printf("[INFO] Running Example19_ReadAndStand...\n");
    // Example19_ReadAndStand();
    // printf("[INFO] Example19 completed.\n");

    // 运行示例20 - 选择电机移动到物理零位
    // printf("[INFO] Running Example20_MoveToPhysicalZero...\n");
    // Example20_MoveToPhysicalZero();
    // printf("[INFO] Example20 completed.\n");


    // 运行示例21 - Xbox 手柄控制
    // printf("[INFO] Running Example21_XboxControllerControl...\n");
    // Example21_XboxControllerControl();
    // printf("[INFO] Example21 completed.\n");

    // 运行示例22 - 起立 + 轮子阻抗模式测试
    // printf("[INFO] Running Example22_StandAndWheelTest...\n");
    // Example22_StandAndWheelTest();
    // printf("[INFO] Example22 completed.\n");

    // 运行示例23 - 单路 CAN 键盘控制（选路 + 站立 + ↑↓高度 + ←→轮子）
    // printf("[INFO] Running Example23_SingleCanKeyboardControl...\n");
    // Example23_SingleCanKeyboardControl();
    // printf("[INFO] Example23 completed.\n");



    // 运行示例24 - 只读固件参数诊断（不使能电机，安全）
    // printf("[INFO] Running Example24_ReadMotorParams...\n");
    // Example24_ReadMotorParams();
    // printf("[INFO] Example24 completed.\n");

    // 运行示例25 - RL 策略控制（dogurdf sim2real 部署）
    // printf("[INFO] Running Example25_RLPolicyControl...\n");
    // Example25_RLPolicyControl();
    // printf("[INFO] Example25 completed.\n");

    // 运行示例26 - 键盘输入接收测试（纯诊断，不碰电机）
    // printf("[INFO] Running Example26_KeyboardInputTest...\n");
    // Example26_KeyboardInputTest();
    // printf("[INFO] Example26 completed.\n");

    // 运行示例27 - CANET 接收频率探针（纯读取，不使能电机）
    // printf("[INFO] Running Example27_CANetFrequencyProbe...\n");
    // Example27_CANetFrequencyProbe();
    // printf("[INFO] Example27 completed.\n");

    // 运行示例28 - CANET 批量发送探针（电机下电，不使能电机）
    // printf("[INFO] Running Example28_CANetBatchProbe...\n");
    // Example28_CANetBatchProbe();
    // printf("[INFO] Example28 completed.\n");

    // 运行示例29 - 控制环频率测试（拆锁后，电机可下电）
    // printf("[INFO] Running Example29_MainLoopCadenceTest...\n");
    // Example29_MainLoopCadenceTest();
    // printf("[INFO] Example29 completed.\n");

    // 运行示例30 - RL 策略链路离线验证（不碰 CAN）
    // printf("[INFO] Running Example30_RLPolicyLinkTest...\n");
    // Example30_RLPolicyLinkTest();
    // printf("[INFO] Example30 completed.\n");

    // 运行示例31 - RL 零位对齐（摆腿读角，不使能电机）
    // printf("[INFO] Running Example31_RLZeroAlign...\n");
    // Example31_RLZeroAlign();
    // printf("[INFO] Example31 completed.\n");

    // 运行示例32 - RL 默认姿态验证（命令到 DEFAULT_POSE，低增益）
    printf("[INFO] Running Example32_RLPoseCheck...\n");
    Example32_RLPoseCheck();
    printf("[INFO] Example32 completed.\n");
    return 0;
}


