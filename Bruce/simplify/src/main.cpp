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
    printf("[INFO] Running Example21_XboxControllerControl...\n");
    Example21_XboxControllerControl();
    printf("[INFO] Example21 completed.\n");

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

    return 0;
}


