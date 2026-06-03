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

    // 运行示例11 - 全电机扭矩控制（CAN0~3）
    // printf("[INFO] Running Example11_MoveAll...\n");
    // Example11_MoveAll();
    // printf("[INFO] Example11 completed.\n");

    // 运行示例12 - 正弦周期运动控制
    // printf("[INFO] Running Example12_SinusoidalMotion...\n");
    // Example12_SinusoidalMotion();
    // printf("[INFO] Example12 completed.\n");

    // Example10_MultiMotorTorqueControl();
    Example9_BasicMotorCtr();

    return 0;
}


