/**
 * @file   main.cpp
 * @brief  CAN 电机控制框架的示例入口
 * @details 提供 5 个演示函数：
 *          1) 单电机基本控制      2) 同口多电机          3) 多 CAN 口协同
 *          4) 电机健康检查        5) 长时间运行演示
 *          运行方式：./motor_app <示例编号>   默认为 1
 */
#include <stdio.h>
#include <unistd.h>       // sleep()
#include <signal.h>       // signal() / SIGINT / SIGTERM
#include "motor_controller.h"

// 全局控制器指针：仅供信号处理函数访问，用于 Ctrl+C 时优雅停机
// 使用裸指针是为了在 SignalHandler 中以 C 语言方式访问；实际对象生命周期在 main 内
MotorController* g_controller = nullptr;

/**
 * @brief 信号处理函数：捕获 SIGINT / SIGTERM，触发优雅退出
 * @note  信号上下文里只做最少量的动作，避免调用不可重入函数导致未定义行为
 */
void SignalHandler(int sig) {
    printf("\n[INFO] Received signal %d, shutting down...\n", sig);
    if (g_controller) {
        g_controller->Stop();    // 触发 manager 停线程 + 关设备
    }
    exit(0);   // 直接退出进程，让 atexit/析构接管剩余清理
}

// ================= 示例 1：基本电机控制 =================
// 演示完整的"初始化 → 启动 → 控制 → 查询 → 停止"单电机流程
void Example1_BasicControl() {
    printf("\n========== Example 1: Basic Motor Control ==========\n");

    MotorController controller;

    if (!controller.Initialize()) {
        printf("[ERROR] Failed to initialize controller\n");
        return;
    }
    if (!controller.Start()) {
        printf("[ERROR] Failed to start controller\n");
        return;
    }

    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 发送一次移动命令：can0 上的 1 号电机，速度 1000，正向
    printf("\n[ACTION] Moving motor (can0, id=1) at speed 1000, forward\n");
    controller.MoveMotor(0, 1, 1000, 1);
    sleep(2);   // 等待电机运行 2 秒，同时后台线程会收集状态

    controller.PrintMotorStatus(0, 1);   // 打印最新状态

    printf("[ACTION] Stopping motor (can0, id=1)\n");
    controller.StopMotor(0, 1);
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 1 completed\n");
}

// ================= 示例 2：同一 CAN 口上的多电机协同 =================
// 验证 BroadcastCommand / 多电机并行命令的正确性
void Example2_MultiMotorControl() {
    printf("\n========== Example 2: Multi-Motor Control ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 给 can0 的 3 个电机分别下不同速度，制造差异以便观察
    printf("\n[ACTION] Starting all motors on can0\n");
    for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
        uint16_t speed = 500 + motor_id * 200;   // 700 / 900 / 1100
        controller.MoveMotor(0, motor_id, speed, 1);
        printf("  Motor %d: speed=%d\n", motor_id, speed);
    }

    sleep(3);
    controller.PrintAllMotorStatus();    // 一次性查看全部电机

    printf("\n[ACTION] Stopping all motors\n");
    controller.StopAllMotors();          // 广播停机
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 2 completed\n");
}

// ================= 示例 3：跨 CAN 口控制 =================
// 演示同一应用同时操作 4 个 CANET 设备（can0~can3）
void Example3_MultiCanPortControl() {
    printf("\n========== Example 3: Multi-CAN Port Control ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 每个 CAN 口上只启动 1 号电机，速度按口号递增以便辨识
    printf("\n[ACTION] Starting motors on different CAN ports\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        uint16_t speed = 800 + can_port * 100;    // 800/900/1000/1100
        controller.MoveMotor(can_port, 1, speed, 1);
        printf("  CAN%d Motor 1: speed=%d\n", can_port, speed);
    }

    sleep(3);
    controller.PrintAllMotorStatus();

    printf("\n[ACTION] Stopping all motors\n");
    controller.StopAllMotors();
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 3 completed\n");
}

// ================= 示例 4：健康检查与故障处理 =================
// 启动若干电机后轮询健康状态，不健康时调用 HandleMotorError
void Example4_HealthCheck() {
    printf("\n========== Example 4: Motor Health Check ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 只启动 can0 和 can1，共 6 个电机，减小演示数据量
    printf("\n[ACTION] Starting motors\n");
    for (uint8_t can_port = 0; can_port < 2; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            controller.MoveMotor(can_port, motor_id, 1000, 1);
        }
    }
    sleep(2);    // 等待若干状态回报

    // 对每台电机执行健康检查；不健康的走故障处理策略
    printf("\n[ACTION] Checking motor health\n");
    for (uint8_t can_port = 0; can_port < 2; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            bool healthy = controller.CheckMotorHealth(can_port, motor_id);
            if (!healthy) {
                MotorStatus status = controller.GetMotorStatus(can_port, motor_id);
                controller.HandleMotorError(can_port, motor_id, status.error_code);
            }
        }
    }

    printf("\n[ACTION] Stopping all motors\n");
    controller.StopAllMotors();
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 4 completed\n");
}

// ================= 示例 5：长时间运行演示 =================
// 启动全部 12 个电机，运行 10 秒，期间可作为吞吐/稳定性压力测试
void Example5_InteractiveControl() {
    printf("\n========== Example 5: Interactive Control ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());
    printf("[INFO] Running for 10 seconds with periodic status updates...\n\n");

    // 一次性启动全部 12 个电机（4 口 × 3 电机）
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            controller.MoveMotor(can_port, motor_id, 1000, 1);
        }
    }

    // 每秒打印一次心跳；实际项目可替换成更详细的状态打印或日志采集
    for (int i = 0; i < 10; i++) {
        sleep(1);
        printf("[%d] Motors running...\n", i + 1);
    }

    printf("\n[ACTION] Stopping all motors\n");
    controller.StopAllMotors();
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 5 completed\n");
}

// ================= 程序入口 =================
// 通过命令行参数选择示例编号；未指定则默认运行示例 1
int main(int argc, char* argv[]) {
    printf("========================================\n");
    printf("   CAN Motor Control Framework Demo\n");
    printf("========================================\n\n");

    // 注册 SIGINT / SIGTERM 处理，支持 Ctrl+C 优雅退出
    signal(SIGINT,  SignalHandler);
    signal(SIGTERM, SignalHandler);

    // 解析命令行参数（可选），默认运行示例 1
    int example = 1;
    if (argc > 1) {
        example = atoi(argv[1]);
    }

    printf("Usage: %s [example_number]\n", argv[0]);
    printf("  1 - Basic Motor Control\n");
    printf("  2 - Multi-Motor Control\n");
    printf("  3 - Multi-CAN Port Control\n");
    printf("  4 - Motor Health Check\n");
    printf("  5 - Interactive Control\n\n");

    printf("Running Example %d...\n", example);

    // 分发到对应示例；未知编号返回非 0，便于脚本判断
    switch (example) {
        case 1: Example1_BasicControl();        break;
        case 2: Example2_MultiMotorControl();   break;
        case 3: Example3_MultiCanPortControl(); break;
        case 4: Example4_HealthCheck();         break;
        case 5: Example5_InteractiveControl();  break;
        default:
            printf("[ERROR] Invalid example number: %d\n", example);
            return 1;
    }

    printf("\n========================================\n");
    printf("   Demo Completed\n");
    printf("========================================\n");

    return 0;
}
