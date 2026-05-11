/**
 * @file   main.cpp
 * @brief  CAN 电机控制框架的示例入口
 * @details 提供 6 个演示函数：
 *          1) 单电机基本控制（三种模式）  2) 同口多电机          3) 多 CAN 口协同
 *          4) 电机健康检查                5) 长时间运行演示      6) 原始 CAN 帧测试（仅依赖 BSP）
 *          运行方式：./motor_app <示例编号>   默认为 1
 */
#include <stdio.h>
#include <unistd.h>       // sleep()
#include <signal.h>       // signal() / SIGINT / SIGTERM
#include "motor_controller.h"
#include "bsp/bsp_can.h"   // 示例 6 直接使用 BSP 层

// 全局控制器指针：仅供信号处理函数访问，用于 Ctrl+C 时优雅停机
MotorController* g_controller = nullptr;

/**
 * @brief 信号处理函数：捕获 SIGINT / SIGTERM，触发优雅退出
 */
void SignalHandler(int sig) {
    printf("\n[INFO] Received signal %d, shutting down...\n", sig);
    if (g_controller) {
        g_controller->Stop();
    }
    exit(0);
}

// ================= 示例 1：基本电机控制（三种模式） =================
void Example1_BasicControl() {
    printf("\n========== Example 1: Basic Motor Control (Three Modes) ==========\n");

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

    // 使能电机
    printf("\n[ACTION] Enabling motor (can0, id=1)\n");
    controller.EnableMotor(0, 1);
    sleep(1);

    // 模式 1：阻抗控制
    printf("\n[ACTION] Impedance control: pos=0, vel=0, kp=10, kd=1, torque=0\n");
    controller.ImpedanceControl(0, 1, 0, 0, 10, 1, 0);
    sleep(2);

    // 模式 2：速度控制
    printf("\n[ACTION] Speed control: vel=5 rad/s, kp=10, ki=0.1\n");
    controller.SpeedControl(0, 1, 5.0f, 10.0f, 0.1f);
    sleep(2);

    // 模式 3：位置控制
    printf("\n[ACTION] Position control: pos=1 rad, kvp=5, kp=10, kd=1, kvi=0.1\n");
    controller.PositionControl(0, 1, 1.0f, 5.0f, 10.0f, 1.0f, 0.1f);
    sleep(2);

    controller.PrintMotorStatus(0, 1);

    printf("\n[ACTION] Disabling motor (can0, id=1)\n");
    controller.DisableMotor(0, 1);
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 1 completed\n");
}

// ================= 示例 2：同一 CAN 口上的多电机协同 =================
void Example2_MultiMotorControl() {
    printf("\n========== Example 2: Multi-Motor Control ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 使能 can0 的 3 个电机
    printf("\n[ACTION] Enabling all motors on can0\n");
    for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
        controller.EnableMotor(0, motor_id);
    }
    sleep(1);

    // 给 3 个电机分别下不同的速度控制命令
    printf("\n[ACTION] Starting all motors on can0 with different speeds\n");
    for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
        float vel = 2.0f + motor_id * 1.0f;  // 3.0 / 4.0 / 5.0 rad/s
        controller.SpeedControl(0, motor_id, vel, 10.0f, 0.1f);
        printf("  Motor %d: vel=%.1f rad/s\n", motor_id, vel);
    }

    sleep(3);
    controller.PrintAllMotorStatus();

    printf("\n[ACTION] Disabling all motors on can0\n");
    for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
        controller.DisableMotor(0, motor_id);
    }
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 2 completed\n");
}

// ================= 示例 3：跨 CAN 口控制 =================
void Example3_MultiCanPortControl() {
    printf("\n========== Example 3: Multi-CAN Port Control ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 在 4 个 CAN 口上各启动 1 个电机
    printf("\n[ACTION] Starting motors on all CAN ports\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        controller.EnableMotor(can_port, 1);
        float vel = 2.0f + can_port * 1.0f;
        controller.SpeedControl(can_port, 1, vel, 10.0f, 0.1f);
        printf("  CAN%d Motor 1: vel=%.1f rad/s\n", can_port, vel);
    }

    sleep(3);
    controller.PrintAllMotorStatus();

    printf("\n[ACTION] Disabling all motors\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        controller.DisableMotor(can_port, 1);
    }
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 3 completed\n");
}

// ================= 示例 4：电机健康检查 =================
void Example4_HealthCheck() {
    printf("\n========== Example 4: Motor Health Check ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 启动所有电机
    printf("\n[ACTION] Enabling all motors\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            controller.EnableMotor(can_port, motor_id);
            controller.SpeedControl(can_port, motor_id, 3.0f, 10.0f, 0.1f);
        }
    }

    sleep(2);

    // 检查所有电机的健康状态
    printf("\n[ACTION] Checking health status of all motors\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            if (controller.CheckMotorHealth(can_port, motor_id)) {
                printf("  CAN%d Motor %d: HEALTHY\n", can_port, motor_id);
            } else {
                printf("  CAN%d Motor %d: UNHEALTHY\n", can_port, motor_id);
                MotorStatus status = controller.GetMotorStatus(can_port, motor_id);
                controller.HandleMotorError(can_port, motor_id, status.error_code);
            }
        }
    }

    printf("\n[ACTION] Disabling all motors\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            controller.DisableMotor(can_port, motor_id);
        }
    }
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 4 completed\n");
}

// ================= 示例 5：长时间运行（全 12 电机） =================
void Example5_LongRunning() {
    printf("\n========== Example 5: Long Running (All 12 Motors) ==========\n");

    MotorController controller;
    if (!controller.Initialize() || !controller.Start()) {
        printf("[ERROR] Failed to initialize/start controller\n");
        return;
    }
    printf("[INFO] System started. Total motors: %d\n", controller.GetMotorCount());

    // 启动全部 12 个电机
    printf("\n[ACTION] Enabling all 12 motors\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            controller.EnableMotor(can_port, motor_id);
            float vel = 2.0f + (can_port * 3 + motor_id) * 0.5f;
            controller.SpeedControl(can_port, motor_id, vel, 10.0f, 0.1f);
        }
    }

    printf("[INFO] All motors running. Monitoring for 10 seconds...\n");
    for (int i = 0; i < 10; i++) {
        sleep(1);
        if (i % 5 == 0) {
            printf("[INFO] Status check at %d seconds\n", i);
            controller.PrintAllMotorStatus();
        }
    }

    printf("\n[ACTION] Disabling all motors\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            controller.DisableMotor(can_port, motor_id);
        }
    }
    sleep(1);

    controller.Stop();
    printf("[INFO] Example 5 completed\n");
}

// ================= 示例 6：原始 CAN 帧测试（仅依赖 BSP） =================
void Example6_RawCanFrameTest() {
    printf("\n========== Example 6: Raw CAN Frame Test (BSP Only) ==========\n");
    printf("[INFO] This example only depends on BSP layer, no motor needed\n");

    BspCan& bsp = BspCan::GetInstance();

    // 初始化 can0
    CanDeviceConfig config;
    config.device_idx = 0;
    config.port = 4001;
    config.server_ip = "192.168.0.178";
    config.work_mode = TCP_CLIENT;

    if (!bsp.InitDevice(0, config)) {
        printf("[ERROR] Failed to initialize device 0\n");
        return;
    }

    if (!bsp.StartDevice(0)) {
        printf("[ERROR] Failed to start device 0\n");
        return;
    }

    printf("[INFO] Device 0 initialized and started\n");

    // 发送测试帧
    printf("\n[ACTION] Sending test frames\n");
    for (int i = 0; i < 3; i++) {
        BspCanFrame frame;
        frame.id = 0x001 + i;
        frame.dlc = 8;
        frame.is_extended = 0;
        for (int j = 0; j < 8; j++) {
            frame.data[j] = i * 8 + j;
        }

        printf("[ACTION] Sending frame %d: ID=0x%03x, data=[", i + 1, frame.id);
        for (int j = 0; j < 8; j++) {
            printf("%02x ", frame.data[j]);
        }
        printf("]\n");

        if (!bsp.SendFrame(0, frame)) {
            printf("[ERROR] Failed to send frame\n");
        }

        sleep(1);
    }

    bsp.StopDevice(0);
    printf("[INFO] Example 6 completed\n");
}

// ================= 主函数 =================
int main(int argc, char* argv[]) {
    int example = 1;
    if (argc > 1) {
        example = atoi(argv[1]);
    }

    // 注册信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    printf("========================================\n");
    printf("   CAN Motor Control Framework Demo\n");
    printf("========================================\n\n");

    printf("Usage: %s [example_number]\n", argv[0]);
    printf("  1 - Basic Motor Control (Three Modes)\n");
    printf("  2 - Multi-Motor Control\n");
    printf("  3 - Multi-CAN Port Control\n");
    printf("  4 - Motor Health Check\n");
    printf("  5 - Long Running (All 12 Motors)\n");
    printf("  6 - Raw CAN Frame Test (BSP Only)\n\n");

    printf("Running Example %d...\n", example);

    switch (example) {
        case 1:
            Example1_BasicControl();
            break;
        case 2:
            Example2_MultiMotorControl();
            break;
        case 3:
            Example3_MultiCanPortControl();
            break;
        case 4:
            Example4_HealthCheck();
            break;
        case 5:
            Example5_LongRunning();
            break;
        case 6:
            Example6_RawCanFrameTest();
            break;
        default:
            printf("[ERROR] Unknown example number: %d\n", example);
            return -1;
    }

    printf("\n========================================\n");
    printf("   Demo Completed\n");
    printf("========================================\n");

    return 0;
}
