#include "example.h"
#include "thread/thread_manager.h"
#include <thread>
#include <chrono>
#include <cmath>




// ================= 示例 1：原始 CAN 帧测试（仅依赖 BSP） =================(PASS)
void Example1_RawCanFrameTest() {
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

// ================= 辅助：ThreadState 转字符串 =================
static const char* state_str(ThreadState s) {
    switch (s) {
        case ThreadState::UNREGISTERED: return "UNREGISTERED";
        case ThreadState::REGISTERED:   return "REGISTERED";
        case ThreadState::RUNNING:      return "RUNNING";
        case ThreadState::STOPPED:      return "STOPPED";
        case ThreadState::ERROR:        return "ERROR";
        default:                        return "UNKNOWN";
    }
}

// ================= 示例 2：ONCE 模式测试 =================（PASS）
void Example2_OnceMode() {
    printf("\n========== Example 2: ONCE Mode Test ==========\n");
    ThreadManager mgr;

    mgr.register_thread("once_task", []() {
        printf("  [ONCE] 任务执行中...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        printf("  [ONCE] 任务完成\n");
    }, ThreadMode::ONCE);

    mgr.start_thread("once_task");
    printf("  启动后状态: %s\n", state_str(mgr.get_thread_state("once_task")));

    // 等待任务自然结束（任务耗时 100ms，等 300ms 足够）
    std::this_thread::sleep_for(std::chrono::milliseconds(105));
    printf("  等待后状态: %s（期望: STOPPED）\n",
           state_str(mgr.get_thread_state("once_task")));
}

// ================= 示例 3：LOOP 模式测试 =================(PASS)
void Example3_LoopMode() {
    printf("\n========== Example 3: LOOP Mode Test ==========\n");
    ThreadManager mgr;
    int loop_count = 0;

    mgr.register_thread("loop_task", [&loop_count]() {
        loop_count++;
        printf("  [LOOP] 第 %d 次执行\n", loop_count);
    }, ThreadMode::LOOP, 101);  // 每 100ms 执行一次

    mgr.start_thread("loop_task");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mgr.stop_thread("loop_task");

    printf("  停止后状态: %s（期望: STOPPED）\n",
           state_str(mgr.get_thread_state("loop_task")));
    printf("  共执行 %d 次（期望约 5 次）\n", loop_count);
}

// ================= 示例 4：共享数据区测试 =================
void Example4_SharedData() {
    printf("\n========== Example 4: SharedData Test ==========\n");
    ThreadManager mgr;
    auto& shared = mgr.get_shared_data();
    shared.set<float>("speed", 0.0f);

    // 写线程：每 200ms 将 speed 递增 1.0
    mgr.register_thread("writer", [&shared]() {
        float v = shared.get<float>("speed") + 1.0f;
        shared.set<float>("speed", v);
        printf("  [WRITER] speed = %.1f\n", v);
    }, ThreadMode::LOOP, 200);

    // 读线程：每 300ms 读取并打印 speed
    mgr.register_thread("reader", [&shared]() {
        printf("  [READER] speed = %.1f\n", shared.get<float>("speed"));
    }, ThreadMode::LOOP, 300);

    mgr.start_thread("writer");
    mgr.start_thread("reader");
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    mgr.stop_thread("writer");
    mgr.stop_thread("reader");

    printf("  最终 speed = %.1f（期望约 5.0）\n", shared.get<float>("speed"));
}

// ================= 示例 5：重复注册异常测试 =================
void Example5_DuplicateRegister() {
    printf("\n========== Example 5: Duplicate Register Test ==========\n");
    ThreadManager mgr;

    mgr.register_thread("task", []() {}, ThreadMode::ONCE);
    printf("  第一次注册: 成功\n");

    try {
        mgr.register_thread("task", []() {}, ThreadMode::ONCE);
        printf("  第二次注册: 未抛出异常（不符合预期）\n");
    } catch (const std::runtime_error& e) {
        printf("  第二次注册: 捕获异常 \"%s\"（期望: 抛出异常）\n", e.what());
    }
}

// ================= 示例 6：线程重启测试 =================
void Example6_ThreadRestart() {
    printf("\n========== Example 6: Thread Restart Test ==========\n");
    ThreadManager mgr;
    int run_count = 0;

    mgr.register_thread("task", [&run_count]() {
        run_count++;
        printf("  [ONCE] 第 %d 次运行\n", run_count);
    }, ThreadMode::ONCE);

    // 第一次启动，等待自然结束
    mgr.start_thread("task");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    printf("  第一次完成后状态: %s\n", state_str(mgr.get_thread_state("task")));

    // 从 STOPPED 重新启动
    mgr.start_thread("task");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    printf("  第二次完成后状态: %s\n", state_str(mgr.get_thread_state("task")));

    printf("  共运行 %d 次（期望: 2）\n", run_count);
}

// ================= 示例 7：析构自动清理测试 =================
void Example7_AutoCleanup() {
    printf("\n========== Example 7: Auto Cleanup Test ==========\n");
    std::atomic<int> cleanup_count{0};

    {
        ThreadManager mgr;
        mgr.register_thread("loop_task", [&cleanup_count]() {
            cleanup_count++;
        }, ThreadMode::LOOP, 100);

        mgr.start_thread("loop_task");
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        printf("  析构前执行次数: %d\n", cleanup_count.load());
        printf("  mgr 即将析构（不手动 stop）...\n");
    } // mgr 析构，自动 stop + join 所有线程

    // 能到这里说明析构成功 join 了线程，线程已停止
    printf("  析构后执行次数: %d（与析构前相同，线程已停止）\n", cleanup_count.load());
}

// ================= 示例 8：状态查询测试 =================
void Example8_StateQuery() {
    printf("\n========== Example 8: State Query Test ==========\n");
    ThreadManager mgr;

    // 未注册时
    printf("  未注册时: %s（期望: UNREGISTERED）\n",
           state_str(mgr.get_thread_state("task")));

    // 注册后
    mgr.register_thread("task", []() {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }, ThreadMode::ONCE);
    printf("  注册后:   %s（期望: REGISTERED）\n",
           state_str(mgr.get_thread_state("task")));

    // 启动后立刻查询（任务执行中）
    mgr.start_thread("task");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    printf("  运行中:   %s（期望: RUNNING）\n",
           state_str(mgr.get_thread_state("task")));

    // 等待任务完成
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    printf("  完成后:   %s（期望: STOPPED）\n",
           state_str(mgr.get_thread_state("task")));
}

void Example9_BasicMotorCtr(){
    // 初始化
    int canlabel = 1;
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize\n");
        return;
    }

    // 启动线程
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);  // 等待线程启动
    printf("[INFO] Threads started\n");
    fflush(stdout);

    // 使能电机 - CAN2, motor_id=1
    printf("[INFO] Enabling CAN1 motor_id=1...\n");
    motor_mgr.EnableMotor(canlabel, 1);
    sleep(1);  // 等待电机启动
    printf("[INFO] Motor enabled\n");
    fflush(stdout);

    // 发送阻抗控制命令 - CAN2, motor_id=1
    printf("[INFO] Sending impedance control command to CAN1 motor_id=1...\n");
    motor_mgr.SendImpedance(canlabel, 1, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
    printf("[INFO] Impedance control command sent\n");
    fflush(stdout);

    // 循环读取状态（修复：使用 usleep 替代 sleep）
    int output_count = 0;
    for (int i = 0; i < 10000; i++) {
        MotorStatus status = motor_mgr.GetStatus(canlabel, 1);

        // 每 100 次循环输出一次（减少输出频率，避免刷屏）
        if (i % 100 == 0) {
            printf("[%d] CAN%d-M1 - Pos: %.4f rad, Vel: %.4f rad/s, Torque: %.4f Nm, Error: 0x%02x, ACK: %d, Fault: %d, Enable: %d\n",
                   i, canlabel,status.position, status.velocity, status.torque, status.error_code,
                   status.ack, status.fault, status.enable);
            fflush(stdout);  // 立即刷新输出缓冲区
            output_count++;
        }

        usleep(10000);  // 10ms，使用 usleep 更精确
    }

    printf("[INFO] Loop completed. Total outputs: %d\n", output_count);
    fflush(stdout);

    // 禁用电机
    motor_mgr.DisableMotor(canlabel, 1);
    printf("[INFO] Motor disabled\n");
    fflush(stdout);

    // 关闭
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example9 completed\n");
    fflush(stdout);
}

/**
 * @brief 示例10：多电机扭矩控制
 * @details 控制 CAN0 和 CAN1 上的所有电机（共 6 个）输出 5Nm 扭矩
 *          每隔 100ms 打印每个电机的位置、速度、扭矩
 */
void Example10_MultiMotorTorqueControl() {
    printf("\n========== Example 10: Multi-Motor Torque Control ==========\n");
    printf("[INFO] Controlling 6 motors on CAN0 and CAN1 with 5Nm torque\n");
    printf("[INFO] Printing motor status every 100ms\n\n");

    // 初始化
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }

    // 启动线程
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);  // 等待线程启动
    printf("[INFO] Threads started\n");
    fflush(stdout);

    // 定义要控制的电机：CAN0 和 CAN1 上的所有电机
    struct MotorInfo {
        uint8_t can_port;
        uint8_t motor_id;
        const char* name;
    };

    MotorInfo motors[] = {
        {0, 1, "CAN0-M1"},
        {0, 2, "CAN0-M2"},
        {0, 3, "CAN0-M3"},
        {1, 1, "CAN1-M1"},
        {1, 2, "CAN1-M2"},
        {1, 3, "CAN1-M3"}
    };
    int motor_count = sizeof(motors) / sizeof(motors[0]);

    // 使能所有电机
    printf("[INFO] Enabling all motors...\n");
    for (int i = 0; i < motor_count; i++) {
        motor_mgr.EnableMotor(motors[i].can_port, motors[i].motor_id);
    }
    sleep(1);  // 等待电机启动
    printf("[INFO] All motors enabled\n");
    fflush(stdout);

    // 发送扭矩控制命令（5Nm）给所有电机
    printf("[INFO] Sending 5Nm torque command to all motors...\n");
    for (int i = 0; i < motor_count; i++) {
        // 使用阻抗控制模式，设置扭矩为 5Nm
        // 参数：位置=0, 速度=0, Kp=0, Kd=0, 扭矩=5Nm
        motor_mgr.SendImpedance(motors[i].can_port, motors[i].motor_id,
                                0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
    }
    printf("[INFO] Torque commands sent\n");
    fflush(stdout);

    // 循环读取状态（每隔 100ms 打印一次）
    printf("\n[INFO] Reading motor status every 100ms...\n");
    printf("%-12s | Pos(rad)  | Vel(rad/s) | Torque(Nm) | Error\n", "Motor");
    printf("%-12s-+-----------+------------+------------+-------\n", "");
    fflush(stdout);

    int loop_count = 0;
    int max_loops = 1000;  // 运行 1000 * 100ms = 100 秒

    for (int loop = 0; loop < max_loops; loop++) {
        // 每隔 100ms 打印一次
        printf("[%3d] ", loop);
        for (int i = 0; i < motor_count; i++) {
            MotorStatus status = motor_mgr.GetStatus(motors[i].can_port, motors[i].motor_id);
            printf("%-8s: P=%.3f V=%.3f T=%.3f E=0x%02x | ",
                   motors[i].name,
                   status.position, status.velocity, status.torque, status.error_code);
        }
        printf("\n");
        fflush(stdout);

        usleep(100000);  // 100ms
        loop_count++;
    }

    printf("\n[INFO] Status reading completed (%d loops)\n", loop_count);
    fflush(stdout);

    // 禁用所有电机
    printf("[INFO] Disabling all motors...\n");
    for (int i = 0; i < motor_count; i++) {
        motor_mgr.DisableMotor(motors[i].can_port, motors[i].motor_id);
    }
    printf("[INFO] All motors disabled\n");
    fflush(stdout);

    // 关闭
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example10 completed\n");
    fflush(stdout);
}

/**
 * @brief 示例11：全电机扭矩控制
 * @details 控制 CAN0~3 上的所有 12 个电机（共 4 路 CAN × 3 个电机）输出 5Nm 扭矩
 *          每隔 100ms 打印所有电机的位置、速度、扭矩
 */
void Example11_MoveAll() {
    printf("\n========== Example 11: All Motors Torque Control (CAN0~3) ==========\n");
    printf("[INFO] Controlling all 12 motors with 5Nm torque\n");
    printf("[INFO] Printing all motor status every 100ms\n\n");

    // 初始化
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }

    // 启动线程
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);
    printf("[INFO] Threads started\n");
    fflush(stdout);

    // 定义所有 12 个电机（CAN0~3，每路 3 个）
    struct MotorInfo {
        uint8_t can_port;
        uint8_t motor_id;
        const char* name;
    };

    MotorInfo motors[] = {
        {0, 1, "C0-M1"}, {0, 2, "C0-M2"}, {0, 3, "C0-M3"},
        {1, 1, "C1-M1"}, {1, 2, "C1-M2"}, {1, 3, "C1-M3"},
        {2, 1, "C2-M1"}, {2, 2, "C2-M2"}, {2, 3, "C2-M3"},
        {3, 1, "C3-M1"}, {3, 2, "C3-M2"}, {3, 3, "C3-M3"}
    };
    int motor_count = sizeof(motors) / sizeof(motors[0]);

    // 使能所有电机
    printf("[INFO] Enabling all %d motors...\n", motor_count);
    for (int i = 0; i < motor_count; i++) {
        motor_mgr.EnableMotor(motors[i].can_port, motors[i].motor_id);
    }
    sleep(1);
    printf("[INFO] All motors enabled\n");
    fflush(stdout);

    // 发送扭矩控制命令给所有电机（5Nm）
    printf("[INFO] Sending 5Nm torque command to all motors...\n");
    for (int i = 0; i < motor_count; i++) {
        motor_mgr.SendImpedance(motors[i].can_port, motors[i].motor_id,0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
    }
    printf("[INFO] Torque commands sent\n");
    fflush(stdout);

    // 循环读取并打印所有电机状态（每 100ms 一次）
    printf("\n[INFO] Reading all motor status every 100ms...\n");
    printf("Loop | ");
    for (int i = 0; i < motor_count; i++) {
        printf("%-8s ", motors[i].name);
    }
    printf("\n     | ");
    for (int i = 0; i < motor_count; i++) {
        printf("P/V/T    ");
    }
    printf("\n");
    printf("-----|");
    for (int i = 0; i < motor_count; i++) {
        printf("----------");
    }
    printf("\n");
    fflush(stdout);

    // 100 次循环，每次 100ms，共 10 秒
    int max_loops = 1000;
    for (int loop = 0; loop < max_loops; loop++) {
        printf("[%3d] | ", loop);

        for (int i = 0; i < motor_count; i++) {
            MotorStatus status = motor_mgr.GetStatus(motors[i].can_port, motors[i].motor_id);
            printf("%.2f/%.2f/%.2f ",
                   status.position, status.velocity, status.torque);
        }
        printf("\n");
        fflush(stdout);

        usleep(100000);  // 100ms
    }

    printf("\n[INFO] Status reading completed (%d loops, 10 seconds total)\n", max_loops);
    fflush(stdout);

    // 禁用所有电机
    printf("[INFO] Disabling all motors...\n");
    for (int i = 0; i < motor_count; i++) {
        motor_mgr.DisableMotor(motors[i].can_port, motors[i].motor_id);
    }
    printf("[INFO] All motors disabled\n");
    fflush(stdout);

    // 清理
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example11 completed\n");
    fflush(stdout);
}

/**
 * @brief 示例12：正弦周期运动控制
 * @details 控制 CAN1 的 1 号电机以正弦波周期运动
 *          - 目标角度在 0 ~ -1 rad 之间变化
 *          - 周期 T = 2 秒
 *          - Kp = 200, Kd = 15
 *          - 运动时间 20 秒（10 个完整周期）
 */
void Example12_SinusoidalMotion() {
    printf("\n========== Example 12: Sinusoidal Motion Control ==========\n");
    printf("[INFO] CAN1 Motor-1 sinusoidal motion control\n");
    printf("[INFO] Target angle: 0 ~ -1 rad\n");
    printf("[INFO] Period: 2 seconds\n");
    printf("[INFO] Kp=200, Kd=15\n");
    printf("[INFO] Duration: 20 seconds (10 cycles)\n\n");

    // 初始化
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }

    // 启动线程
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);
    printf("[INFO] Threads started\n");
    fflush(stdout);

    // 使能电机
    printf("[INFO] Enabling CAN1 motor_id=1...\n");
    motor_mgr.EnableMotor(1, 1);
    sleep(1);
    printf("[INFO] Motor enabled\n");
    fflush(stdout);

    // 控制参数
    const uint8_t can_port = 1;
    const uint8_t motor_id = 1;
    const float Kp = 200.0f;
    const float Kd = 15.0f;
    const float period = 2.0f;  // 2 秒周期
    const float amplitude = 0.5f;  // 幅度 0.5，范围 0 ~ -1
    const int total_loops = 200;  // 200 * 100ms = 20 秒（10 个完整周期）

    printf("\n[INFO] Starting sinusoidal motion...\n");
    printf("Time(s) | Pos(rad) | Vel(rad/s) | Torque(Nm) | Actual_Pos | Actual_Vel | Actual_Torque\n");
    printf("--------|----------|------------|------------|------------|------------|-------------\n");
    fflush(stdout);


    for (int loop = 0; loop < total_loops; loop++) {
        // 计算当前时间（秒）
        float elapsed_time = loop * 0.1f;  // 每次循环 100ms = 0.1s

        // 计算目标位置（正弦波）
        // pos = -0.5 * (1 - cos(π * t / period))
        // 这使得位置在 0 到 -1 之间变化，周期为 2 秒
        float target_pos = -amplitude * (1.0f - cosf(M_PI * elapsed_time / period));

        // 计算目标速度（正弦波的导数）
        // v = -0.5 * π/period * sin(π * t / period)
        float target_vel = -amplitude * (M_PI / period) * sinf(M_PI * elapsed_time / period);

        // 发送阻抗控制命令
        motor_mgr.SendImpedance(can_port, motor_id,
                                target_pos, target_vel, Kp, Kd, 0.0f);

        // 读取实际状态
        MotorStatus status = motor_mgr.GetStatus(can_port, motor_id);

        // 打印状态（每 5 个循环打印一次）
        if (loop % 5 == 0) {
            printf("%7.2f | %8.4f | %10.4f | %10.4f | %10.4f | %10.4f | %13.4f\n",
                   elapsed_time, target_pos, target_vel, 0.0f,
                   status.position, status.velocity, status.torque);
            fflush(stdout);
        }

        usleep(100000);  // 100ms
    }

    printf("\n[INFO] Sinusoidal motion completed (20 seconds, 10 cycles)\n");
    fflush(stdout);

    // 禁用电机
    printf("[INFO] Disabling motor...\n");
    motor_mgr.DisableMotor(can_port, motor_id);
    printf("[INFO] Motor disabled\n");
    fflush(stdout);

    // 清理
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example12 completed\n");
    fflush(stdout);
}
