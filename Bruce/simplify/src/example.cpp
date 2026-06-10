#include "example.h"
#include "thread/thread_manager.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <cstring>




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
    // 控制参数
    const uint8_t can_port = 0;
    const uint8_t motor_id = 1;
    const float Kp = 200.0f;
    const float Kd = 20.0f;
    const float period = 2.0f;  // 2 秒周期
    const float amplitude = 0.5f;  // 幅度 0.5，范围 0 ~ -1
    const int total_loops = 200;  // 200 * 100ms = 20 秒（10 个完整周期）


    // 启动线程
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);
    printf("[INFO] Threads started\n");
    fflush(stdout);

    // 使能电机
    printf("[INFO] Enabling CAN1 motor_id=1...\n");
    motor_mgr.EnableMotor(can_port, motor_id);
    sleep(1);
    printf("[INFO] Motor enabled\n");
    fflush(stdout);



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

/**
 * @brief 示例13：多电机同向正弦运动
 * @details 4 条腿的 Hip 关节同时同向正弦运动
 *          标定正确时，4 条腿应肉眼可见摆向同一侧
 *          周期 T=2s, Kp=200, Kd=15, 运动 20 秒
 */
void Example13_MultiMotorSinusoidalMotion() {
    printf("\n========== Example 13: Multi-Motor Sinusoidal Motion ==========\n");
    printf("[INFO] 4 Hip motors moving in the SAME direction\n");
    printf("[INFO] Period: 2 seconds, Kp=200, Kd=15\n");
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

    // 4 条腿的 1 号电机（Hip），标定自适应方向
    struct MotorInfo {
        uint8_t can_port;
        uint8_t motor_id;
        const char* name;
    };

    MotorInfo motors[] = {
        {0, 1, "CAN0-M1"},
        {1, 1, "CAN1-M1"},
        {2, 1, "CAN2-M1"},
        {3, 1, "CAN3-M1"}
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

    // 控制参数
    const float Kp = 200.0f;
    const float Kd = 15.0f;
    const float period = 2.0f;
    const float amplitude = 0.5f;
    const int total_loops = 200;  // 20 秒

    printf("\n[INFO] Starting sinusoidal motion...\n");
    printf("Time(s) | ");
    for (int i = 0; i < motor_count; i++) {
        printf("%-8s ", motors[i].name);
    }
    printf("\n--------|");
    for (int i = 0; i < motor_count; i++) {
        printf("----------");
    }
    printf("\n");
    fflush(stdout);

    for (int loop = 0; loop < total_loops; loop++) {
        float elapsed_time = loop * 0.1f;

        printf("%7.2f | ", elapsed_time);

        // 所有电机统一 target_pos 向负方向摆动 (0 ~ -1 rad)
        float target_pos = -amplitude * (1.0f - cosf(M_PI * elapsed_time / period));
        float target_vel = -amplitude * (M_PI / period) * sinf(M_PI * elapsed_time / period);

        for (int i = 0; i < motor_count; i++) {
            motor_mgr.SendImpedance(motors[i].can_port, motors[i].motor_id,
                                    target_pos, target_vel, Kp, Kd, 0.0f);
            MotorStatus status = motor_mgr.GetStatus(motors[i].can_port, motors[i].motor_id);
            printf("%.2f/%.2f ", status.position, status.velocity);
        }
        printf("\n");
        fflush(stdout);

        usleep(100000);  // 100ms
    }

    printf("\n[INFO] Sinusoidal motion completed (20 seconds, 10 cycles)\n");
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

    printf("[INFO] Example13 completed\n");
    fflush(stdout);
}

// ================= 示例 14：标定检测 =================
/**
 * @brief 逐个测试每个电机的实际运动方向，生成建议的标定矩阵
 *
 * 原理：
 *   1. 向电机发送 +0.5 rad 位置指令
 *   2. 读取实际反馈位置
 *   3. 比较指令方向与反馈方向 → 确定 pos_scale
 *   4. 速度方向同理 → 确定 vel_scale
 *
 * 说明：
 *   - 可运行多次，矩阵值会被自动识别
 *   - 每次只测试一个电机（逐个运动，更安全）
 *   - 建议先从 CAN0-M1 开始确认
 */
void Example14_CalibrationDetect() {
    printf("\n========== Example 14: Calibration Detection ==========\n");
    printf("[INFO] This test will move each motor individually.\n");
    printf("[INFO] Make sure the robot is suspended or in a safe position.\n");
    printf("[INFO] Press Ctrl+C within 3 seconds to abort...\n\n");

    // 倒计时
    for (int i = 3; i > 0; i--) {
        printf("  %d...\n", i);
        fflush(stdout);
        sleep(1);
    }

    // 当前标定矩阵快照
    MotorCalibrationParam current_cal[4][3];
    memcpy(current_cal, MOTOR_CALIBRATION, sizeof(MOTOR_CALIBRATION));
    MotorCalibrationParam suggested[4][3];

    // 初始化
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 测试参数
    const float test_amplitude = 0.3f;  // 测试幅度（弧度）
    const float Kp = 100.0f;
    const float Kd = 10.0f;
    const int samples_per_dir = 5;
    const int sample_interval_ms = 80;

    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        for (uint8_t motor_id = 1; motor_id <= 3; motor_id++) {
            printf("\n--- Testing CAN%d Motor %d ---\n", can_port, motor_id);

            // 使能电机
            motor_mgr.EnableMotor(can_port, motor_id);
            sleep(1);

            // === 位置方向测试 ===
            // 指令 +pos 方向
            motor_mgr.SendImpedance(can_port, motor_id,
                                    test_amplitude, 0.0f, Kp, Kd, 0.0f);
            usleep(500 * 1000);  // 等待到达位置

            float avg_pos_plus = 0.0f;
            for (int s = 0; s < samples_per_dir; s++) {
                MotorStatus st = motor_mgr.GetStatus(can_port, motor_id);
                avg_pos_plus += st.position;
                usleep(sample_interval_ms * 1000);
            }
            avg_pos_plus /= samples_per_dir;

            // 指令 -pos 方向
            motor_mgr.SendImpedance(can_port, motor_id,
                                    -test_amplitude, 0.0f, Kp, Kd, 0.0f);
            usleep(500 * 1000);

            float avg_pos_minus = 0.0f;
            for (int s = 0; s < samples_per_dir; s++) {
                MotorStatus st = motor_mgr.GetStatus(can_port, motor_id);
                avg_pos_minus += st.position;
                usleep(sample_interval_ms * 1000);
            }
            avg_pos_minus /= samples_per_dir;

            // 判断位置方向：指令 + 应得到正反馈，指令 - 应得到负反馈
            float pos_dir = avg_pos_plus - avg_pos_minus;  // 应约等于 2 * amplitude
            bool pos_correct = (pos_dir > 0);  // 指令+:反馈正 → 方向正确

            // 建议标定值：方向正确 → 保持；方向反了 → 取反
            float new_pos_scale = pos_correct ?
                current_cal[can_port][motor_id - 1].pos_scale :
                -current_cal[can_port][motor_id - 1].pos_scale;
            // vel_scale 保持原值，不测试速度方向
            float new_vel_scale = current_cal[can_port][motor_id - 1].vel_scale;

            suggested[can_port][motor_id - 1] = {new_pos_scale, new_vel_scale, 0.0f};

            // 打印结果
            printf("  Pos: cmd=+%.1f → avg=%.4f | cmd=-%.1f → avg=%.4f | dir=%+.4f -> %s\n",
                   test_amplitude, avg_pos_plus, test_amplitude, avg_pos_minus,
                   pos_dir, pos_correct ? "OK" : "REVERSED");
            printf("  -> pos_scale: %.1f, vel_scale: %.1f (kept)\n",
                   new_pos_scale, new_vel_scale);

            // 回退到阻抗模式归零，然后禁用
            motor_mgr.SendImpedance(can_port, motor_id, 0.0f, 0.0f, 100.0f, 10.0f, 0.0f);
            usleep(300 * 1000);
            motor_mgr.DisableMotor(can_port, motor_id);
        }
    }

    // === 打印建议的完整标定矩阵 ===
    printf("\n\n========== Suggested Calibration Matrix ==========\n");
    printf("static const MotorCalibrationParam MOTOR_CALIBRATION[4][3] = {\n");
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        const char* legs[] = {"左前", "右前", "左后", "右后"};
        printf("    // CAN%d 端口 (%s腿)\n    {\n", can_port, legs[can_port]);
        for (uint8_t motor_id = 0; motor_id < 3; motor_id++) {
            const char* joints[] = {"Hip", "Thigh", "Calf"};
            MotorCalibrationParam& cal = suggested[can_port][motor_id];
            printf("        {% .1ff, % .1ff, 0.0f},      // Motor %d (%s)\n",
                   cal.pos_scale, cal.vel_scale, motor_id + 1, joints[motor_id]);
        }
        printf("    },\n");
    }
    printf("};\n\n");

    // 清理
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example14 completed. ");
    printf("Copy the matrix above into motor_calibration.h and recompile.\n");
    fflush(stdout);
}

// ================= 示例 15：标定验证 =================
/**
 * @brief 直观验证标定结果：4 个同一关节同时同向运动
 *
 * 操作方式：
 *   终端输入关节号：1=Hip, 2=Thigh, 3=Calf
 *   4 条腿的该关节同时做正弦运动
 *
 * 验证方法：
 *   - 如果标定正确 → 4 条腿的关节肉眼可见的摆向同一侧
 *   - 如果某条腿摆反了 → 该电机 pos_scale 需要取反
 */
void Example15_CalibrationVerify() {
    printf("\n========== Example 15: Calibration Verification ==========\n");
    printf("[INFO] This test moves the SAME joint on ALL 4 legs together.\n");
    printf("[INFO] If calibration is correct, all 4 legs should swing in the SAME direction.\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    const float Kp = 200.0f;
    const float Kd = 15.0f;
    const float amplitude = 0.5f;
    const float period = 2.0f;

    printf("Available joints:\n");
    printf("  1 = Hip   2 = Thigh   3 = Calf\n");
    printf("Enter joint number: ");

    int joint = 0;
    if (scanf("%d", &joint) != 1 || joint < 1 || joint > 3) {
        printf("[ERROR] Invalid joint number. Exiting.\n");
        thread_mgr.stop_thread("motor_receive");
        thread_mgr.stop_thread("motor_send");
        motor_mgr.Stop();
        return;
    }

    printf("\n[INFO] Moving all 4 %s joints simultaneously\n",
           joint == 1 ? "Hip" : joint == 2 ? "Thigh" : "Calf");
    printf("[INFO] Watch: all 4 should swing the SAME direction\n\n");

    // 使能全部 4 条腿的同一个关节
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        motor_mgr.EnableMotor(can_port, (uint8_t)joint);
    }
    sleep(1);

    // 正弦运动 10 秒
    const int total_loops = 100;  // 100ms * 100 = 10s
    for (int loop = 0; loop < total_loops; loop++) {
        float elapsed = loop * 0.1f;
        float target_pos = amplitude * (1.0f - cosf(M_PI * elapsed / period));

        printf("  t=%.1fs pos=%.3f | ", elapsed, target_pos);

        for (uint8_t can_port = 0; can_port < 4; can_port++) {
            motor_mgr.SendImpedance(can_port, (uint8_t)joint,
                                    target_pos, 0.0f, Kp, Kd, 0.0f);
            MotorStatus st = motor_mgr.GetStatus(can_port, (uint8_t)joint);
            printf("C%d:%.2f ", can_port, st.position);
        }
        printf("\n");
        fflush(stdout);

        usleep(100000);
    }

    // 归零
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        motor_mgr.SendImpedance(can_port, (uint8_t)joint,
                                0.0f, 0.0f, Kp, Kd, 0.0f);
    }
    usleep(300000);

    // 禁用
    for (uint8_t can_port = 0; can_port < 4; can_port++) {
        motor_mgr.DisableMotor(can_port, (uint8_t)joint);
    }

    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example15 completed\n");
    fflush(stdout);
}

// ================= 示例 16：单电机交互测试 =================
/**
 * @brief 选择电机和测试方法，实时打印状态
 *
 * 方法：
 *   1 - 扭矩测试：阻抗模式下发 +5Nm，打印 位置/扭矩/速度
 *   2 - 位置测试：阻抗模式下标定后位置 +0.5rad，打印 位置/扭矩/速度
 *
 * 使用：选择 CAN 端口 (0~3) → 电机 ID (1~3) → 测试方法 (1/2)
 *       Ctrl+C 退出
 */
void Example16_MotorTest() {
    printf("\n========== Example 16: Motor Interactive Test ==========\n");

    // 选择 CAN 端口
    int can_port = 0;
    printf("Enter CAN port (0~3): ");
    if (scanf("%d", &can_port) != 1 || can_port < 0 || can_port > 3) {
        printf("[ERROR] Invalid CAN port\n");
        return;
    }

    // 选择电机 ID
    int motor_id = 0;
    printf("Enter motor ID (1~3): ");
    if (scanf("%d", &motor_id) != 1 || motor_id < 1 || motor_id > 3) {
        printf("[ERROR] Invalid motor ID\n");
        return;
    }

    // 选择测试方法
    int method = 0;
    printf("\nTest methods:\n");
    printf("  1 - Torque test  (+5Nm impedance control)\n");
    printf("  2 - Position test (current position + 0.5 rad)\n");
    printf("Enter method (1/2): ");
    if (scanf("%d", &method) != 1 || (method != 1 && method != 2)) {
        printf("[ERROR] Invalid method\n");
        return;
    }

    printf("\n[INFO] Testing CAN%d Motor %d — Method %d\n", can_port, motor_id, method);
    printf("[INFO] Press Ctrl+C to stop\n\n");

    // 初始化
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 使能电机
    motor_mgr.EnableMotor((uint8_t)can_port, (uint8_t)motor_id);
    sleep(1);

    // 方法2 需要先读取当前位置作为基准
    MotorStatus init_status = motor_mgr.GetStatus((uint8_t)can_port, (uint8_t)motor_id);
    float target_pos = init_status.position + 0.5f;

    if (method == 2) {
        printf("[INFO] Current position: %.4f rad, target: %.4f rad\n",
               init_status.position, target_pos);
    }

    // 打印表头
    printf("\n%8s | %10s | %10s | %10s | %10s\n",
           "Time", "Pos(rad)", "Torque(Nm)", "Vel(rad/s)", "Target");
    printf("----------|------------|------------|------------|----------\n");
    fflush(stdout);

    // 主循环
    const float Kp = 200.0f;
    const float Kd = 15.0f;
    float elapsed = 0.0f;

    while (true) {
        switch (method) {
            case 1:  // 扭矩测试
                motor_mgr.SendImpedance((uint8_t)can_port, (uint8_t)motor_id,
                                        0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
                break;
            case 2:  // 位置测试
                motor_mgr.SendImpedance((uint8_t)can_port, (uint8_t)motor_id,
                                        target_pos, 0.0f, Kp, Kd, 0.0f);
                break;
        }

        MotorStatus st = motor_mgr.GetStatus((uint8_t)can_port, (uint8_t)motor_id);
        float target_display = (method == 1) ? 5.0f : target_pos;

        printf("%7.1fs | %10.4f | %10.4f | %10.4f | %10.4f\n",
               elapsed, st.position, st.torque, st.velocity, target_display);
        fflush(stdout);

        usleep(100000);
        elapsed += 0.1f;
    }
}
