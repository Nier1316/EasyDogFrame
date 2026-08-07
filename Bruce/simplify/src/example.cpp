#include "example.h"
#include "SimSync.h"
#include "leg_kinematics.h"
#include "math/wheel_position_loop.h"
#include "thread/thread_manager.h"
#include "xbox_controller.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <cstring>
#include <termios.h>   // Example23 终端 raw 模式键盘输入
#include <fcntl.h>     // Example23 非阻塞读 stdin




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
    MotorCalibrationParam current_cal[CAN_PORTS][MOTORS_PER_CAN];
    memcpy(current_cal, MOTOR_CALIBRATION, sizeof(MOTOR_CALIBRATION));
    MotorCalibrationParam suggested[CAN_PORTS][MOTORS_PER_CAN];

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
    printf("static const MotorCalibrationParam MOTOR_CALIBRATION[%d][%d] = {\n",
           CAN_PORTS, MOTORS_PER_CAN);
    for (uint8_t can_port = 0; can_port < CAN_PORTS; can_port++) {
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

// ================= 示例 17：SimSync 仿真集成 =================
/**
 * @brief 连接 MATLAB 仿真，发送关节角控制四足机器狗模型
 *
 * 关节映射：
 *   CAN0 = FL(左前腿) → joints[0..2]
 *   CAN1 = FR(右前腿) → joints[3..5]
 *   CAN2 = RL(左后腿) → joints[6..8]
 *   CAN3 = RR(右后腿) → joints[9..11]
 *
 * 每个 CAN 端口 3 个电机依次为：θ₁(髋外摆), θ₂(大腿), θ₃(小腿)
 *
 * 使用方式：
 *   1. 先启动 MATLAB: quadruped_realtime(12345)
 *   2. 再运行本程序
 */
void Example17_SimSyncIntegration() {
    printf("\n========== Example 17: SimSync Simulation Integration ==========\n");
    printf("[INFO] Connecting to MATLAB simulation at 127.0.0.1:12345\n");
    printf("[INFO] Make sure quadruped_realtime.m is running first\n\n");

    SimSync sim("127.0.0.1", 12345);
    if (!sim.connected()) {
        printf("[ERROR] Failed to connect to simulation server.\n");
        printf("[ERROR] Start quadruped_realtime.m in MATLAB first.\n");
        return;
    }
    printf("[INFO] Connected to simulation!\n\n");

    // 关节角度映射表：CAN端口 → SimSync数组偏移
    struct LegMap {
        uint8_t can_port;      // CAN 端口
        uint8_t joint_offset;  // 在 joints 数组中的起始偏移
        const char* name;
    };
    const LegMap legs[4] = {
        {0, 0,  "FL"},    // CAN0 = 左前腿
        {1, 3,  "FR"},    // CAN1 = 右前腿
        {2, 6,  "RL"},    // CAN2 = 左后腿
        {3, 9,  "RR"},    // CAN3 = 右后腿
    };

    float joints[12] = {0};

    // 设定站立姿态（所有腿相同）
    auto set_standing = [&]() {
        for (int i = 0; i < 4; i++) {
            joints[legs[i].joint_offset + 0] = 0.0f;    // θ1: 髋外摆 0°
            joints[legs[i].joint_offset + 1] = -30.0f;   // θ2: 大腿 -30°
            joints[legs[i].joint_offset + 2] = 60.0f;    // θ3: 小腿 60°
        }
    };

    // ============ Phase 1: 站立姿态 ============
    printf("[Phase 1] Standing pose (3 seconds)...\n");
    set_standing();
    for (int i = 0; i < 150; i++) {  // 3s × 50Hz
        if (!sim.send_deg(joints)) {
            printf("[ERROR] Lost connection during Phase 1.\n");
            return;
        }
        usleep(20000);  // 20ms = 50Hz
    }
    printf("[Phase 1] Done.\n\n");

    // ============ Phase 2: Trot 步态运动 ============
    // FL(0) + RR(3) 同相，FR(1) + RL(2) 反相
    const float PERIOD     = 2.0f;     // 步态周期（秒）
    const float AMP_HIP    = 5.0f;     // 髋外摆幅度
    const float AMP_THIGH  = 15.0f;    // 大腿摆动幅度
    const float AMP_CALF   = 15.0f;    // 小腿摆动幅度
    const float OFFSET     = 30.0f;    // 大腿向前偏移基值
    const float CALF_BASE  = 60.0f;    // 小腿角度基值

    printf("[Phase 2] Trot gait motion (20 seconds)...\n");
    for (int frame = 0; frame < 1000; frame++) {  // 20s × 50Hz
        float t = frame * 0.02f;
        float phase = 2.0f * M_PI * t / PERIOD;

        for (int leg = 0; leg < 4; leg++) {
            // FL(0) 和 RR(3) 同相 → phase
            // FR(1) 和 RL(2) 反相 → phase + π
            float leg_phase = (leg == 1 || leg == 2) ? phase + M_PI : phase;
            float s = sinf(leg_phase);

            joints[legs[leg].joint_offset + 0] =  AMP_HIP * s;         // θ1: 髋外摆
            joints[legs[leg].joint_offset + 1] = -OFFSET + AMP_THIGH * s;  // θ2: 大腿
            joints[legs[leg].joint_offset + 2] =  CALF_BASE - AMP_CALF * s; // θ3: 小腿
        }

        if (!sim.send_deg(joints)) {
            printf("[ERROR] Lost connection during Phase 2.\n");
            return;
        }

        // 每 25 帧（0.5 秒）打印一次
        if (frame % 25 == 0) {
            printf("  t=%5.1f | FL:θ1=%+5.1f θ2=%+5.1f θ3=%5.1f | FR:θ1=%+5.1f θ2=%+5.1f θ3=%5.1f\n",
                   t,
                   joints[0], joints[1], joints[2],
                   joints[3], joints[4], joints[5]);
        }

        usleep(20000);
    }
    printf("[Phase 2] Done.\n\n");

    // ============ Phase 3: 回到站立姿态 ============
    printf("[Phase 3] Return to standing pose (1 second)...\n");
    set_standing();
    for (int i = 0; i < 50; i++) {
        sim.send_deg(joints);
        usleep(20000);
    }
    printf("[Phase 3] Done.\n");

    printf("\n[INFO] Example17 completed. Close the MATLAB figure window to stop the simulation.\n");
    fflush(stdout);
}

/**
 * @brief Example18: 基于 IK 的真实电机控制 + 可选仿真同步
 *
 * 流程:
 *   1. 足端位置 (身体坐标系) → hip_rotation_matrix 逆变换 → 髋坐标系
 *   2. leg_ik() 解算得关节指令角 (rad)
 *   3. 经 ApplyMotorCalibrationInverse 后发送给真实电机
 *   4. 可选: SimSync 同步发仿真看一眼
 *
 * 足端轨迹: 原地 Trot 步态 (对角线腿同相摆动)
 */
void Example18_LegIKControl() {
    printf("\n========== Example 18: IK-Based Motor Control ==========\n");
    printf("[INFO] This example controls real motors using inverse kinematics.\n");
    printf("[INFO] Robot will perform a trot gait via foot trajectory.\n\n");

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 可选: 连接 MATLAB 仿真 ----
    SimSync sim("127.0.0.1", 12345);
    if (sim.connected()) {
        printf("[INFO] SimSync connected to MATLAB simulation\n");
    } else {
        printf("[INFO] SimSync not connected (simulation visualization disabled)\n");
    }

    // ---- 使能所有 12 个电机 ----
    printf("[INFO] Enabling all 12 motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    sleep(1);
    printf("[INFO] All motors enabled\n\n");
    fflush(stdout);

    // ---- 控制参数 ----
    const float KP = 200.0f;
    const float KD = 15.0f;
    const float PERIOD = 1.0f;        // 步态周期 (秒)
    const float STEP_LEN = 0.25f;      // 步长 (m)
    const float STEP_HEIGHT = 0.25f;   // 抬腿高度 (m)
    const int   HZ = 1000;             // 控制频率 1kHz
    const int   TOTAL_FRAMES = 20000;  // 运行 10 秒

    // 站立基值: 足端在身体坐标系中的位置
    // 计算: 用 leg_fk_all 验证过的站立姿态 [0°, -30°, 60°]
    float stand_foot[4][3];
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg*3 + 0] = deg2rad(0);
        stand_q[leg*3 + 1] = deg2rad(-30);
        stand_q[leg*3 + 2] = deg2rad(60);
    }
    leg_fk_all(stand_q, stand_foot);

    // ---- 主循环 ----
    printf("Time(s) | Phase | FL_z    | FR_z    | RL_z    | RR_z\n");
    printf("--------|-------|---------|---------|---------|---------\n");
    fflush(stdout);

    for (int frame = 0; frame < TOTAL_FRAMES; frame++) {
        float t = frame * (1.0f / HZ);
        float phase = 2.0f * M_PI * t / PERIOD;

        // 12 个关节角度 (度, 给 SimSync)
        float sim_joints_deg[12];

        for (int leg = 0; leg < 4; leg++) {
            // 对角线腿 (FL+RR 同相, FR+RL 同相)
            float leg_phase = (leg == FL || leg == RR) ? phase : phase + M_PI;
            float swing = std::sin(leg_phase);
            float lift = (1.0f - std::cos(leg_phase)) * 0.5f;  // 0→1 抬腿

            // 足端位移量 (身体坐标系)
            float dx = STEP_LEN * swing;          // X 方向摆动
            float dy = 0.0f;                       // Y 方向(外摆)不动
            float dz = STEP_HEIGHT * lift;         // Z 方向抬腿

            float foot_target_body[3] = {
                stand_foot[leg][0] + dx,
                stand_foot[leg][1] + dy,
                stand_foot[leg][2] + dz,
            };

            // 身体坐标系 → 髋坐标系
            float p_hip[3];
            float R[3][3], Rt[3][3];
            hip_rotation_matrix(static_cast<LegIndex>(leg), R);
            // 逆变换 (R^T)
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    Rt[i][j] = R[j][i];
                }
            }
            float mount_to_foot[3];
            for (int i = 0; i < 3; i++) {
                mount_to_foot[i] = foot_target_body[i] - LEG_MOUNT[leg][i];
            }
            for (int i = 0; i < 3; i++) {
                p_hip[i] = 0;
                for (int j = 0; j < 3; j++) {
                    p_hip[i] += Rt[i][j] * mount_to_foot[j];
                }
            }

            // IK 解算
            float q_cmd[3];
            leg_ik(p_hip, LEG_L1, LEG_L2, LEG_L3,
                   THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET,
                   q_cmd);

            // 钳位到限位范围（指令角坐标，与标定坐标系一致）
            q_cmd[0] = clamp(q_cmd[0],
                deg2rad(LOWER_LIMIT_THETA1_DEG),
                deg2rad(UPPER_LIMIT_THETA1_DEG));
            q_cmd[1] = clamp(q_cmd[1],
                deg2rad(LOWER_LIMIT_THETA2_DEG),
                deg2rad(UPPER_LIMIT_THETA2_DEG));
            q_cmd[2] = clamp(q_cmd[2],
                deg2rad(LOWER_LIMIT_THETA3_DEG),
                deg2rad(UPPER_LIMIT_THETA3_DEG));

            // IK 输出的指令角即为标定坐标系下的目标位置
            // SendThreadFunc 内部自动做逆标定
            for (int j = 0; j < 3; j++) {
                motor_mgr.SendImpedance(leg, j + 1, q_cmd[j], 0.0f, KP, KD, 0.0f);
            }

            // SimSync 用: 指令角 (度)
            sim_joints_deg[leg*3 + 0] = rad2deg(q_cmd[0]);
            sim_joints_deg[leg*3 + 1] = rad2deg(q_cmd[1]);
            sim_joints_deg[leg*3 + 2] = rad2deg(q_cmd[2]);
        }

        // 可选: 发送到仿真
        if (sim.connected()) {
            sim.send_deg(sim_joints_deg);
        }

        // 打印 (500ms 一次)
        if (frame % 500 == 0) {
            MotorStatus st_fl = motor_mgr.GetStatus(FL, 1);
            printf("%7.2f | %5.1f | %7.4f | %7.4f | %7.4f | %7.4f\n",
                   t, phase/(2*M_PI)*PERIOD,
                   st_fl.position, 0.0f, 0.0f, 0.0f);
            fflush(stdout);
        }

        usleep(1000000 / HZ);  // 20ms
    }

    // ---- 归零站立 ----
    printf("\n[INFO] Returning to standing position...\n");
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            motor_mgr.SendImpedance(leg, j + 1, stand_q[leg*3 + j], 0.0f, KP, KD, 0.0f);
        }
    }
    usleep(500000);

    // ---- 禁用所有电机 ----
    printf("[INFO] Disabling all motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    // ---- 清理 ----
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example18 completed\n");
    fflush(stdout);
}

// ================= 示例 19：读取当前姿态并缓慢移动到站立 =================
void Example19_ReadAndStand() {
    printf("\n========== Example 19: Read Current Pose & Slow Stand ==========\n");
    printf("[INFO] Phase 1: Read current joint angles\n");
    printf("[INFO] Phase 2: Slowly interpolate to standing pose\n\n");

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] Failed to initialize MotorManager\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 使能所有 12 个电机 ----
    printf("[INFO] Enabling all 12 motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    sleep(1);
    printf("[INFO] All motors enabled\n\n");

    // ======== 阶段 1：读取当前姿态 ========
    printf("========== Phase 1: Current Joint Angles ==========\n");
    const char* leg_names[] = {"FL(CAN0)", "FR(CAN1)", "RL(CAN2)", "RR(CAN3)"};
    const char* joint_names[] = {"Hip", "Thigh", "Calf"};
    float cur_phys[4][3];  // 当前物理角 (rad)

    for (int leg = 0; leg < 4; leg++) {
        printf("  %s:\n", leg_names[leg]);
        for (int j = 0; j < 3; j++) {
            MotorStatus st = motor_mgr.GetStatus(leg, j + 1);
            cur_phys[leg][j] = st.position;  // 已标定的物理角 (rad)
            printf("    %s: %7.4f rad (%6.2f°)  %s%s\n",
                   joint_names[j],
                   st.position,
                   rad2deg(st.position),
                   st.enable ? "" : " [DISABLED]",
                   st.fault ? " [FAULT]" : "");
        }
    }
    printf("\n");

    // ======== 阶段 2：缓慢移动到站立姿态 ========
    // 站立姿态 (标定坐标系): 指令角 [0°, -60°, 60°]
    //   标定坐标系 0 = 物理零位
    printf("========== Phase 2: Moving to Standing Pose ==========\n");
    printf("  Target: Hip=%5.1f°, Thigh=%5.1f°, Calf=%5.1f°\n\n",
           0.0f, -30.0f, 60.0f);
    fflush(stdout);

    const float TGT_PHYS[3] = {
        deg2rad(0.0f),       // Hip:   0°  horizontal
        deg2rad(-60.0f),     // Thigh: -60°
        deg2rad(60.0f),      // Calf:   60°
    };

    const float STAND_DURATION = 10.0f;  // 过渡时间 (秒)
    const int   HZ = 100;               // 控制频率 100Hz
    const int   TOTAL_FRAMES = (int)(STAND_DURATION * HZ);
    const float KP = 200.0f;    // 降低刚度减少振荡
    const float KD = 20.0f;    // 提高阻尼抑制超调

    for (int frame = 0; frame <= TOTAL_FRAMES; frame++) {
        float t = (float)frame / TOTAL_FRAMES;  // 0.0 → 1.0

        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                // 线性插值 (物理角)，SendThreadFunc 内部会自动做逆标定
                float pos = cur_phys[leg][j] + (TGT_PHYS[j] - cur_phys[leg][j]) * t;
                motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, KP, KD, 0.0f);
            }
        }

        if (frame % 50 == 0) {  // 每 0.5 秒打印进度
            printf("  [Progress] %3.0f%%\n", t * 100.0f);
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // ---- 保持站立 ----
    printf("\n[INFO] Holding standing pose...\n");
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            motor_mgr.SendImpedance(leg, j + 1, TGT_PHYS[j], 0.0f, KP, KD, 0.0f);
        }
    }
    sleep(20);

    // ---- 禁用所有电机 ----
    printf("[INFO] Disabling all motors...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    // ---- 清理 ----
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example19 completed\n");
    fflush(stdout);
}

// ================= 示例 20：选择电机移动到物理零位 =================
void Example20_MoveToPhysicalZero() {
    printf("\n========== Example 20: Move Selected Motor to Physical Zero ==========\n");
    printf("[INFO] Calibrated coordinate: position 0 = physical zero\n");
    printf("  Motor 1 (Hip):   target = 0  (horizontal)\n");
    printf("  Motor 2 (Thigh): target = 0  (vertical)\n");
    printf("  Motor 3 (Calf):  target = π/2  (90°, physical limit)\n\n");

    // ---- 先选择目标电机（只使能选中的电机，避免全部上电过流） ----
    int can_port = -1, motor_id = -1;
    printf("选择 CAN 端口 (0~3): ");
    fflush(stdout);
    if (scanf("%d", &can_port) != 1) { printf("[ERROR] 输入无效\n"); return; }
    printf("选择电机 ID (1~3): ");
    fflush(stdout);
    if (scanf("%d", &motor_id) != 1) { printf("[ERROR] 输入无效\n"); return; }

    if (!(can_port >= 0 && can_port <= 3 && motor_id >= 1 && motor_id <= 3)) {
        printf("[ERROR] CAN 端口 0~3, 电机 ID 1~3\n");
        return;
    }

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 只使能选中的单个电机 ----
    printf("[INFO] 使能 CAN%d-M%d...\n", can_port, motor_id);
    motor_mgr.EnableMotor(can_port, motor_id);
    usleep(300000);

    // ---- 读取当前位置 ----
    const float KP = 200.0f, KD = 10.0f;
    MotorStatus st = motor_mgr.GetStatus(can_port, motor_id);
    float start_pos = st.position;
    printf("[INFO] 当前位置: %.4f rad (%.2f°)\n", start_pos, rad2deg(start_pos));

    // 先发当前位置保持不动
    motor_mgr.SendImpedance(can_port, motor_id, start_pos, 0.0f, KP, KD, 0.0f);
    usleep(100000);

    // ---- 计算目标 ----
    float tgt_pos;
    const char* desc;
    if (motor_id == 3) {
        tgt_pos = M_PI_2;           // 小腿从水平位弯曲 90°
        desc = "小腿 90°";
    } else if (motor_id == 1) {
        tgt_pos = 0.0f;
        desc = "髋水平";
    } else {
        tgt_pos = 0.0f;
        desc = "大腿竖直";
    }

    printf("\n[INFO] CAN%d-M%d: %s\n", can_port, motor_id, desc);
    printf("[INFO] 当前: %.4f rad (%.2f°)  →  目标: %.4f rad (%.2f°)\n\n",
           start_pos, rad2deg(start_pos), tgt_pos, rad2deg(tgt_pos));
    fflush(stdout);

    // ---- 2 秒缓慢插值到目标 ----
    const int HZ = 100;
    const int FRAMES = 200;
    for (int f = 0; f <= FRAMES; f++) {
        float t = (float)f / FRAMES;
        float pos = start_pos + (tgt_pos - start_pos) * t;
        motor_mgr.SendImpedance(can_port, motor_id, pos, 0.0f, KP, KD, 0.0f);
        if (f % 50 == 0) {
            MotorStatus st_now = motor_mgr.GetStatus(can_port, motor_id);
            printf("  [%3d%%] cmd=%.4f rad (%.2f°), actual=%.4f rad (%.2f°)\n",
                   (int)(t * 100), pos, rad2deg(pos),
                   st_now.position, rad2deg(st_now.position));
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // ---- 保持 2 秒 ----
    printf("\n[INFO] 保持中...\n");
    motor_mgr.SendImpedance(can_port, motor_id, tgt_pos, 0.0f, KP, KD, 0.0f);
    sleep(2);

    st = motor_mgr.GetStatus(can_port, motor_id);
    printf("[INFO] 最终: CAN%d-M%d = %.4f rad (%.2f°)\n\n",
           can_port, motor_id, st.position, rad2deg(st.position));

    // ---- 清理 ----
    printf("[INFO] 禁用电机...\n");
    motor_mgr.DisableMotor(can_port, motor_id);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example20 完成\n");
    fflush(stdout);
}

// ================= 示例 21：Xbox 手柄控制 =================
void Example21_XboxControllerControl() {
    printf("\n========== 示例 21：Xbox 手柄控制 ==========\n");
    printf("[INFO] A键  = 起立（记录按下时的姿态作为返回点）\n");
    printf("[INFO] B键  = 缓慢回到起立前的初始姿态\n");
    printf("[INFO] LT扳机 = 升高身体  |  RT扳机 = 降低身体\n");
    printf("[INFO] 右摇杆上下 = 前进/后退  |  右摇杆左右 = 差速转向\n");
    printf("[INFO] Back键 = 退出\n\n");

    // ---- 控制参数 ----
    // 关节 kp/kd/重力前馈已移到 include/motor_calibration.h 的 JOINT_IMPEDANCE 表，
    // 按 [can_port][joint] 逐关节可调，用 GetJointImpedance(leg, j+1) 取。
    // 控制周期、机身高度范围、轮电机参数见 robot_calibration.h §5，
    // 此处不再重复定义（同名局部常量会遮蔽表中的值，改表不生效）。
    const int   HZ = CONTROL_HZ;                 // 主循环频率
    const float WHEEL_DEAD_ZONE = 0.05f;         // 轮子摇杆死区 (归一化，仅本例使用)

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 使能腿部电机（motor_id 1~3）
    printf("[INFO] 使能腿部电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    // 使能轮电机（motor_id 4）
    for (int cp = 0; cp < 4; cp++) {
        motor_mgr.EnableMotor(cp, 4);
    }
    usleep(200000);

    // ---- 初始化 Xbox 手柄 ----
    XboxController controller;
    bool controller_ok = controller.Initialize();
    if (!controller_ok) {
        printf("[ERROR] 未检测到 Xbox 手柄，退出\n");
    }

    // ---- 预计算站立姿态足端位置 ----
    float stand_q[12] = {};
    float base_foot_body[4][3] = {};
    if (controller_ok) {
        // 站立指令角见 robot_calibration.h §5 STAND_*_DEG
        for (int leg = 0; leg < 4; leg++) {
            stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
            stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
            stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
        }
        leg_fk_all(stand_q, base_foot_body);

        printf("[INFO] 站立足端位置 (身体坐标系):\n");
        for (int leg = 0; leg < 4; leg++) {
            printf("  腿%d: [%+.4f, %+.4f, %+.4f] m\n",
                   leg, base_foot_body[leg][0], base_foot_body[leg][1], base_foot_body[leg][2]);
        }
    }

    // ---- 状态变量 ----
    bool standing = false;          // 是否已起立
    float body_height = 0.0f;       // 身体高度偏移量
    bool prev_a = false;            // A键上一帧状态（用于上升沿检测）
    bool prev_b = false;            // B键上一帧状态
    bool prev_back = false;         // Back键上一帧状态

    // 按下 A 键那一刻的关节姿态，B 键据此原路返回。
    // 必须在起立阶段之外声明：主循环里 B 键要用到它。
    float start_pos[4][3] = {};

    // ---- 阶段 1：等待 A 键起立 ----
    if (controller_ok) {
        printf("\n[INFO] 按下 A 键起立...\n");
        fflush(stdout);

        while (!standing) {
            controller.Poll();
            if (!controller.IsConnected()) {
                printf("[ERROR] 手柄断开连接\n");
                break;
            }

            const XboxState& state = controller.GetState();

            // A键上升沿检测
            bool a_pressed = state.a && !prev_a;
            prev_a = state.a;

            if (a_pressed) {
                standing = true;
                // 在按下的这一刻记录姿态：此时电机还没被起立指令推动，
                // 读到的就是真正的初始位置，B 键返回的目标即为此。
                for (int leg = 0; leg < 4; leg++) {
                    for (int j = 0; j < 3; j++) {
                        start_pos[leg][j] = motor_mgr.GetStatus(leg, j + 1).position;
                    }
                }
                printf("[INFO] A键按下！已记录初始姿态，开始起立...\n");
                for (int leg = 0; leg < 4; leg++) {
                    printf("  腿%d 初始角: [%+.4f, %+.4f, %+.4f] rad\n",
                           leg, start_pos[leg][0], start_pos[leg][1], start_pos[leg][2]);
                }
                fflush(stdout);
            }

            usleep(10000);
        }
    }

    // ---- 阶段 2：2秒插值到站立姿态 ----
    if (controller_ok && standing) {
        const int TOTAL_INTERP_FRAMES = STAND_INTERP_FRAMES;  // 见 robot_calibration.h §5
        // start_pos 已在 A 键按下时记录，此处直接用

        // 线性插值过渡
        for (int f = 0; f <= TOTAL_INTERP_FRAMES; f++) {
            float t = (float)f / TOTAL_INTERP_FRAMES;
            if (t > 1.0f) t = 1.0f;

            for (int leg = 0; leg < 4; leg++) {
                for (int j = 0; j < 3; j++) {
                    float pos = start_pos[leg][j]
                              + (stand_q[leg * 3 + j] - start_pos[leg][j]) * t;
                    const JointImpedanceParam& ip = GetJointImpedance(leg, j + 1);
                    // 前馈随插值系数 t 渐入，避免起立结束切主循环时扭矩跳变
                    motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f,
                                            ip.kp, ip.kd, ip.tau_ff * t);
                }
            }

            if (f % 50 == 0) {
                printf("  起立中... %3.0f%%\n", t * 100.0f);
                fflush(stdout);
            }

            usleep(1000000 / HZ);
        }

        printf("[INFO] 已站立。LT/RT=高度, 右摇杆=轮子, Back=退出.\n\n");
        fflush(stdout);

        // ---- 阶段 3：主控制循环 ----
        int frame = 0;
        while (true) {
            controller.Poll();
            if (!controller.IsConnected()) {
                printf("[INFO] 手柄断开，退出...\n");
                break;
            }

            const XboxState& state = controller.GetState();

            // Back键上升沿 → 退出
            bool back_pressed = state.back && !prev_back;
            prev_back = state.back;
            if (back_pressed) {
                printf("[INFO] Back键按下，退出...\n");
                break;
            }

            // B键上升沿 → 缓慢回到 A 键按下时记录的初始姿态
            bool b_pressed = state.b && !prev_b;
            prev_b = state.b;
            if (b_pressed) {
                printf("[INFO] B键按下！缓慢回到初始姿态...\n");
                fflush(stdout);

                // 先停轮子：坐下过程中轮子不该继续转
                for (int cp = 0; cp < 4; cp++) {
                    motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);
                }

                // 从"当前实际位置"而非站立目标插值：手柄调过高度后
                // 两者已不同，用实际位置起步才不会有跳变。
                float from_pos[4][3];
                for (int leg = 0; leg < 4; leg++) {
                    for (int j = 0; j < 3; j++) {
                        from_pos[leg][j] = motor_mgr.GetStatus(leg, j + 1).position;
                    }
                }

                for (int f = 0; f <= STAND_INTERP_FRAMES; f++) {
                    float t = (float)f / STAND_INTERP_FRAMES;
                    if (t > 1.0f) t = 1.0f;

                    for (int leg = 0; leg < 4; leg++) {
                        for (int j = 0; j < 3; j++) {
                            float pos = from_pos[leg][j]
                                      + (start_pos[leg][j] - from_pos[leg][j]) * t;
                            const JointImpedanceParam& ip = GetJointImpedance(leg, j + 1);
                            // 前馈随 t 渐出，落地后不再顶着前馈
                            motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f,
                                                    ip.kp, ip.kd,
                                                    ip.tau_ff * (1.0f - t));
                        }
                    }

                    if (f % 100 == 0) {
                        printf("  回落中... %3.0f%%\n", t * 100.0f);
                        fflush(stdout);
                    }
                    usleep(1000000 / HZ);
                }

                printf("[INFO] 已回到初始姿态，退出主循环\n");
                fflush(stdout);
                break;
            }

            // ---- 身体高度调节（左/右扳机） ----
            const float TRIGGER_DEAD = 0.05f;
            float height_delta = 0.0f;
            if (state.left_trigger  > TRIGGER_DEAD)
                height_delta += HEIGHT_ADJUST_RATE * state.left_trigger  / HZ;
            if (state.right_trigger > TRIGGER_DEAD)
                height_delta -= HEIGHT_ADJUST_RATE * state.right_trigger / HZ;

            body_height += height_delta;
            body_height = clamp(body_height, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX);

            // ---- 轮子控制（右摇杆：Y 轴前后，X 轴左右转，SPEED 模式）----
            // 摇杆归一化输入，各自去死区
            float fwd_stick  = -state.right_stick_y;   // 上推为正 = 前进
            float turn_stick =  state.right_stick_x;   // 右推为正 = 右转
            if (fabsf(fwd_stick)  < WHEEL_DEAD_ZONE) fwd_stick  = 0.0f;
            if (fabsf(turn_stick) < WHEEL_DEAD_ZONE) turn_stick = 0.0f;

            // 差速：右转时左侧加速、右侧减速。X 轴单独推即原地转向。
            float v_fwd  = fwd_stick  * WHEEL_MAX_SPEED;
            float v_turn = turn_stick * WHEEL_MAX_TURN;
            float v_left  = v_fwd + v_turn;
            float v_right = v_fwd - v_turn;

            // 合成速度可能超过单轮上限。按同一比例缩放两侧而非各自钳位，
            // 否则左右差值被改变，转弯半径会随速度漂移。
            float v_peak = fmaxf(fabsf(v_left), fabsf(v_right));
            if (v_peak > WHEEL_SPEED_CAP) {
                float scale = WHEEL_SPEED_CAP / v_peak;
                v_left  *= scale;
                v_right *= scale;
            }

            // CAN0(FL)/CAN2(RL) 为左侧，CAN1(FR)/CAN3(RR) 为右侧
            motor_mgr.SendSpeed(0, 4, v_left,  WHEEL_KVP, WHEEL_KVI);
            motor_mgr.SendSpeed(2, 4, v_left,  WHEEL_KVP, WHEEL_KVI);
            motor_mgr.SendSpeed(1, 4, v_right, WHEEL_KVP, WHEEL_KVI);
            motor_mgr.SendSpeed(3, 4, v_right, WHEEL_KVP, WHEEL_KVI);

            // ---- 四腿 IK 解算并发送指令 ----
            for (int leg = 0; leg < 4; leg++) {
                // 根据身体高度偏移量调整足端目标 Z 分量
                float foot_target_body[3] = {
                    base_foot_body[leg][0],
                    base_foot_body[leg][1],
                    base_foot_body[leg][2] - body_height,
                };

                // 身体坐标系 → 髋坐标系: R^T × (foot_body - LEG_MOUNT)
                float R[3][3], Rt[3][3];
                hip_rotation_matrix(static_cast<LegIndex>(leg), R);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        Rt[i][j] = R[j][i];

                float mount_to_foot[3];
                for (int i = 0; i < 3; i++)
                    mount_to_foot[i] = foot_target_body[i] - LEG_MOUNT[leg][i];

                float p_hip[3] = {0};
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        p_hip[i] += Rt[i][j] * mount_to_foot[j];

                // IK 反解
                float q_cmd[3];
                leg_ik(p_hip, LEG_L1, LEG_L2, LEG_L3,
                       THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET, q_cmd);

                // 钳位到关节限位
                q_cmd[0] = clamp(q_cmd[0],
                    deg2rad(LOWER_LIMIT_THETA1_DEG), deg2rad(UPPER_LIMIT_THETA1_DEG));
                q_cmd[1] = clamp(q_cmd[1],
                    deg2rad(LOWER_LIMIT_THETA2_DEG), deg2rad(UPPER_LIMIT_THETA2_DEG));
                q_cmd[2] = clamp(q_cmd[2],
                    deg2rad(LOWER_LIMIT_THETA3_DEG), deg2rad(UPPER_LIMIT_THETA3_DEG));

                // 发送阻抗控制指令（kp/kd/前馈见 JOINT_IMPEDANCE 表）
                for (int j = 0; j < 3; j++) {
                    const JointImpedanceParam& ip = GetJointImpedance(leg, j + 1);
                    motor_mgr.SendImpedance(leg, j + 1, q_cmd[j], 0.0f,
                                            ip.kp, ip.kd, ip.tau_ff);
                }
            }

            // 每 0.5 秒打印状态（轮子显示目标角速度与 CAN0 反馈速度）
            if (frame % 50 == 0) {
                MotorStatus w0 = motor_mgr.GetStatus(0, 4);
                MotorStatus w1 = motor_mgr.GetStatus(1, 4);
                printf("  高度=%+.3fm  轮目标 左=%+.2f 右=%+.2f  "
                       "实测 轮0=%+.2f 轮1=%+.2f rad/s  LT=%.2f RT=%.2f\n",
                       body_height, v_left, v_right,
                       w0.velocity, w1.velocity,
                       state.left_trigger, state.right_trigger);
                fflush(stdout);
            }

            frame++;
            usleep(1000000 / HZ);
        }
    } // 起立 & 主循环结束

    // ---- 清理 ----
    printf("[INFO] 正在关闭...\n");

    // 停止轮电机（速度模式，目标速度归零）
    for (int cp = 0; cp < 4; cp++) {
        motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);
    }

    // 禁用所有 16 个电机
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    controller.Shutdown();
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] 示例21 完成\n");
    fflush(stdout);
}

// ================= 示例 22：起立 + 轮子阻抗模式测试 =================
void Example22_StandAndWheelTest() {
    printf("\n========== 示例 22：起立 + 轮子阻抗模式测试 ==========\n");
    printf("[INFO] 阶段1: 读取当前姿态\n");
    printf("[INFO] 阶段2: 缓慢起立\n");
    printf("[INFO] 阶段3: 4秒后轮子以 1rad/s 滚动 (KP=3, KD=0.3)\n\n");

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 使能所有 12 条腿电机 + 4 个轮电机 ----
    printf("[INFO] 使能 12 条腿电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
    }
    printf("[INFO] 使能 4 个轮电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        motor_mgr.EnableMotor(cp, 4);
    }
    usleep(500000);  // 等轮电机完全就绪

    // 轮电机统一用厂家速度模式（SPEED），上位机只给目标角速度
    printf("[INFO] 轮电机速度归零保持...\n");
    const float WHEEL_LOOP_VEL = 1.0f;   // 阶段3滚动角速度 (rad/s)
    const float WHEEL_KVP      = 1.0f;   // 速度环 Kp — 初值，实测再调
    const float WHEEL_KVI      = 0.0f;   // 速度环 Ki — 初值，实测再调
    for (int cp = 0; cp < 4; cp++) {
        MotorStatus st = motor_mgr.GetStatus(cp, 4);
        motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);  // 目标速度 0
        printf("  CAN%d-M4: %.4f rad (%.2f°)\n", cp, st.position, rad2deg(st.position));
    }
    fflush(stdout);

    // ======== 阶段 1：等待数据就绪并读取当前姿态 ========
    printf("========== 阶段1: 等待数据就绪 ==========\n");
    {
        MotorStatus st = motor_mgr.GetStatus(0, 1);
        int retry = 0;
        while (fabsf(st.position) < 0.001f && fabsf(st.velocity) < 0.001f && retry < 100) {
            usleep(50000);
            st = motor_mgr.GetStatus(0, 1);
            retry++;
        }
        printf("[INFO] 等待 %d 次 (%.1f s) 后数据就绪\n", retry, retry * 0.05f);
    }

    float cur_pos[4][3];
    printf("当前关节角度:\n");
    for (int leg = 0; leg < 4; leg++) {
        printf("  CAN%d: ", leg);
        for (int j = 0; j < 3; j++) {
            MotorStatus st = motor_mgr.GetStatus(leg, j + 1);
            cur_pos[leg][j] = st.position;
            printf("  M%d=%.2f°", j + 1, rad2deg(st.position));
        }
        printf("\n");
    }
    fflush(stdout);

    // ======== 阶段 2：缓慢起立 ========
    printf("\n========== 阶段2: 起立 ==========\n");
    const float TGT[3] = { deg2rad(0.0f), deg2rad(-30.0f), deg2rad(60.0f) };
    const float KP = 150.0f, KD = 20.0f;
    const int   STAND_DURATION = 3;
    const int   HZ = 100;
    const int   TOTAL_FRAMES = STAND_DURATION * HZ;

    for (int frame = 0; frame <= TOTAL_FRAMES; frame++) {
        float t = (float)frame / TOTAL_FRAMES;
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                float pos = cur_pos[leg][j] + (TGT[j] - cur_pos[leg][j]) * t;
                motor_mgr.SendImpedance(leg, j + 1, pos, 0.0f, KP, KD, 0.0f);
            }
        }
        if (frame % 50 == 0) {
            printf("  起立中... %3.0f%%\n", t * 100.0f);
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // 保持站立 4 秒
    printf("\n[INFO] 保持站立 4 秒...\n");
    for (int sec = 1; sec <= 4; sec++) {
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 3; j++) {
                motor_mgr.SendImpedance(leg, j + 1, TGT[j], 0.0f, KP, KD, 0.0f);
            }
        }
        // 持续保持轮子速度为 0（速度模式锁停）
        for (int cp = 0; cp < 4; cp++) {
            motor_mgr.SendSpeed(cp, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);
        }
        sleep(1);
        printf("  %d/4 秒\n", sec);
        fflush(stdout);
    }

    // ======== 阶段 3：轮电机速度模式滚动 ========
    printf("\n========== 阶段3: 轮电机速度模式滚动 (%.0frad/s, KVP=%.1f, KVI=%.1f) ==========\n",
           WHEEL_LOOP_VEL, WHEEL_KVP, WHEEL_KVI);
    printf("[INFO] 轮子开始滚动...\n");
    fflush(stdout);

    const int WHEEL_DURATION = 10;

    // 保持腿姿态
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            motor_mgr.SendImpedance(leg, j + 1, TGT[j], 0.0f, KP, KD, 0.0f);

    const int WHEEL_HZ = 50;
    for (int sec = 0; sec < WHEEL_DURATION; sec++) {
        for (int f = 0; f < WHEEL_HZ; f++) {
            for (int cp = 0; cp < 4; cp++) {
                // 恒定目标角速度，电机端闭速度环
                motor_mgr.SendSpeed(cp, 4, WHEEL_LOOP_VEL, WHEEL_KVP, WHEEL_KVI);
            }
            usleep(1000000 / WHEEL_HZ);
        }

        // 重新发送腿姿态
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1, TGT[j], 0.0f, KP, KD, 0.0f);

        MotorStatus w0 = motor_mgr.GetStatus(0, 4);
        printf("  轮子 %d/%d 秒  目标=%.2f 轮0实测=%.2f rad/s\n", sec + 1, WHEEL_DURATION,
               WHEEL_LOOP_VEL, w0.velocity);
        fflush(stdout);
    }

    // ---- 清理 ----
    printf("\n[INFO] 停止并禁用所有电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.SendImpedance(cp, mi, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
    usleep(500000);

    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.DisableMotor(cp, mi);
        }
    }

    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] 示例22 完成\n");
    fflush(stdout);
}

// ================= 示例 23：单路 CAN 键盘控制 =================
// 终端输入 0~3 选择 CAN 路；该路 3 个关节电机移动到站立姿态；
// 键盘 ↑/↓ 控制离地高度，←/→ 控制轮电机反转/正转 0.5rad/s。
// 只使能/控制选中的这一路（1 腿 3 关节 + 1 轮），其余三路不动。

namespace {
// 终端 raw 模式管理：进入时关闭行缓冲/回显并设非阻塞，析构自动恢复。
// 与项目其它示例一致，直接用 STDIN_FILENO —— 请从集成终端（Run Program 任务）
// 运行本示例，此时 stdin 是真实终端；调试器内部 Debug Console 不是终端，无法交互。
struct RawTerminal {
    termios old_tio{};
    int     old_flags = 0;
    bool    ok = false;
    RawTerminal() {
        if (tcgetattr(STDIN_FILENO, &old_tio) != 0) return;
        termios raw = old_tio;
        raw.c_lflag &= ~(ICANON | ECHO);   // 关闭行缓冲和回显
        raw.c_cc[VMIN]  = 0;               // 非阻塞：无输入立即返回
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
        old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
        ok = true;
    }
    ~RawTerminal() {
        if (!ok) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }
};

// 方向键解析结果
enum class KeyDir { NONE, UP, DOWN, LEFT, RIGHT, QUIT };

// 非阻塞读取一个方向键（方向键是 ESC [ A/B/C/D 三字节序列）。
// 读到 'q' 返回 QUIT；无输入返回 NONE。
KeyDir poll_key() {
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return KeyDir::NONE;
    if (c == 'q' || c == 'Q') return KeyDir::QUIT;
    if (c != 0x1b) return KeyDir::NONE;          // 非 ESC，忽略
    unsigned char seq[2];
    if (read(STDIN_FILENO, &seq[0], 1) != 1) return KeyDir::NONE;
    if (read(STDIN_FILENO, &seq[1], 1) != 1) return KeyDir::NONE;
    if (seq[0] != '[') return KeyDir::NONE;
    switch (seq[1]) {
        case 'A': return KeyDir::UP;
        case 'B': return KeyDir::DOWN;
        case 'C': return KeyDir::RIGHT;
        case 'D': return KeyDir::LEFT;
        default:  return KeyDir::NONE;
    }
}
} // namespace

void Example23_SingleCanKeyboardControl() {
    printf("\n========== 示例 23：单路 CAN 键盘控制 ==========\n");

    // ---- 选择 CAN 路（与其它示例一致，阻塞式 scanf，此时终端为正常行模式）----
    int can_port = -1;
    printf("请输入要控制的 CAN 路编号 (0~3): ");
    fflush(stdout);
    if (scanf("%d", &can_port) != 1 || can_port < 0 || can_port > 3) {
        printf("[ERROR] 无效输入，退出\n");
        return;
    }
    // 吞掉行尾换行，避免污染后续 raw 读取
    int ch; while ((ch = getchar()) != '\n' && ch != EOF) {}
    printf("[INFO] 选择 CAN%d（腿 %d + 轮）\n", can_port, can_port);

    // ---- 控制参数 ----
    const float KP = 150.0f;                 // 关节阻抗刚度
    const float KD = 20.0f;                  // 关节阻抗阻尼
    const int   HZ = 100;                    // 主循环频率
    const float HEIGHT_STEP  = 0.002f;       // 每帧按住方向键的高度调节量 (m)
    const float BODY_HEIGHT_MIN = -0.15f;    // 最低（深蹲）
    const float BODY_HEIGHT_MAX =  0.10f;    // 最高（站立）
    const float WHEEL_SPEED  = 1.5f;         // 轮子正/反转角速度 (rad/s)
    const float WHEEL_KVP    = 3.0f;         // 速度环 Kp — 初值，实测再调
    const float WHEEL_KVI    = 0.3f;         // 速度环 Ki — 初值，实测再调
    // 松开方向键后轮子指令保持的帧数（终端无按键释放事件，用超时判定停止）
    const int   WHEEL_HOLD_FRAMES = 8;

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 只使能选中路的 3 个关节 + 1 个轮
    printf("[INFO] 使能 CAN%d 电机...\n", can_port);
    for (int mi = 1; mi <= 4; mi++) motor_mgr.EnableMotor(can_port, mi);
    usleep(300000);

    // ---- 预计算该腿站立姿态足端位置 ----
    const int leg = can_port;  // CAN 路 == LegIndex (CAN0=FL...CAN3=RR)
    float stand_q[12] = {};
    float base_foot_body[4][3] = {};
    for (int l = 0; l < 4; l++) {
        stand_q[l * 3 + 0] = deg2rad(0);    // Hip
        stand_q[l * 3 + 1] = deg2rad(-30);  // Thigh
        stand_q[l * 3 + 2] = deg2rad(60);   // Calf
    }
    leg_fk_all(stand_q, base_foot_body);

    // ---- 缓慢移动到站立姿态（2 秒插值）----
    printf("[INFO] 移动到站立姿态...\n");
    float start_pos[3];
    for (int j = 0; j < 3; j++)
        start_pos[j] = motor_mgr.GetStatus(can_port, j + 1).position;

    const int STAND_FRAMES = 200;  // 2s × 100Hz
    for (int f = 0; f <= STAND_FRAMES; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int j = 0; j < 3; j++) {
            float pos = start_pos[j] + (stand_q[leg * 3 + j] - start_pos[j]) * t;
            motor_mgr.SendImpedance(can_port, j + 1, pos, 0.0f, KP, KD, 0.0f);
        }
        usleep(1000000 / HZ);
    }
    printf("[INFO] 已站立\n");

    // 轮电机初始速度归零（厂家 SPEED 模式）
    motor_mgr.SendSpeed(can_port, 4, 0.0f, WHEEL_KVP, WHEEL_KVI);

    // ---- 进入终端 raw 模式，开始键盘控制 ----
    printf("\n[操作] ↑/↓ = 升高/降低身体   ←/→ = 轮子反转/正转 0.5rad/s   q = 退出\n\n");
    fflush(stdout);

    RawTerminal term;
    if (!term.ok) {
        printf("[ERROR] 无法设置终端 raw 模式，退出\n");
    } else {
        float body_height = 0.0f;   // 身体高度偏移量
        int   wheel_hold  = 0;      // 轮子指令保持计数（>0 表示最近有左右键）
        float wheel_stick = 0.0f;   // 轮子摇杆等效输入 [-1,1]
        const float dt = 1.0f / HZ;
        bool running = true;
        int  frame = 0;

        while (running) {
            // 一帧内消化所有已缓冲按键（raw 非阻塞，可能积压多个）
            bool got_wheel_key = false;
            KeyDir k;
            while ((k = poll_key()) != KeyDir::NONE) {
                switch (k) {
                    case KeyDir::UP:    body_height += HEIGHT_STEP; break;
                    case KeyDir::DOWN:  body_height -= HEIGHT_STEP; break;
                    case KeyDir::LEFT:  wheel_stick = -1.0f; got_wheel_key = true; break;  // 反转
                    case KeyDir::RIGHT: wheel_stick = +1.0f; got_wheel_key = true; break;  // 正转
                    case KeyDir::QUIT:  running = false; break;
                    default: break;
                }
            }
            body_height = clamp(body_height, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX);

            // 轮子：终端无“松开”事件，用超时判定停止。有左右键则续期保持计数
            if (got_wheel_key) wheel_hold = WHEEL_HOLD_FRAMES;
            if (wheel_hold > 0) { wheel_hold--; }
            else                { wheel_stick = 0.0f; }  // 超时无键 → 停

            // 轮子：速度模式下发目标角速度
            float wheel_speed = wheel_stick * WHEEL_SPEED;
            motor_mgr.SendSpeed(can_port, 4, wheel_speed, WHEEL_KVP, WHEEL_KVI);

            // 腿 IK：站立足端 + 高度偏移
            float foot_target_body[3] = {
                base_foot_body[leg][0],
                base_foot_body[leg][1],
                base_foot_body[leg][2] - body_height,
            };
            float R[3][3], Rt[3][3];
            hip_rotation_matrix(static_cast<LegIndex>(leg), R);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) Rt[i][j] = R[j][i];
            float mount_to_foot[3];
            for (int i = 0; i < 3; i++)
                mount_to_foot[i] = foot_target_body[i] - LEG_MOUNT[leg][i];
            float p_hip[3] = {0};
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) p_hip[i] += Rt[i][j] * mount_to_foot[j];

            float q_cmd[3];
            leg_ik(p_hip, LEG_L1, LEG_L2, LEG_L3,
                   THETA1_OFFSET, THETA2_OFFSET, THETA3_OFFSET, q_cmd);
            q_cmd[0] = clamp(q_cmd[0], deg2rad(LOWER_LIMIT_THETA1_DEG), deg2rad(UPPER_LIMIT_THETA1_DEG));
            q_cmd[1] = clamp(q_cmd[1], deg2rad(LOWER_LIMIT_THETA2_DEG), deg2rad(UPPER_LIMIT_THETA2_DEG));
            q_cmd[2] = clamp(q_cmd[2], deg2rad(LOWER_LIMIT_THETA3_DEG), deg2rad(UPPER_LIMIT_THETA3_DEG));
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(can_port, j + 1, q_cmd[j], 0.0f, KP, KD, 0.0f);

            if (frame % 50 == 0) {
                MotorStatus wst = motor_mgr.GetStatus(can_port, 4);
                printf("\r  高度=%+.3fm  轮 目标=%.2f 实测=%.2f rad/s  ",
                       body_height, wheel_speed, wst.velocity);
                fflush(stdout);
            }
            frame++;
            usleep(1000000 / HZ);
        }
    }
    // term 析构：恢复终端

    // ---- 清理 ----
    printf("\n[INFO] 正在关闭...\n");
    motor_mgr.SendImpedance(can_port, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // 轮零扭矩
    for (int mi = 1; mi <= 4; mi++) motor_mgr.DisableMotor(can_port, mi);

    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] 示例23 完成\n");
    fflush(stdout);
}
