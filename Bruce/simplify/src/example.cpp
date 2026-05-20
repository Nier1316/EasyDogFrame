#include "example.h"
#include "thread/thread_manager.h"
#include <thread>
#include <chrono>




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