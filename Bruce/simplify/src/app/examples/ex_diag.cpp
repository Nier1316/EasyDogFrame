// ===== 诊断/只读示例 24,26-29,33,34 =====
// 由原单文件 example.cpp 示例拆包而来（现拆到 src/app/examples/），公共 helper 见 app/examples_common.h
#include "app/examples/ex_diag.h"
#include "app/examples_common.h"
#include "transport/canet_transport.h"
#include "transport/usb2can_transport.h"
#include "motor/motor_manager.h"
#include "motor/motor_calibration.h"
#include "runtime/thread_manager.h"
#include "common/motor_logger.h"
#include "common/log_control.h"
#include "motion/robot_calibration.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>

using logctl::LogCat;
#include "strategy/imu_device.h"
#include "strategy/xbox_controller.h"
#include "strategy/rl_controller.h"
#include "motion/SimSync.h"
#include "motion/leg_kinematics.h"
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <algorithm>

void Example24_ReadMotorParams() {
    printf("\n========== 示例 24：只读固件参数诊断（不使能电机）==========\n");
    printf("[INFO] 全程不使能电机，可反复运行。\n");
    printf("[INFO] 重点：使能前的速度读数——判断 CAN1 轮电机的假速度\n");
    printf("       是否与使能动作无关（若上电即为非零，则属固件自身状态）。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    // 只启接收线程：发送线程会跳过未使能电机，本例也不需要它
    thread_mgr.start_thread("motor_receive");
    // 打开详细打印，让角度/速度/扭矩回帧也显示原始值
    g_param_verbose = true;
    sleep(1);

    // ---- 第一步：上电后立即读速度，这是本次诊断的核心判据 ----
    printf("---------- 关键判据：使能前的轮电机速度 (0x10) ----------\n");
    printf("[INFO] 连读 5 轮，观察是否有非零值及其衰减趋势\n");
    for (int round = 0; round < 5; round++) {
        printf("  --- 第 %d 轮 ---\n", round + 1);
        fflush(stdout);
        for (int cp = 0; cp < 4; cp++) {
            motor_mgr.ReadParam(cp, 4, MOTOR_OR_velocity);
            usleep(60000);
        }
        usleep(200000);
    }

    struct ParamItem { uint8_t type; const char* name; };
    const ParamItem items[] = {
        // 本次新增：定位 CAN1 差异
        { MOTOR_WR_Velfilter_constant,     "★角速度低通滤波常数" },
        { MOTOR_WR_Major,                  "★电机型号" },
        { MOTOR_OR_We,                     "★电角速度" },
        { MOTOR_OR_Iq,                     "★Q轴电流 A" },
        { MOTOR_OR_Te,                     "★电磁转矩 Nm" },
        { MOTOR_OR_Angel,                  "★电角度" },
        // 状态与量程
        { MOTOR_WR_CONTROL_MODE,           "控制模式(0阻抗/1速度/2位置)" },
        { MOTOR_OR_velocity,               "当前角速度 rad/s" },
        { MOTOR_OR_angle,                  "当前角度 rad" },
        { MOTOR_OR_torque,                 "当前扭矩 Nm" },
        { MOTOR_OR_error_register,         "错误寄存器" },
        { MOTOR_OR_error_history_register, "历史错误寄存器" },
        { MOTOR_OR_temperature,            "温度 C" },
        { MOTOR_WR_CAN_REPLY_MAX_Torque,   "量程 t_max Nm" },
        { MOTOR_WR_CAN_REPLY_MAX_Velocity, "量程 v_max rad/s" },
        { MOTOR_WR_CAN_REPLY_MAX_KP,       "量程 kp_max" },
        { MOTOR_WR_CAN_REPLY_MAX_KD,       "量程 kd_max" },
        { MOTOR_WR_Current_Limit,          "最大电流限制 A" },
        { MOTOR_WR_KT_OUT,                 "转矩系数 A/Nm" },
        { MOTOR_WR_GR,                     "减速比" },
        { MOTOR_WR_J,                      "转动惯量" },
        { MOTOR_WR_B,                      "粘滞系数" },
        { MOTOR_WR_Tf,                     "静摩擦力矩 Nm" },
        { MOTOR_WR_CAN_Timeout,            "CAN 超时周期数" },
    };
    const int n_items = sizeof(items) / sizeof(items[0]);

    // ---- 第二步：轮电机四路横向对比，★项用于定位 CAN1 差异 ----
    printf("\n---------- 轮电机 (motor_id=4) 四路对比 ----------\n");
    for (int i = 0; i < n_items; i++) {
        printf("\n>>> %s (type=0x%02X)\n", items[i].name, items[i].type);
        fflush(stdout);
        for (int cp = 0; cp < 4; cp++) {
            motor_mgr.ReadParam(cp, 4, items[i].type);
            usleep(80000);   // 避免多路输出交织
        }
    }

    // ---- 第三步：CAN0 关节电机作参照 ----
    printf("\n---------- 参照：CAN0 关节电机 ----------\n");
    for (int mi = 1; mi <= 3; mi++) {
        printf("\n>>> CAN0 motor%d\n", mi);
        fflush(stdout);
        for (int i = 0; i < n_items; i++) {
            motor_mgr.ReadParam(0, mi, items[i].type);
            usleep(80000);
        }
    }

    g_param_verbose = false;
    printf("\n[INFO] 读取完毕。核对要点：\n");
    printf("  1) 第一步里 CAN1 轮速度若非零 → 假速度与使能无关，属固件状态\n");
    printf("  2) ★滤波常数(0x5C) 四路是否相同 → 不同则解释 tau≈13.3ms 的衰减\n");
    printf("  3) ★型号(0x50) 四路是否相同 → 不同说明硬件批次差异\n");
    printf("  4) 量程各项与 MOTOR_LIMITS（ele_motor_def.h）是否一致\n");

    thread_mgr.stop_thread("motor_receive");
    motor_mgr.Stop();
    printf("\n[INFO] 示例24 完成（未使能任何电机）\n");
    fflush(stdout);
}

// ================= 示例 26：键盘输入接收测试（纯诊断，不碰电机） =================
// 验证当前运行环境（集成终端/调试器）能否收到键盘输入（行模式 + raw 模式两段）。
void Example26_KeyboardInputTest() {
    printf("\n========== 示例 26：键盘输入接收测试 ==========\n");
    printf("[INFO] 纯诊断，不初始化 CAN、不使能电机。\n");

    // ---- 环境诊断 ----
    printf("\n--- 环境诊断 ---\n");
    printf("[INFO] isatty(stdin)=%d   (1=是终端, 0=被重定向/管道)\n", isatty(STDIN_FILENO));
    struct termios check_tio;
    printf("[INFO] tcgetattr(stdin)=%d  (0=成功, 即可以进 raw 模式)\n",
           tcgetattr(STDIN_FILENO, &check_tio));
    printf("[提示] 调试器 Debug Console 通常无法做 raw/行模式交互；\n");
    printf("       若 isatty=1 却收不到键，请改用集成终端运行。\n\n");
    fflush(stdout);

    // ---- 阶段1：行模式输入（复刻 Example23 选路逻辑） ----
    printf("--- 阶段1：行模式输入，请输入 0~3，3 秒无输入则超时 ---\n");
    struct pollfd pfd;
    pfd.fd = fileno(stdin);
    pfd.events = POLLIN;
    pfd.revents = 0;
    printf("> ");
    fflush(stdout);
    int pr = poll(&pfd, 1, 3000);
    int val = -1;
    bool got = false;
    if (pr > 0 && (pfd.revents & POLLIN)) {
        if (scanf("%d", &val) == 1) got = true;
        else {
            // 有数据但不是数字：把残渣清掉并报告，便于判断收到的是啥
            int c; while ((c = getchar()) != '\n' && c != EOF) {}
        }
    }
    printf("  poll 返回=%d %s | 读到数字=%s | 值=%d\n",
           pr, pr > 0 ? "(有数据)" : (pr == 0 ? "(超时)" : "(错误)"),
           got ? "是" : "否", got ? val : -1);
    if (!got)
        printf("[WARN] 阶段1 收不到输入：若在调试器里这是预期的，请用集成终端。\n");
    printf("[INFO] 阶段1 完成。\n\n");
    fflush(stdout);

    // ---- 阶段2：raw 模式方向键（复刻 Example23 控制循环） ----
    printf("--- 阶段2：raw 模式方向键 ↑↓←→ / q 退出，10 秒窗口 ---\n");
    RawTerminal term;
    if (!term.ok) {
        printf("[ERROR] 无法进入 raw 模式（stdin 不是可交互终端？）。\n");
        printf("[INFO] 请在集成终端里运行本示例。\n");
        return;
    }
    printf("[INFO] raw 模式已就绪，请按方向键（每 2 秒打印一次心跳）：\n");
    fflush(stdout);

    auto t0 = std::chrono::steady_clock::now();
    int key_count = 0;
    bool quit = false;
    while (!quit) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count() / 1000.0;
        if (t >= 10.0) { printf("\n[INFO] 10 秒超时。\n"); break; }

        KeyDir k;
        while ((k = poll_key()) != KeyDir::NONE) {
            key_count++;
            const char* name = (k == KeyDir::UP)    ? "↑ UP" :
                               (k == KeyDir::DOWN)  ? "↓ DOWN" :
                               (k == KeyDir::LEFT)  ? "← LEFT" :
                               (k == KeyDir::RIGHT) ? "→ RIGHT" : "q QUIT";
            printf("  [t=%6.2fs] 按键: %s\n", t, name);
            fflush(stdout);
            if (k == KeyDir::QUIT) { quit = true; break; }
        }
        // 心跳：每 2 秒打一行，证明循环活着、在等输入
        if (key_count == 0 && fmod(t, 2.0) < 0.05) {
            printf("  [t=%6.2fs] 等待输入中...（按键不生效？检查终端焦点）\n", t);
            fflush(stdout);
        }
        usleep(50000);
    }

    if (key_count > 0) {
        printf("[INFO] 阶段2 共收到 %d 个按键，raw 模式输入正常。\n", key_count);
    } else {
        printf("[WARN] 阶段2 没收到任何按键：raw 模式输入不可用。\n");
        printf("[INFO] 请确认在集成终端（Run Program 任务）中运行，并让窗口聚焦。\n");
    }
    // term 析构自动恢复终端

    printf("\n[INFO] 示例26 完成。\n");
    fflush(stdout);
}

// ================= 示例 27：CANET 接收频率探针（不使能电机，纯读取测频） =================
// 目的：验证"CANET 性能不差于 USB2CAN"的假设。此前日志显示控制闭环只有 ~25Hz
// （16 电机，每帧 VCI_Transmit 阻塞 ~10ms）。本示例把一切"主动动作"去掉：
//   不使能电机、不启发送线程、不下发控制帧，只测 CANET 设备本身的读写能力，
// 用于定位那 40ms 到底在设备侧还是在我们自己的收发架构：
//   阶段1  VCI_Receive(timeout=1ms) 空轮询耗时 —— SDK 是否真按 1ms 超时返回
//   阶段2  单条读命令往返延迟（发读参数帧 → 收到回帧）
//   阶段3  连续读请求吞吐（单电机持续往返 / 全 16 电机并发读）
// 对照基准：USB2CAN 直连通常单命令往返 1~2ms，12 电机可到 500Hz+。
void Example27_CANetFrequencyProbe() {
    printf("\n========== 示例 27：CANET 接收频率探针 ==========\n");
    printf("[INFO] 不使能电机、不启发送线程，只测 CANET 设备读写能力。\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    g_param_verbose = false;
    printf("[INFO] 4 路 CANET 连接已建立（未启动发送线程、未使能电机）。\n\n");

    // 读参数帧（复刻 float2bag 的读路径：RW=0，参数值 0）。
    // 发送 tx_id = motor_id；电机回应帧 rx_id = 50 + motor_id
    auto send_read = [](int can, int motor_id, uint8_t type) {
        uint8_t f[8] = {0x80, 0, 0, 0, 0, 0x00, type, 0xEC};
        CanetTransport::GetInstance().Can_Tx(can, motor_id, f, 8);
    };
    // 排空某路积压帧，避免上阶段残留污染测量起点
    auto drain = [](int can) {
        std::vector<BspCanFrame> frames;
        for (int i = 0; i < 20; i++) CanetTransport::GetInstance().ReceiveFrames(can, frames, 1);
    };
    using clk = std::chrono::steady_clock;
    auto ms_since = [](clk::time_point t0) {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            clk::now() - t0).count() / 1000.0;
    };

    // ---- 阶段1：VCI_Receive 空轮询耗时 ----
    printf("--- 阶段1：VCI_Receive(timeout=1ms) 空轮询耗时（无任何帧在途）---\n");
    {
        const int N = 100;
        std::vector<double> ms;
        for (int i = 0; i < N; i++) {
            auto t0 = clk::now();
            std::vector<BspCanFrame> frames;
            CanetTransport::GetInstance().ReceiveFrames(0, frames, 1);
            ms.push_back(ms_since(t0));
        }
        std::sort(ms.begin(), ms.end());
        printf("  单次空轮询: min=%.2fms  p50=%.2fms  p90=%.2fms  max=%.2fms\n",
               ms.front(), ms[N / 2], ms[int(N * 0.9)], ms.back());
        printf("  %s\n", ms[N / 2] > 5.0
            ? "  [判读] SDK 空等也阻塞 >5ms → 接收线程会长时间占设备锁，拖慢发送"
            : "  [判读] SDK 尊重 1ms 超时 → 空轮询不是瓶颈，40ms 另有来源");
    }

    // ---- 阶段2：单条读命令往返延迟 ----
    printf("\n--- 阶段2：单条读命令往返延迟（CANx 轮电机 motor4，各 3 次，超时 100ms）---\n");
    printf("  %-5s %-9s %-9s %-9s %s\n", "CAN", "min(ms)", "avg(ms)", "max(ms)", "超时");
    for (int can = 0; can < 4; can++) {
        drain(can);
        std::vector<double> lats;
        int timeout_cnt = 0;
        for (int rep = 0; rep < 3; rep++) {
            auto t0 = clk::now();
            send_read(can, 4, MOTOR_OR_velocity);
            bool got = false;
            while (ms_since(t0) < 100.0) {
                std::vector<BspCanFrame> frames;
                if (CanetTransport::GetInstance().ReceiveFrames(can, frames, 1)) {
                    for (auto& f : frames) if (f.id == 50 + 4) { got = true; break; }
                }
                if (got) break;
            }
            if (got) lats.push_back(ms_since(t0));
            else     timeout_cnt++;
        }
        double sum = 0; for (double v : lats) sum += v;
        double mn = lats.empty() ? -1 : *std::min_element(lats.begin(), lats.end());
        double mx = lats.empty() ? -1 : *std::max_element(lats.begin(), lats.end());
        printf("  %-5d %-9.2f %-9.2f %-9.2f %d/%d\n",
               can, mn, lats.empty() ? -1 : sum / lats.size(), mx, timeout_cnt, 3);
    }

    // ---- 阶段3a：单电机持续往返速率 ----
    printf("\n--- 阶段3a：单电机持续往返速率（CAN0 motor1，2s 请求-回应 ping-pong）---\n");
    {
        drain(0);
        auto t_end = clk::now() + std::chrono::milliseconds(2000);
        int n = 0, miss = 0;
        while (clk::now() < t_end) {
            send_read(0, 1, MOTOR_OR_velocity);
            auto t0 = clk::now();
            bool got = false;
            while (ms_since(t0) < 50.0) {
                std::vector<BspCanFrame> frames;
                if (CanetTransport::GetInstance().ReceiveFrames(0, frames, 1)) {
                    for (auto& f : frames) if (f.id == 51) { got = true; break; }
                }
                if (got) break;
            }
            if (got) n++; else miss++;
        }
        printf("  成功 %d 次，超时 %d 次 → %.0f 次/s\n", n, miss, n / 2.0);
    }

    // ---- 阶段3b：全 16 电机并发读吞吐 ----
    printf("\n--- 阶段3b：全 16 电机并发读吞吐（2s，每轮 16 条读 + 收回应）---\n");
    {
        for (int can = 0; can < 4; can++) drain(can);
        auto t_end = clk::now() + std::chrono::milliseconds(2000);
        int total = 0;
        while (clk::now() < t_end) {
            for (int can = 0; can < 4; can++)
                for (int id = 1; id <= 4; id++)
                    send_read(can, id, MOTOR_OR_velocity);
            for (int can = 0; can < 4; can++) {
                while (clk::now() < t_end) {
                    std::vector<BspCanFrame> frames;
                    if (CanetTransport::GetInstance().ReceiveFrames(can, frames, 1)) {
                        for (auto& f : frames) if (f.id >= 51 && f.id <= 54) total++;
                    } else break;
                }
            }
        }
        printf("  收到 %d 帧 / 2.0s = %.0f 帧/s（对照：16 电机跑 500Hz 需 8000 帧/s）\n",
               total, total / 2.0);
    }

    // ---- 收尾 ----
    printf("\n[INFO] 判读指引：\n");
    printf("  阶段1 p50>5ms → SDK 空等也阻塞，收发架构要先改（分离锁/批量）\n");
    printf("  阶段2 往返 ~2ms + 阶段3a 单电机 >400Hz → 设备本身不慢，瓶颈在我们的发送线程\n");
    printf("  阶段2 往返 ~10ms + 阶段3b 全 16 电机 <1000帧/s → CANET 设备串行处理，需换直连 CAN 卡\n");

    motor_mgr.Stop();
    printf("\n[INFO] 示例27 完成（未使能任何电机）。\n");
    fflush(stdout);
}

// ================= 示例 28：CANET 批量发送探针（电机下电，不使能电机） =================
// 目的：在零风险（电机已下电）下量化"批量发送 vs 逐帧发送"的耗时差，
// 验证 VCI_Transmit 一次传多帧能否绕开 SDK 的 ~10ms 逐帧地板。
// 阶段1 单帧发送耗时  |  阶段2 批量4帧发送耗时  |  阶段3 完整一轮（批量4发+捞接收）
// 阶段4 确认电机下电（捞接收应无帧）
void Example28_CANetBatchProbe() {
    printf("\n========== 示例 28：CANET 批量发送探针 ==========\n");
    printf("[INFO] 电机下电、CANET 供电；不使能电机，只测发送路径时序。\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败（某路 CANET 打开失败）\n");
        return;
    }
    // 不启动任何线程，主线程直接操作 CanetTransport
    CanetTransport& bsp = CanetTransport::GetInstance();

    using clk = std::chrono::steady_clock;
    auto ms_since = [](clk::time_point t0) {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            clk::now() - t0).count() / 1000.0;
    };
    auto report = [](const char* label, std::vector<double>& ms) {
        std::sort(ms.begin(), ms.end());
        int n = (int)ms.size();
        printf("  %-28s min=%6.2f  p50=%6.2f  p90=%6.2f  max=%6.2f ms\n",
               label, ms.front(), ms[n / 2], ms[int(n * 0.9)], ms.back());
    };

    // 读参数帧（RW=0），CAN ID=motor_id；电机下电不会回应，仅用于测发送路径
    auto build_read = [](int motor_id) {
        BspCanFrame f;
        f.id = (uint32_t)motor_id;
        f.dlc = 8;
        f.is_extended = 0;
        uint8_t d[8] = {0x80, 0, 0, 0, 0, 0x00, MOTOR_OR_velocity, 0xEC};
        memcpy(f.data, d, 8);
        return f;
    };

    // ---- 阶段1：单帧发送耗时 ----
    printf("\n--- 阶段1：单帧 VCI_Transmit 耗时（CAN0，N=50）---\n");
    {
        std::vector<double> ms;
        for (int i = 0; i < 50; i++) {
            BspCanFrame f = build_read(1);
            auto t0 = clk::now();
            bsp.Can_Tx(0, f.id, f.data, f.dlc);
            ms.push_back(ms_since(t0));
        }
        report("单帧发送", ms);
    }

    // ---- 阶段2：批量4帧发送耗时 ----
    printf("\n--- 阶段2：批量 4 帧 VCI_Transmit 耗时（CAN0，N=50）---\n");
    {
        std::vector<BspCanFrame> batch;
        for (int id = 1; id <= 4; id++) batch.push_back(build_read(id));
        std::vector<double> ms;
        for (int i = 0; i < 50; i++) {
            auto t0 = clk::now();
            bsp.SendFramesBatch(0, batch);
            ms.push_back(ms_since(t0));
        }
        report("批量4帧发送", ms);
    }

    // ---- 阶段3：完整一轮（批量4发 + 捞一次接收）----
    printf("\n--- 阶段3：完整一轮 批量4发 + 捞接收（N=50）---\n");
    {
        std::vector<BspCanFrame> batch;
        for (int id = 1; id <= 4; id++) batch.push_back(build_read(id));
        std::vector<double> ms;
        int rx_total = 0;
        for (int i = 0; i < 50; i++) {
            auto t0 = clk::now();
            bsp.SendFramesBatch(0, batch);
            std::vector<BspCanFrame> rx;
            if (bsp.ReceiveFrames(0, rx, 1)) rx_total += (int)rx.size();
            ms.push_back(ms_since(t0));
        }
        report("批量4发+捞接收", ms);
        printf("  捞到 %d 帧（电机下电期望≈0）\n", rx_total);
    }

    // ---- 阶段4：确认电机下电（多次捞接收应无帧）----
    printf("\n--- 阶段4：电机下电核对（连续捞 20 次）---\n");
    {
        int got = 0;
        for (int i = 0; i < 20; i++) {
            std::vector<BspCanFrame> rx;
            if (bsp.ReceiveFrames(0, rx, 1)) got += (int)rx.size();
        }
        printf("  捞到 %d 帧 → %s\n", got,
               got == 0 ? "[OK] 电机已下电，CAN 上无活动节点" :
                          "[!] 仍有帧返回，注意核对是否真的下电");
    }

    // ---- 阶段5：4 路打开状态 ----
    printf("\n--- 阶段5：4 路 CANET 打开状态 ---\n");
    printf("[INFO] 能走到这里说明 MotorManager::Initialize 已成功打开全部 4 路\n");
    printf("[INFO] （若 4004 端口真有问题，上面会直接失败退出）\n");

    // ---- 阶段6：SDK 并发能力（发送能否与阻塞接收并行）----
    // 直接调 VCI_*（绕过 CanetTransport/CanDevice 的锁），测 SDK 本身是否允许
    // 一个线程持续 VCI_Transmit 的同时另一个线程在 VCI_Receive 里阻塞 10ms。
    // 若发送保持 ~0.01ms → SDK 支持并发 → 分离收发锁即可根治 40ms；
    // 若发送被拖到 ~10ms → SDK 内部串行 → 只能靠批量压缩调用次数。
    printf("\n--- 阶段6：SDK 并发能力（发线程 100 次批量发 + 收线程 100 次阻塞收并行）---\n");
    {
        VCI_CAN_OBJ batch[4];
        for (int i = 0; i < 4; i++) {
            batch[i].ID = (UINT)(i + 1);
            batch[i].DataLen = 8;
            batch[i].ExternFlag = 0;
            batch[i].RemoteFlag = 0;
            batch[i].SendType = 0;
            memset(batch[i].Data, 0, 8);
            batch[i].Data[0] = 0x80;
            batch[i].Data[6] = MOTOR_OR_velocity;
            batch[i].Data[7] = 0xEC;
        }
        long send_us_sum = 0;
        int send_cnt = 0;
        std::thread send_th([&]() {
            for (int i = 0; i < 100; i++) {
                auto t0 = clk::now();
                VCI_Transmit(VCI_CANETE, 0, 0, batch, 4);
                send_us_sum += (long)(ms_since(t0) * 1000.0);
                send_cnt++;
            }
        });
        std::thread recv_th([&]() {
            VCI_CAN_OBJ buf[100];
            for (int i = 0; i < 100; i++)
                VCI_Receive(VCI_CANETE, 0, 0, buf, 100, 1);
        });
        send_th.join();
        recv_th.join();
        double avg = send_cnt ? (double)send_us_sum / send_cnt / 1000.0 : -1.0;
        printf("  并行期间 单次批量发送平均 %.3f ms（阶段2 串行时 0.01ms）\n", avg);
        printf("  %s\n", avg < 2.0
            ? "  [OK] SDK 允许发送与接收并行 → 分离收发锁即可根治发送被阻塞"
            : "  [判读] SDK 内部串行收发 → 只能靠批量压缩 SDK 调用次数");
    }

    // ---- 阶段7：包装路径并发验证（拆锁后 CanetTransport 收发是否仍被串行）----
    // 走正式的 CanetTransport::SendFramesBatch / ReceiveFrames（含我们自己的锁），
    // 验证拆锁后发送线程不再被接收线程的 10ms 阻塞卡住。
    printf("\n--- 阶段7：包装路径并发（CanetTransport 批量发 vs 阻塞收 并行，拆锁验证）---\n");
    {
        std::vector<BspCanFrame> batch;
        for (int id = 1; id <= 4; id++) batch.push_back(build_read(id));
        long send_us_sum = 0;
        int send_cnt = 0;
        std::thread send_th([&]() {
            for (int i = 0; i < 100; i++) {
                auto t0 = clk::now();
                bsp.SendFramesBatch(0, batch);
                send_us_sum += (long)(ms_since(t0) * 1000.0);
                send_cnt++;
            }
        });
        std::thread recv_th([&]() {
            std::vector<BspCanFrame> rx;
            for (int i = 0; i < 100; i++)
                bsp.ReceiveFrames(0, rx, 1);
        });
        send_th.join();
        recv_th.join();
        double avg = send_cnt ? (double)send_us_sum / send_cnt / 1000.0 : -1.0;
        printf("  并行期间 CanetTransport 批量发送平均 %.3f ms\n", avg);
        printf("  %s\n", avg < 2.0
            ? "  [OK] 拆锁生效：CanetTransport 发送不再被接收阻塞（40ms→~10ms 的根源已消除）"
            : "  [!] CanetTransport 仍串行，需继续排查");
    }

    // ---- 阶段8：VCI_Receive timeout 参数扫描（定位 10ms 地板的机制）----
    // 空轮询下改 timeout 看实际返回时间。判读：
    //   各 timeout 都 ~10ms 返回  → SDK 内部写死 10ms 等待，无视 timeout（硬地板）
    //   小 timeout 被抬到 ~10ms、大 timeout 随参数走 → SDK 按 10ms 粒度轮询
    //   timeout=1 就 ~1ms 返回    → 之前 10ms 另有来源（设备/网络）
    printf("\n--- 阶段8：VCI_Receive timeout 参数扫描（空轮询，定位 10ms 地板机制）---\n");
    {
        int timeouts[] = {1, 5, 10, 20, 50, 100};
        printf("  %-10s %-12s %-12s\n", "timeout", "实际返回(ms)", "判读");
        for (int t : timeouts) {
            auto t0 = clk::now();
            VCI_CAN_OBJ buf[100];
            VCI_Receive(VCI_CANETE, 0, 0, buf, 100, t);
            double r = ms_since(t0);
            const char* tag;
            if (r < 2.0)            tag = "≈timeout，无地板";
            else if (r < t * 0.9)   tag = "被 ~10ms 下限抬升";
            else if (r <= t + 1.0)  tag = "随 timeout 走";
            else                    tag = ">timeout，异常";
            printf("  %-10d %-12.2f %s\n", t, r, tag);
        }
    }

    motor_mgr.Stop();
    printf("\n[INFO] 示例28 完成（未使能任何电机）。\n");
    fflush(stdout);
}

// ================= 示例 29：控制环频率测试（拆锁后） =================
// 目的：端到端验证"收发分离锁"改造是否把控制环从 40ms 拉回 ~10ms。
// 做法：启动接收+发送线程，使能 CAN1 的 4 个电机（电机可下电：无反馈无扭矩，
//       仅时序测试），主循环连续发 100 帧阻抗指令并测实际帧间隔。
// 判读：拆锁前主循环被发送线程抢锁拖到 ~40ms；拆锁后应回到 ~10ms（usleep 值）。
void Example29_MainLoopCadenceTest() {
    printf("\n========== 示例 29：控制环频率测试（拆锁后）==========\n");
    printf("[INFO] 使能 CAN1 4 电机（电机可下电：无反馈无扭矩），仅测时序。\n");
    printf("[INFO] 预期：主循环 ~10ms/帧（拆锁前实测 40ms）。\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 写阻抗模式 + 使能（电机下电时无回应，但帧会发上总线；kp=kd=0 即使电机有电也安全）
    printf("[INFO] 写阻抗模式 + 使能 CAN1 电机...\n");
    for (int mi = 1; mi <= 4; mi++) motor_mgr.SetControlMode(1, mi, IMPEDANCE);
    usleep(200000);
    for (int mi = 1; mi <= 4; mi++) motor_mgr.EnableMotor(1, mi);
    usleep(200000);

    using clk = std::chrono::steady_clock;
    const int FRAMES = 100;
    std::vector<double> gaps;
    auto prev = clk::now();
    for (int f = 0; f < FRAMES; f++) {
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.SendImpedance(1, mi, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // 零增益零扭矩
        auto now = clk::now();
        gaps.push_back(std::chrono::duration_cast<std::chrono::microseconds>(now - prev).count() / 1000.0);
        prev = now;
        usleep(10000);
    }
    std::sort(gaps.begin(), gaps.end());
    int n = (int)gaps.size();
    printf("\n  主循环帧间隔: min=%.2f  p50=%.2f  p90=%.2f  max=%.2f ms（usleep=10ms）\n",
           gaps.front(), gaps[n / 2], gaps[int(n * 0.9)], gaps.back());
    printf("  %s\n", gaps[n / 2] < 25.0
        ? "  [OK] 主循环回到 ~10ms（拆锁前 40ms）→ 响应性问题解除"
        : "  [!] 仍 >25ms → 锁之外还有别的阻塞，需继续排查");

    // 停线程 + 清理
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("\n[INFO] 示例29 完成（log/ 下 sendcan CSV 可进一步核对每电机命令帧间隔）。\n");
    fflush(stdout);
}

// ================= 示例 30：RL 策略链路离线验证（不碰 CAN） =================
// 用导出工具生成的 REF_OBS/REF_ACTION 参考数据验证 MLP 权重与推理正确性，
// 以及观测构建与 sim2sim.py 的一致性。纯 CPU 计算，不初始化 CAN、不使能电机。
void Example33_IMUCheck() {
    printf("\n========== 示例 33：IMU 链路验证 ==========\n");
    printf("[INFO] 只读 IMU，不初始化 CAN、不使能电机。\n");
    printf("[INFO] 开机时请保持机身水平（IMU 水平校准基准）。\n\n");

    ImuDevice imu;
    imu.SetMount(ImuMount::Z_DOWN_X);   // 与 Example25 一致的实际安装方向
    const char* imu_port = "/dev/ttyUSB0";
    if (!imu.Initialize(imu_port, 115200)) {
        printf("[ERROR] IMU 打开失败 %s\n", imu_port);
        return;
    }
    printf("[INFO] IMU 已连接。60s 窗口，1s 刷新；把机器狗放平并绕各轴缓慢倾斜。\n\n");
    printf("  t(s)  |    gx      gy      gz (rad/s) |  roll  pitch   yaw (deg) |  pgr_x   pgr_y   pgr_z |    q0\n");
    printf("  --------------------------------------------------------------------------------------------\n");
    fflush(stdout);

    using clk = std::chrono::steady_clock;
    auto t0 = clk::now();
    int last = -1;
    while (true) {
        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
            clk::now() - t0).count();
        if (elapsed >= 60000) break;
        int sec = elapsed / 1000;
        if (sec == last) { usleep(50000); continue; }
        last = sec;

        float gx, gy, gz;
        float quat[4];
        imu.GetGyro(gx, gy, gz);
        imu.GetQuat(quat[0], quat[1], quat[2], quat[3]);
        const float w = quat[0], x = quat[1], y = quat[2], z = quat[3];

        // 欧拉角（Z-Y-X 顺序），供直观判读
        float roll  = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        float pitch = asinf(2 * (w * y - z * x));
        float yaw   = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

        float pgr[3];
        const float down[3] = {0.0f, 0.0f, -1.0f};
        rl::world2self(quat, down, pgr);   // ⚠ quat 必须是连续数组，勿用 &w（w 是独立局部变量，&w 非数组）

        printf("  %4d  | %7.3f %8.3f %8.3f | %6.1f %6.1f %6.1f | %7.3f %7.3f %7.3f | %6.3f\n",
               sec, gx, gy, gz, rad2deg(roll), rad2deg(pitch), rad2deg(yaw),
               pgr[0], pgr[1], pgr[2], w);
        fflush(stdout);
    }

    imu.Shutdown();
    printf("\n[INFO] 示例33 完成。判读要点：\n");
    printf("  1) 水平放平静止：pgr≈(0,0,-1)，欧拉角≈0，gyro≈0，q0≈1。\n");
    printf("  2) 前倾 pitch>0 → pgr.x 变正；右倾 roll>0 → pgr.y 变负（MuJoCo 约定）。\n");
    printf("  3) gyro 仅在转动时非零。\n");
    printf("  4) 放平但 pgr 明显偏离 (0,0,-1) → IMU 安装方向或水平校准有问题，先不上 RL。\n");
    fflush(stdout);
}

// ================= 示例 34：轮子扭矩方向测向 =================
// 目的：验证 MOTOR_CALIBRATION 轮子 pos_scale（扭矩下发方向）是否正确。
//   RL 轮子走 wheel_torque 扭矩前馈（下发经 pos_scale 翻转），与速度环（vel_scale）独立。
//   2026-08-21 实测已把 CAN0/2 轮子 pos_scale 修正为 +1，本示例用于复核/复测。
// 流程：使能 16 电机 → 10s 起立（四腿支撑、轮子悬空）→ +1.0 Nm 测向 3s
//       → 停 0.5s → -1.0 Nm 测向 3s → 10s 回位 → 失能。
// 判读：
//   +1.0 Nm → vel_can>0（向前）：固件正扭矩=前滚，pos_scale=+1 正确（策略正扭矩=前滚）。
//   +1.0 Nm → vel_can<0（向后）：pos_scale 方向反，需再翻转该路。
//   四路转向应一致。
void Example34_WheelDirectionCheck() {
    printf("\n========== 示例 34：轮子扭矩方向测向 ==========\n");
    printf("[WARN] 将使能 16 电机并起立！轮子会转动（±1.0 Nm）。\n");
    printf("       请确保机器平稳可支撑、手远离关节和轮子。Ctrl+C 急停。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    signal(SIGINT, rl_signal_handler);       // Ctrl+C 急停
    g_rl_stop = 0;

    using clk = std::chrono::steady_clock;

    // 使能 16 电机（腿支撑 + 轮测向）
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
    usleep(200000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.EnableMotor(cp, mi);
    usleep(300000);

    // 读当前腿位置（作为回位目标）
    float start_pos[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            start_pos[leg * 3 + j] = motor_mgr.GetStatus(leg, j + 1).position;

    // 起立目标：STAND_*（真机实测站立指令角），四腿支撑、轮子悬空
    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }

    auto send_leg = [&](const float* q) {
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1, q[leg * 3 + j], 0.0f, 200.0f, 20.0f, 0.0f);
    };

    // 10s 慢插值起立（参考 Example25，轮子 0 扭矩）
    printf("[INFO] 起立中（10s，四腿支撑、轮子悬空）...\n");
    const int STAND_FRAMES = 500;
    for (int f = 0; f <= STAND_FRAMES && !g_rl_stop; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1,
                    start_pos[leg * 3 + j] + (stand_q[leg * 3 + j] - start_pos[leg * 3 + j]) * t,
                    0.0f, 200.0f, 20.0f, 0.0f);
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);   // 轮 0 扭矩
        if (f % 50 == 0) { printf("\r  [%3.0f%%] ", t * 100); fflush(stdout); }
        usleep(20000);   // 50Hz
    }
    printf("\n[INFO] 起立完成，轮子应悬空。开始 ±1.0 Nm 扭矩测向。\n");

    auto show_vel = [&]() {
        printf("%+6.3f %+6.3f %+6.3f %+6.3f  ",
               motor_mgr.GetStatus(0, 4).velocity,
               motor_mgr.GetStatus(1, 4).velocity,
               motor_mgr.GetStatus(2, 4).velocity,
               motor_mgr.GetStatus(3, 4).velocity);
    };
    auto send_tau = [&](float tau) {
        send_leg(stand_q);   // 腿保持站立
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0, 0, 0, 0, tau);
    };

    // +1.0 Nm：若固件正扭矩=前滚，轮子应向前（vel_can>0）转
    printf("[+1.0 Nm] 观察轮子转向（向前=vel_can>0，向后=vel_can<0）:\n");
    auto tq0 = clk::now();
    while (!g_rl_stop) {
        int el = (int)std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - tq0).count();
        if (el >= 3000) break;
        float tau = 1.0f * std::min(1.0f, (float)el / 1000.0f);   // 1s 渐变到 1.0
        send_tau(tau);
        if (el % 500 < 50) { printf("\r  t=%3.0fms tau=%+.2f | FL", (float)el, tau); show_vel(); fflush(stdout); }
        usleep(20000);
    }
    printf("\n");
    send_tau(0.0f);
    usleep(500000);

    // -1.0 Nm：应反向
    printf("[-1.0 Nm] 观察轮子转向（应反向）:\n");
    tq0 = clk::now();
    while (!g_rl_stop) {
        int el = (int)std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - tq0).count();
        if (el >= 3000) break;
        float tau = -1.0f * std::min(1.0f, (float)el / 1000.0f);
        send_tau(tau);
        if (el % 500 < 50) { printf("\r  t=%3.0fms tau=%+.2f | FL", (float)el, tau); show_vel(); fflush(stdout); }
        usleep(20000);
    }
    printf("\n");

    // 缓慢插值回到初始姿态（10s），轮子 0 扭矩
    printf("[INFO] 缓慢放下腿回初始姿态（10s）...\n");
    for (int f = 0; f <= STAND_FRAMES && !g_rl_stop; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++)
                motor_mgr.SendImpedance(leg, j + 1,
                    stand_q[leg * 3 + j] + (start_pos[leg * 3 + j] - stand_q[leg * 3 + j]) * t,
                    0.0f, 200.0f, 20.0f, 0.0f);
        for (int cp = 0; cp < 4; cp++)
            motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        if (f % 50 == 0) { printf("\r  [%3.0f%%] ", t * 100); fflush(stdout); }
        usleep(20000);
    }
    printf("\n[INFO] 已回位。\n");

    // 失能 16 电机
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.DisableMotor(cp, mi);
    signal(SIGINT, SIG_DFL);

    printf("\n[判读] 各轮在 +1.0 Nm 时的转向：\n");
    printf("  +1.0 Nm → vel_can>0（向前）：固件正扭矩=前滚，pos_scale=+1 正确（策略正扭矩=前滚）。\n");
    printf("  +1.0 Nm → vel_can<0（向后）：pos_scale 方向反，需再翻转该路。\n");
    printf("  四路转向应一致；若个别相反，只改那一路。\n");

    thread_mgr.stop_thread("motor_send");
    thread_mgr.stop_thread("motor_receive");
    motor_mgr.Stop();
    printf("\n[INFO] 示例34 完成。\n");
    fflush(stdout);
}

// ================= 示例 35：轮电机前馈标定 =================
// 目的：测量四个轮电机在悬空状态下"恰好克服静摩擦开始转动"的正/负扭矩前馈值，
//   用于 wheel_torque 的摩擦前馈补偿：tau = KD*(VEL_SCALE*a - vel) + tau_ff。
// 流程：记录初始腿位置 → 5s 起立（四腿支撑、轮子悬空）→ 逐电机标定（CAN0→CAN3）：
//   先测正扭矩：↑/↓ 从 0 以 0.1 梯度调（正阶段 clamp≥0），轮子恰好转动时按回车记录；
//   再测负扭矩：同样从 0 往下调（负阶段 clamp≤0），回车记录。
// → 4 电机测完打印每路正/负前馈值 → 5s 回位 → 失能。
// 注：需从集成终端运行（stdin 为真实终端）；Ctrl+Q 提前退出（回位+失能）。

// ================= 示例 39：USB2CAN 传输链路验证 =================
// 走 CanTransport 接口直接测达妙 USB2CAN（不经过 MotorManager/CANET）：
//   open ttyACM0 → 发读电机参数帧 → 收反馈。
// 验证「换硬件不动上层」：上层只需 CanTransport*，本示例即 Usb2CanTransport。
// 注：MotorManager::SetChannelTransport(1, &Usb2CanTransport::GetInstance())
//     可实现 CAN1 换 USB2CAN、其余保持 CANET（真机 RL 验证时用）。
void Example39_Usb2CanProbe() {
    printf("\n========== Example 39: USB2CAN 传输链路验证（官方 SDK） ==========\n");
    printf("[WARN] 达妙 USB2FDCAN 接 USB（udev 0666 已配），CAN_H/L 接 CAN1 那路电机总线。\n");
    printf("       波特率默认 1Mbps（设备预设）；电机总线一致才通。\n\n");

    // 走 CanTransport 接口（官方 SDK 版 Usb2CanTransport）——等价换硬件后上层用法
    Usb2CanTransport& u2c = Usb2CanTransport::GetInstance();
    TransportConfig cfg;
    cfg.device_idx = 1;      // 逻辑 CAN1
    cfg.usb_baud   = 0;      // 0 = 用设备默认（实测 1Mbps）
    if (!u2c.open(1, cfg)) {
        printf("[ERROR] Usb2Can open 失败（看上方 Usb2Can 错误信息）\n");
        return;
    }

    // 发读 CAN1 电机2 控制模式参数帧（只读安全）
    CanFrame f;
    f.id = 2; f.dlc = 8; f.is_extended = 0;
    uint8_t d[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5B, 0xEC};
    std::memcpy(f.data, d, 8);
    u2c.send(1, f);
    printf("[INFO] 已发读参数帧到 CAN1 motor2 (id=0x02, dlc=8)\n");

    int got = 0;
    for (int k = 0; k < 10 && got < 3; k++) {
        std::vector<CanFrame> frames;
        if (u2c.recv(1, frames, 100)) {
            for (const auto& fr : frames) {
                printf("[RX] id=0x%03X dlc=%u %s data=", fr.id, fr.dlc,
                       fr.is_extended ? "EXT" : "STD");
                for (int i = 0; i < fr.dlc; i++) printf("%02X ", fr.data[i]);
                printf("\n");
                got++;
            }
        }
    }
    if (got == 0)
        printf("[!] 无反馈。检查：波特率是否匹配、CAN_H/L 接线、120Ω 终端电阻、电机是否上电。\n");
    else
        printf("[OK] 收到 %d 帧 —— USB2CAN 链路通！id 应在 0x33(51=50+motor2) 附近。\n", got);

    u2c.close(1);
    printf("[INFO] Example39 完成\n");
}

// ================= 示例 40：USB2CAN 读 CAN1 电机姿态 =================
// 走完整 MotorManager 链路验证 CAN1 的 USB2CAN 可用性：
//   命令下发（使能/目标）→ motor.transport(Usb2CanTransport) → CAN 总线
//   状态回报（51-54 帧）→ ReceiveOnce → Usb2CanTransport 回调 → GetStatus
// 只使能 CAN1（其余路不动），目标=当前角低增益保持，读位置/速度/扭矩。
void Example40_Usb2CanReadStatus() {
    printf("\n========== Example 40: USB2CAN 读电机姿态（4 路全 USB2CAN，不使能） ==========\n");
    printf("[INFO] 不使能电机。发读参数帧经 USB2CAN：角度/速度/扭矩→更新 GetStatus；\n");
    printf("       控制模式 0x5B→接收线程打印 [PARAM]（链路证据）。\n");
    printf("       ⚠ 判读：电机参数读得到（GetStatus 非 0 / [PARAM] 有打印）→ 发送方向通；\n");
    printf("         读不到（GetStatus 恒 0、无 [PARAM]）→ USB2CAN 发送方向断（读帧未达电机）。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());   // 4 路全 USB2CAN（2 双路模块）
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 不使能！只发读参数帧（RW=0），回报更新 MotorStatus / default 分支打印 [PARAM]
    // 聚焦诊断寄存器：型号(0x50) / 错误(0x12) / CAN超时(0x57) / 限位(0x53/0x54) / 模式(0x5B)
    //   - 型号可确认协议兼容；错误=0 才可能正常控制；
    //   - CAN_Timeout>0 表示"N 个周期未收到控制帧即失能"，若控制帧持续则应保持使能；
    //   - 限位 Max/Min 若电机当前角在此范围内则非限位挡住。
    printf("[INFO] 循环读 4 路电机诊断寄存器（型号/错误/CAN超时/限位/模式，12s，Ctrl+C 退出）...\n");
    for (int k = 0; k < 60 && !g_rl_stop; k++) {
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 4; mi++) {
                motor_mgr.ReadParam(cp, mi, MOTOR_OR_angle);          // 更新 position（GetStatus 显示）
                motor_mgr.ReadParam(cp, mi, MOTOR_WR_Major);          // 电机型号
                motor_mgr.ReadParam(cp, mi, MOTOR_OR_error_register); // 错误寄存器
                motor_mgr.ReadParam(cp, mi, MOTOR_WR_CAN_Timeout);    // CAN 超时失能周期
                motor_mgr.ReadParam(cp, mi, MOTOR_WR_Max_Angle);      // 限位上限
                motor_mgr.ReadParam(cp, mi, MOTOR_WR_Min_Angle);      // 限位下限
                motor_mgr.ReadParam(cp, mi, MOTOR_WR_CONTROL_MODE);   // 控制模式
            }
        usleep(200000);   // 等回报（16 电机 × 7 参数串行，放宽间隔）

        if (k % 4 == 0) {
            printf("  [%2ds] GetStatus（err=错误码, 非0即故障）:\n", k / 4);
            for (int cp = 0; cp < 4; cp++) {
                printf("    CAN%d: ", cp);
                for (int mi = 1; mi <= 4; mi++) {
                    MotorStatus st = motor_mgr.GetStatus(cp, mi);
                    printf("m%d(%+.3f,%+.2f,%+.2f,e%u) ", mi, st.position, st.velocity, st.torque, st.error_code);
                }
                printf("\n");
            }
            fflush(stdout);
        }
        usleep(100000);
    }

    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example40 完成（上方 [PARAM] type=0x5B 即链路通；GetStatus 为姿态）\n");
}

// ================= 示例 41：CAN1 三关节 500Hz 插值回站立 =================
// 单独测 CAN1（FR 右前腿，走 USB2CAN）的三关节 hip/thigh/calf：
//   500Hz 高频控制，从当前姿态插值到站立姿态（{0°,-60°,60°} 真机角）。
// 验证 USB2CAN 在高频（500Hz×3=1500帧/s）下能否驱动关节到位。
void Example41_CAN1_500HzStand() {
    printf("\n========== Example 41: CAN1 三关节 500Hz 插值回站立 ==========\n");
    printf("[WARN] 将使能 CAN1(FR右前腿) hip/thigh/calf，500Hz 插值到站立姿态。狗请架起。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    motor_mgr.SetChannelTransport(1, &Usb2CanTransport::GetInstance());
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 预写模式 → 零扭矩预置（覆盖固件残留目标，避免使能瞬间冲）→ 使能
    for (int mi = 1; mi <= 3; mi++) motor_mgr.SetControlMode(1, mi, IMPEDANCE);
    usleep(100000);
    for (int mi = 1; mi <= 3; mi++) motor_mgr.PreEnableZeroTorque(1, mi);
    usleep(100000);   // 等固件接收零扭矩帧
    for (int mi = 1; mi <= 3; mi++) motor_mgr.EnableMotor(1, mi);
    usleep(200000);

    // 站立目标（真机指令角）与当前位置
    const float target[3] = {
        deg2rad(STAND_HIP_DEG), deg2rad(STAND_THIGH_DEG), deg2rad(STAND_CALF_DEG)
    };
    float cur[3];
    for (int mi = 1; mi <= 3; mi++) cur[mi - 1] = motor_mgr.GetStatus(1, mi).position;

    printf("[INFO] 目标站立: hip=%+.2f° thigh=%+.2f° calf=%+.2f°\n",
           STAND_HIP_DEG, STAND_THIGH_DEG, STAND_CALF_DEG);
    printf("[INFO] 当前: hip=%+.3f thigh=%+.3f calf=%+.3f rad\n", cur[0], cur[1], cur[2]);

    // 500Hz 插值 2s 到站立
    const int   HZ = 500;
    const int   STEPS = HZ * 2;   // 2s
    const float KP = 200.0f, KD = 20.0f;
    printf("[INFO] 500Hz 插值 2s 到站立（每 0.5s 打印实际位置）...\n");
    for (int k = 0; k <= STEPS && !g_rl_stop; k++) {
        float t = (float)k / STEPS;
        for (int mi = 1; mi <= 3; mi++) {
            float pos = cur[mi - 1] + (target[mi - 1] - cur[mi - 1]) * t;
            motor_mgr.SendImpedance(1, mi, pos, 0.0f, KP, KD, 0.0f);
        }
        if (k % 250 == 0) {
            MotorStatus s1 = motor_mgr.GetStatus(1, 1);
            MotorStatus s2 = motor_mgr.GetStatus(1, 2);
            MotorStatus s3 = motor_mgr.GetStatus(1, 3);
            printf("  [%3.0f%%] 目标(%.3f,%.3f,%.3f) 实际(%.3f,%.3f,%.3f)\n",
                   t * 100, target[0], target[1], target[2],
                   s1.position, s2.position, s3.position);
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    // 保持站立 20s 确认到位
    printf("[INFO] 到位，保持 1s 确认...\n");
    for (int k = 0; k < 10000 && !g_rl_stop; k++) {
        for (int mi = 1; mi <= 3; mi++)
            motor_mgr.SendImpedance(1, mi, target[mi - 1], 0.0f, KP, KD, 0.0f);
        if (k % 250 == 0) {
            MotorStatus s1 = motor_mgr.GetStatus(1, 1);
            MotorStatus s2 = motor_mgr.GetStatus(1, 2);
            MotorStatus s3 = motor_mgr.GetStatus(1, 3);
            printf("  保持: 实际(%.3f,%.3f,%.3f) 目标(%.3f,%.3f,%.3f)\n",
                   s1.position, s2.position, s3.position, target[0], target[1], target[2]);
            fflush(stdout);
        }
        usleep(1000000 / HZ);
    }

    printf("[INFO] 失能 CAN1 关节...\n");
    for (int mi = 1; mi <= 3; mi++) motor_mgr.DisableMotor(1, mi);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example41 完成\n");
}

// ================= 示例 42：USB2CAN 控制频率测试 =================
// 测达妙 USB2CAN 可达控制频率（发送吞吐），不使能电机（只发读参数帧，安全）：
//   1) 单帧 send 耗时（50 帧）→ 理论上限 + 多电机循环推算
//   2) 目标频率节流扫描（100/500/1k/2k/4k Hz，usleep 节流）→ 实际可达
// 结论参考：单帧 ~53us → 单电机上限 ~18kHz；实际频率受 usleep 精度限制。
void Example42_Usb2CanRateTest() {
    printf("\n========== Example 42: USB2CAN 控制频率测试 ==========\n");
    printf("[INFO] 不使能电机，只发读参数帧测 USB2CAN 发送吞吐（可达控制频率）。\n\n");

    Usb2CanTransport& u2c = Usb2CanTransport::GetInstance();
    TransportConfig cfg;
    cfg.device_idx = 1;
    cfg.usb_baud   = 0;
    if (!u2c.open(1, cfg)) { printf("[ERROR] Usb2Can open 失败\n"); return; }

    CanFrame f;
    f.id = 1; f.dlc = 8; f.is_extended = 0;
    uint8_t d[8] = {0x80, 0, 0, 0, 0, 0, 0x0F, 0xEC};
    std::memcpy(f.data, d, 8);

    // 1) 单帧 send 耗时 → 理论上限（50 帧，避免 USB 缓冲阻塞）
    double tmin = 1e9, tmax = 0, tsum = 0;
    for (int i = 0; i < 50; i++) {
        auto t0 = std::chrono::steady_clock::now();
        u2c.send(1, f);
        double us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - t0).count();
        if (us < tmin) tmin = us;
        if (us > tmax) tmax = us;
        tsum += us;
    }
    double avg = tsum / 50;
    printf("[单帧] 平均 %.2f us, 最小 %.2f, 最大 %.2f → 单电机理论上限 ~%.0f Hz\n",
           avg, tmin, tmax, 1e6 / avg);
    printf("        多电机推算: 每轮 %d 电机 × %.0fus → 循环上限 ~%.0f Hz\n",
           4, 4 * avg, 1e6 / (4 * avg));

    // 2) 目标频率节流扫描（1 电机，usleep 节流模拟控制循环）
    int freqs[] = {100, 500, 1000, 2000, 4000};
    for (int fq : freqs) {
        int period_us = 1000000 / fq;
        auto t0 = std::chrono::steady_clock::now();
        int sent = 0, fail = 0;
        auto end = t0 + std::chrono::seconds(1);
        while (std::chrono::steady_clock::now() < end) {
            if (u2c.send(1, f)) sent++; else fail++;
            usleep(period_us);
        }
        double dur = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        printf("[%4dHz] 实际 %.0f 帧/%.2fs = %.0f Hz (期望 %d), fail=%d\n",
               fq, (double)sent, dur, sent / dur, fq, fail);
    }

    u2c.close(1);
    printf("[INFO] Example42 完成\n");
}

// ================= 示例 43：CAN 顺序标定（轮电机为准） =================
// 4 个达妙模块全接（CAN0~3 各一路），走完整 MotorManager 链路，读电机反馈
// 判断各 USB2CAN 模块的逻辑路序。判定：每路 [Usb2Can: CANx] 出现 + GetStatus
// 各电机有值 = 该路 USB2CAN 通。
// ⚠ 设备插入顺序 = 逻辑路：第一个插入的模块是 CAN0，依次 CAN1/CAN2/CAN3。
void Example43_CANOrderCalibrate() {
    printf("\n========== Example 43: CAN 顺序标定（轮电机为准） ==========\n");
    printf("[INFO] 4 路 USB2CAN，不使能电机。依次转动 FL/FR/RL/RR 腿的轮子，\n");
    printf("       每腿轮子转超过 360°（一整圈）程序自动记录该路，进入下一条。\n");
    printf("       只检测轮电机(motor4)，避免腿关节微动干扰。\n\n");

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());
    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    const char* LEGS[4] = {"FL(左前)", "FR(右前)", "RL(左后)", "RR(右后)"};
    int leg_to_can[4] = {-1, -1, -1, -1};
    const float PI = 3.14159265f;

    for (int leg = 0; leg < 4; leg++) {
        printf("\n[%d/4] 请转动 %s 腿的轮子，转超过 360°（一整圈）...\n", leg + 1, LEGS[leg]);
        fflush(stdout);

        // 初始化各路轮角度基准
        float prev_w[4];
        for (int cp = 0; cp < 4; cp++) {
            motor_mgr.ReadParam(cp, 4, MOTOR_OR_angle);
            usleep(10000);
            prev_w[cp] = motor_mgr.GetStatus(cp, 4).position;
        }

        // 只读轮角度，累积变化（处理 ±π wrap），转超 2π(360°) 判为该路
        float acc[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        int best_can = -1;
        for (int k = 0; k < 500 && best_can < 0 && !g_rl_stop; k++) {  // 10s (20ms/次)
            for (int cp = 0; cp < 4; cp++)
                motor_mgr.ReadParam(cp, 4, MOTOR_OR_angle);
            usleep(20000);
            if (k % 2 == 0) {   // 每 0.2s 累积轮角度变化
                for (int cp = 0; cp < 4; cp++) {
                    float cur = motor_mgr.GetStatus(cp, 4).position;
                    float d = cur - prev_w[cp];
                    while (d > PI)  d -= 2 * PI;   // wrap 到 [-π, π]
                    while (d < -PI) d += 2 * PI;
                    acc[cp] += std::fabs(d);
                    prev_w[cp] = cur;
                }
                for (int cp = 0; cp < 4; cp++) {
                    if (acc[cp] > 2.0f * PI) { best_can = cp; break; }   // 转超 360°
                }
            }
        }

        if (best_can >= 0) {
            leg_to_can[leg] = best_can;
            printf("  ✓ %s 轮转超 360° -> CAN%d (累积 %.0f°)\n",
                   LEGS[leg], best_can, acc[best_can] * 180.0f / PI);
        } else {
            printf("  ✗ 超时未检测到 %s 轮转动（检查轮是否转够 360°/该路是否通）\n", LEGS[leg]);
        }
    }

    printf("\n========== CAN 顺序标定结果 ==========\n");
    for (int leg = 0; leg < 4; leg++)
        printf("  %-10s -> CAN%d\n", LEGS[leg], leg_to_can[leg]);
    printf("  期望: FL=CAN0 FR=CAN1 RL=CAN2 RR=CAN3\n");
    printf("  若不符：调整模块插入顺序（设备索引按 USB 枚举）或改 usb2can_transport 映射。\n");

    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] Example43 完成\n");
}
void Example44_USB2CanXboxControl() {
    printf("\n========== 示例 44：Xbox 手柄控制（USB2CAN 4 路） ==========\n");
    printf("[INFO] 4 路全走达妙 USB2CAN。A键  = 起立（记录按下时的姿态作为返回点）\n");
    printf("[INFO] B键  = 缓慢回到起立前的初始姿态\n");
    printf("[INFO] 十字键↑ = 升高身体  |  十字键↓ = 降低身体\n");
    printf("[INFO] 右摇杆上下 = 前进/后退  |  右摇杆左右 = 差速转向（轮走阻抗前馈扭矩 ±3Nm）\n");
    printf("[INFO] Back键 = 退出\n\n");

    // ---- 控制参数 ----
    // 关节 kp/kd/重力前馈已移到 include/motor_calibration.h 的 JOINT_IMPEDANCE 表，
    // 按 [can_port][joint] 逐关节可调，用 GetJointImpedance(leg, j+1) 取。
    // 控制周期、机身高度范围、轮电机参数见 robot_calibration.h §5，
    // 此处不再重复定义（同名局部常量会遮蔽表中的值，改表不生效）。
    const int   HZ = CONTROL_HZ;                 // 主循环频率
    const float WHEEL_DEAD_ZONE  = 0.05f;        // 轮子摇杆死区 (归一化，仅本例使用)
    const float WHEEL_TORQUE_MAX = 3.0f;         // 轮子阻抗前馈扭矩上限 (Nm) —— 满摇杆 ±3Nm

    // ---- 初始化 ----
    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    // 4 路全走达妙 USB2CAN（2 个双路模块 × 2 通道）
    motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 先把 16 个电机的固件模式全部写好，再使能。
    // SetControlMode 直接发 0x5B 帧、不经发送线程，所以未使能也能写进去。
    // 若省掉这步，电机会在固件默认模式（阻抗）下被使能，固件拿一个未知的
    // 位置目标去闭环——实测 CAN1 轮电机在使能瞬间就转起来。
    // ⚠ 轮不能以 SPEED 模式直接使能：固件使能初始化动作（疑似转子对齐）
    //    绕过速度环直驱电流环，实测疯转（robot_calibration.h §5 已证伪此方向，
    //    PreEnableZeroSpeed 零速度帧也压不住）。本例轮全程走阻抗前馈扭矩
    //    （kp=kd=0, 上位机给 τ），不用固件速度环。
    printf("[INFO] 预写固件控制模式（关节/轮均阻抗，轮走前馈扭矩）...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            motor_mgr.SetControlMode(cp, mi, IMPEDANCE);
        }
        motor_mgr.SetControlMode(cp, 4, IMPEDANCE);
    }
    usleep(100000);   // 留 100ms 给固件写入生效（远大于 20ms 的 settle 窗口）

    // 使能前安全预置（直发，覆盖固件残留目标，避免使能瞬间动作）：全电机零扭矩阻抗帧
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            motor_mgr.PreEnableZeroTorque(cp, mi);
    usleep(100000);

    printf("[INFO] 使能全部电机...\n");
    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 4; mi++) {
            motor_mgr.EnableMotor(cp, mi);
        }
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

            // 记录手柄输入（与电机日志同时间戳基准，便于后续对齐排查）
            MotorLogger::GetInstance().LogXbox(
                state.left_stick_x, state.left_stick_y,
                state.right_stick_x, state.right_stick_y,
                state.left_trigger, state.right_trigger,
                state.a, state.b, state.x, state.y, state.lb, state.rb,
                state.back, state.start, state.ls, state.rs,
                state.dpad_up, state.dpad_down, state.dpad_left, state.dpad_right);

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

            if (f % 500 == 0) {   // 500Hz 循环下 0.1s→1s 一条，避免刷屏
                printf("  起立中... %3.0f%%\n", t * 100.0f);
                fflush(stdout);
            }

            usleep(1000000 / HZ);
        }

        printf("[INFO] 已站立。十字键↑↓=高度, 右摇杆=轮子, Back=退出.\n\n");
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

            // 记录手柄输入（与电机日志同时间戳基准，便于后续对齐排查）
            MotorLogger::GetInstance().LogXbox(
                state.left_stick_x, state.left_stick_y,
                state.right_stick_x, state.right_stick_y,
                state.left_trigger, state.right_trigger,
                state.a, state.b, state.x, state.y, state.lb, state.rb,
                state.back, state.start, state.ls, state.rs,
                state.dpad_up, state.dpad_down, state.dpad_left, state.dpad_right);

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

                // 先停轮子：坐下过程中轮子不该继续转（阻抗前馈，扭矩归零）
                for (int cp = 0; cp < 4; cp++) {
                    motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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

                    if (f % 500 == 0) {   // 0.2s→1s 一条
                        printf("  回落中... %3.0f%%\n", t * 100.0f);
                        fflush(stdout);
                    }
                    usleep(1000000 / HZ);
                }

                printf("[INFO] 已回到初始姿态，退出主循环\n");
                fflush(stdout);
                break;
            }

            // ---- 身体高度调节（十字键 ↑/↓，替代扳机） ----
            // RT 扳机硬件漂移（未按下就输出 0.496），改用数字量十字键，无漂移、可靠。
            // 步长与扳机满程速率一致（HEIGHT_ADJUST_RATE / HZ），按住即连续调节。
            const float HEIGHT_STEP = HEIGHT_ADJUST_RATE / HZ;
            if (state.dpad_up)
                body_height += HEIGHT_STEP;
            if (state.dpad_down)
                body_height -= HEIGHT_STEP;
            body_height = clamp(body_height, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX);

            // ---- 轮子控制（右摇杆：Y 轴前后，X 轴左右转，阻抗前馈扭矩）----
            // 摇杆归一化输入，各自去死区
            float fwd_stick  = -state.right_stick_y;   // 上推为正 = 前进
            float turn_stick =  state.right_stick_x;   // 右推为正 = 右转
            if (fabsf(fwd_stick)  < WHEEL_DEAD_ZONE) fwd_stick  = 0.0f;
            if (fabsf(turn_stick) < WHEEL_DEAD_ZONE) turn_stick = 0.0f;

            // 差速：右转时左侧加扭矩、右侧减扭矩。X 轴单独推即原地转向。
            // 阻抗前馈（kp=kd=0，上位机开环）：满摇杆 = ±WHEEL_TORQUE_MAX Nm
            float tau_fwd  = fwd_stick  * WHEEL_TORQUE_MAX;
            float tau_turn = turn_stick * WHEEL_TORQUE_MAX * 0.8f;   // 转向分量留 20% 余量
            float tau_left  = tau_fwd + tau_turn;
            float tau_right = tau_fwd - tau_turn;

            // 合成扭矩可能超过单轮上限。按同一比例缩放两侧而非各自钳位，
            // 否则左右差值被改变，转弯半径会随速度漂移。
            float tau_peak = fmaxf(fabsf(tau_left), fabsf(tau_right));
            if (tau_peak > WHEEL_TORQUE_MAX) {
                float scale = WHEEL_TORQUE_MAX / tau_peak;
                tau_left  *= scale;
                tau_right *= scale;
            }

            // CAN0(FL)/CAN2(RL) 为左侧，CAN1(FR)/CAN3(RR) 为右侧
            motor_mgr.SendImpedance(0, 4, 0.0f, 0.0f, 0.0f, 0.0f, tau_left);
            motor_mgr.SendImpedance(2, 4, 0.0f, 0.0f, 0.0f, 0.0f, tau_left);
            motor_mgr.SendImpedance(1, 4, 0.0f, 0.0f, 0.0f, 0.0f, tau_right);
            motor_mgr.SendImpedance(3, 4, 0.0f, 0.0f, 0.0f, 0.0f, tau_right);

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

            // 每 1 秒打印状态（轮子显示目标扭矩与 CAN0 反馈速度）
            if (frame % 500 == 0) {   // 500Hz 主循环下 0.1s→1s 一条
                MotorStatus w0 = motor_mgr.GetStatus(0, 4);
                MotorStatus w1 = motor_mgr.GetStatus(1, 4);
                printf("  高度=%+.3fm  轮扭矩 左=%+.2f 右=%+.2f  "
                       "实测 轮0=%+.2f 轮1=%+.2f rad/s  十字键↑=%d ↓=%d\n",
                       body_height, tau_left, tau_right,
                       w0.velocity, w1.velocity,
                       state.dpad_up, state.dpad_down);
                fflush(stdout);
            }

            frame++;
            usleep(1000000 / HZ);
        }
    } // 起立 & 主循环结束

    // ---- 清理 ----
    printf("[INFO] 正在关闭...\n");

    // 停止轮电机（阻抗前馈，扭矩归零）
    for (int cp = 0; cp < 4; cp++) {
        motor_mgr.SendImpedance(cp, 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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

    printf("[INFO] 示例44 完成\n");
    fflush(stdout);
}

// ================= 示例 45：USB2CAN 移到物理零位 =================

void Example45_USB2CanMoveToZero() {
    printf("\n========== Example 45: Move Selected Motor to Physical Zero (USB2CAN) ==========\n");
    printf("[INFO] 4 路全走达妙 USB2CAN。Calibrated coordinate: position 0 = physical zero\n");
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
    motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());   // 4 路全 USB2CAN

    if (!motor_mgr.Initialize(thread_mgr)) {
        printf("[ERROR] MotorManager 初始化失败\n");
        return;
    }

    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 只使能选中的单个电机 ----
    printf("[INFO] 使能 CAN%d-M%d...\n", can_port, motor_id);
    motor_mgr.SetControlMode(can_port, motor_id, IMPEDANCE);   // 写模式
    usleep(100000);
    motor_mgr.PreEnableZeroTorque(can_port, motor_id);          // 零扭矩预置，避免使能瞬间冲
    usleep(100000);
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

    // ---- 保持 20 秒 ----
    printf("\n[INFO] 保持中...\n");
    motor_mgr.SendImpedance(can_port, motor_id, tgt_pos, 0.0f, KP, KD, 0.0f);
    sleep(20);

    st = motor_mgr.GetStatus(can_port, motor_id);
    printf("[INFO] 最终: CAN%d-M%d = %.4f rad (%.2f°)\n\n",
           can_port, motor_id, st.position, rad2deg(st.position));

    // ---- 清理 ----
    printf("[INFO] 禁用电机...\n");
    motor_mgr.DisableMotor(can_port, motor_id);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();

    printf("[INFO] Example45 完成\n");
    fflush(stdout);
}

// ================= 示例 46：USB2CAN 单电机阶跃响应测试 =================
// 目标：区分"批量发送丢帧"（SendOnce 1ms 周期发 16 帧，USB2CAN 吞吐不足导致 thigh 阻抗指令不连续）
//      vs "单发也弱"（USB2CAN 链路本身或电机问题）。
// 做法：只使能 1 个电机，发 +0.3 rad 位置阶跃（kp=200/kd=20），记录 3s 内实际位置/扭矩。
// 判读：
//   扭矩≈60Nm(200×0.3) 且 1s 内到位 → 电机正常发力 → 问题在批量发送（1ms×16 帧吞吐不足）
//   扭矩<20Nm 或到位慢 → 单发也弱 → USB2CAN 链路/电机问题（非批量）
//   到位但扭矩很小 → 阻抗刚度低，kp 未真正生效（查固件阻抗参数）
void Example46_USB2CanSingleMotorStep() {
    printf("\n========== 示例 46：USB2CAN 单电机阶跃响应测试 ==========\n");
    printf("[INFO] 只使能 1 个电机，发 +0.3 rad 位置阶跃，记录实际位置/扭矩（3s）。\n");
    printf("[INFO] 判读：扭矩≈60Nm(200×0.3) 且快速到位 → 电机正常，问题在批量发送(1ms×16帧)；\n");
    printf("[INFO]       扭矩<20Nm 或到位慢 → 单发也弱，USB2CAN 链路/电机问题。\n\n");

    int can_port = -1, motor_id = -1;
    printf("CAN 端口 (0~3, 回车默认 1=FR): "); fflush(stdout);
    if (scanf("%d", &can_port) != 1 || can_port < 0 || can_port > 3) { can_port = 1; }
    printf("电机 ID (1~4, 回车默认 2=Thigh): "); fflush(stdout);
    if (scanf("%d", &motor_id) != 1 || motor_id < 1 || motor_id > 4) { motor_id = 2; }
    printf("[INFO] 选择 CAN%d-M%d\n", can_port, motor_id);

    MotorManager& motor_mgr = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    motor_mgr.SetTransport(&Usb2CanTransport::GetInstance());   // 4 路全 USB2CAN
    if (!motor_mgr.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // 使能单个电机（阻抗安全使能）
    motor_mgr.SetControlMode(can_port, motor_id, IMPEDANCE);
    usleep(100000);
    motor_mgr.PreEnableZeroTorque(can_port, motor_id);
    usleep(100000);
    motor_mgr.EnableMotor(can_port, motor_id);
    usleep(300000);

    // 保持当前角 0.5s，确认电机响应（若已自由/不发力，保持阶段实际会漂移）
    float start = motor_mgr.GetStatus(can_port, motor_id).position;
    motor_mgr.SendImpedance(can_port, motor_id, start, 0.0f, 200.0f, 20.0f, 0.0f);
    usleep(500000);
    MotorStatus s0 = motor_mgr.GetStatus(can_port, motor_id);
    printf("\n[INFO] 保持中：目标 %+.3f，实际 %+.3f，扭矩 %+.2f Nm\n",
           start, s0.position, s0.torque);
    if (fabsf(s0.position - start) > 0.05f)
        printf("[WARN] 保持阶段漂移 %+.3f rad → 电机未在阻抗闭环（kp 未生效/自由）\n",
               s0.position - start);

    // 阶跃 +0.3 rad，观察 3s（50Hz 下发）
    float target = start + 0.3f;
    printf("\n[INFO] 阶跃：%+.3f → %+.3f rad (+0.3)，kp=200/kd=20\n", start, target);
    printf("   时间   目标     实际     误差    扭矩\n");
    for (int f = 0; f < 60; f++) {
        motor_mgr.SendImpedance(can_port, motor_id, target, 0.0f, 200.0f, 20.0f, 0.0f);
        if (f % 5 == 0) {
            MotorStatus st = motor_mgr.GetStatus(can_port, motor_id);
            printf("  %4.1fs  %+.3f  %+.3f  %+.3f  %+6.2f\n",
                   f * 0.05f, target, st.position, target - st.position, st.torque);
        }
        usleep(20000);
    }

    // 稳态判读
    MotorStatus sf = motor_mgr.GetStatus(can_port, motor_id);
    float err = target - sf.position;
    float eff_kp = (fabsf(err) > 1e-4f) ? (sf.torque / err) : 0.0f;   // 实测等效刚度 Nm/rad
    printf("\n[判读] 稳态误差 %+.3f rad，扭矩 %+.2f Nm（kp×0.3 ≈ %.0f Nm）\n",
           err, sf.torque, 200.0f * 0.3f);
    // 扭矩与误差自洽（实测 kp≈设定 kp）→ 阻抗闭环正常，误差来自重力/负载平衡：
    //   稳态误差 = 负载扭矩 / kp，纯 PD 无前馈时的固有特性，加 tau_ff/增大 kp 才可消除。
    if (fabsf(sf.torque) > 5.0f && eff_kp > 50.0f && eff_kp < 1000.0f)
        printf("[结论] 阻抗闭环正常：实测刚度 %.0f Nm/rad（设定 200），扭矩 %+.1f Nm 平衡负载，\n"
               "       稳态误差 %+.3f rad = 负载/kp（纯 PD 固有，非链路/电机问题）\n",
               eff_kp, sf.torque, err);
    else if (fabsf(err) < 0.03f && fabsf(sf.torque) > 30.0f)
        printf("[结论] 电机正常发力 → 问题在批量发送（SendOnce 1ms×16 帧 USB2CAN 吞吐不足）\n");
    else if (fabsf(err) < 0.03f)
        printf("[结论] 到位但扭矩小 → 阻抗刚度低（kp 未真正生效），查固件阻抗参数\n");
    else
        printf("[结论] 扭矩与误差不自洽（实测 kp %.0f），且未到位 → USB2CAN 链路或电机问题，非批量发送\n",
               eff_kp);

    motor_mgr.DisableMotor(can_port, motor_id);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    motor_mgr.Stop();
    printf("[INFO] 示例46 完成\n");
}

// ================= 示例 47：悬空 chirp 扫频 + 最小二乘参数辨识 =================
// 目的：辨识单关节执行器参数，用于 tau_ff 前馈（重力/摩擦补偿）与 KP/KD 临界阻尼整定。
// 模型：τ = J·θ̈ + B·θ̇ + f_c·sign(θ̇) + b
//       J=转动惯量(kg·m²), B=粘性阻尼(Nm·s/rad), f_c=库仑摩擦(Nm), b=重力偏置(Nm)
// 做法：悬空固定机身，对单关节开环扭矩 chirp 扫频激励（kp=kd=0），500Hz 采集 θ/θ̇/τ，
//       离线最小二乘拟合 4 参数（正规方程 + 高斯消元，纯 C++ 无外部依赖）。
// 安全：幅度小（默认 ±1.5 Nm）、从低频扫起、|位置|超 0.6 rad 自动停止。务必固定机身。
// 注意：开环扭矩下电机会摆动；若摇头/振动剧烈立即 Ctrl+C（signal 已接）。
static bool SysIdSolveN(const float A[5][5], const float b[5], float x[5], int n) {
    float M[5][6];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) M[i][j] = A[i][j];
        M[i][n] = b[i];
    }
    for (int col = 0; col < n; col++) {
        int piv = col;
        for (int r = col + 1; r < n; r++)
            if (fabsf(M[r][col]) > fabsf(M[piv][col])) piv = r;
        if (fabsf(M[piv][col]) < 1e-9f) return false;
        if (piv != col)
            for (int j = col; j <= n; j++) std::swap(M[piv][j], M[col][j]);
        for (int r = 0; r < n; r++) {
            if (r == col) continue;
            float f = M[r][col] / M[col][col];
            for (int j = col; j <= n; j++) M[r][j] -= f * M[col][j];
        }
    }
    for (int i = 0; i < n; i++) x[i] = M[i][n] / M[i][i];
    return true;
}

// 单关节辨识结果
struct JointSysIdResult {
    float J = 0, B = 0, fc = 0, Kg = 0, b = 0;
    float amp = 0, pos_range = 0;
    bool  ok = false;
};

void Example47_ChirpSysId() {
    printf("\n========== 示例 47：整狗站立 + 12 关节 chirp 扫描辨识 ==========\n");
    printf("[INFO] 模型 τ = J·θ̈ + B·θ̇ + f_c·sign(θ̇) + K_g·θ + b\n");
    printf("[INFO] 流程：整狗起立 STAND_*（四轮悬空）→ 逐个扫描 12 腿关节\n");
    printf("[INFO]   每关节：读 τ_steady → 斜坡测静摩擦定幅度 → 开环 chirp(0.5→15Hz, 8s)\n");
    printf("[INFO] 其他腿全程 PD 保持站立；结果汇总表 + log/sysid_all.csv\n");
    printf("[INFO] 每关节约 15~30s，12 关节约 4~6 分钟；异常立即 Ctrl+C。\n\n");

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    mm.SetTransport(&Usb2CanTransport::GetInstance());
    if (!mm.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 整狗起立到 STAND_*（12 腿关节使能 + 插值，轮子悬空零扭矩）----
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.SetControlMode(cp, mi, IMPEDANCE);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.PreEnableZeroTorque(cp, mi);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.EnableMotor(cp, mi);
    usleep(300000);

    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }
    float start_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            start_pos[cp * 3 + mi - 1] = mm.GetStatus(cp, mi).position;

    printf("[INFO] 整狗起立中（10s 到 STAND_* {0,-60,60}°，轮子悬空）...\n");
    const int STAND_FRAMES = 500;
    for (int f = 0; f <= STAND_FRAMES; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = start_pos[cp * 3 + mi - 1]
                          + (stand_q[cp * 3 + mi - 1] - start_pos[cp * 3 + mi - 1]) * t;
                mm.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  起立 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / 50);
    }
    printf("[INFO] 起立完成，狗保持站立（轮子悬空）。开始 12 关节扫描。\n");

    const char* jname[3] = {"hip", "thigh", "calf"};
    JointSysIdResult res[4][3];
    const float F0 = 0.5f, F1 = 15.0f, DUR = 8.0f, DT = 0.002f;
    const float POS_LIMIT = 0.6f;   // 相对站立角（绝对位置 thigh/calf 本身超 ±0.6，须相对）
    const int   N = (int)(DUR / DT);

    for (int cp = 0; cp < 4; cp++) {
        for (int mi = 1; mi <= 3; mi++) {
            printf("\n===== [%d/12] 辨识 CAN%d-M%d（%s）=====\n",
                   cp * 3 + mi, cp, mi, jname[mi - 1]);

            // 其他 11 腿保持站立（PD 保持 STAND_*，狗不塌）
            auto keep_others = [&]() {
                for (int c2 = 0; c2 < 4; c2++)
                    for (int m2 = 1; m2 <= 3; m2++) {
                        if (c2 == cp && m2 == mi) continue;
                        mm.SendImpedance(c2, m2, stand_q[c2 * 3 + m2 - 1],
                                         0.0f, 200.0f, 20.0f, 0.0f);
                    }
            };

            // 读稳态扭矩（站立姿态重力偏置）
            float tau_steady = mm.GetStatus(cp, mi).torque;
            printf("[INFO] τ_steady=%+.2f Nm\n", tau_steady);

            // 斜坡测静摩擦：chirp 幅度须 > 静摩擦才动
            float amp_bias = 1.0f;
            bool broke = false;
            printf("[INFO] 斜坡测静摩擦（每 0.5s +0.5 Nm，最长 20s）...\n");
            for (int st = 0; st < 40 && !broke; st++) {
                float extra = 0.5f * (st + 1);
                mm.SendImpedance(cp, mi, 0.0f, 0.0f, 0.0f, 0.0f, tau_steady + extra);
                keep_others();
                usleep(400000);
                MotorStatus s1 = mm.GetStatus(cp, mi);
                usleep(100000);
                MotorStatus s2 = mm.GetStatus(cp, mi);
                if (fabsf(s2.position - s1.position) > 0.02f) {
                    amp_bias = extra + 2.0f;   // 突破量 + 2 Nm 余量
                    printf("  突破 +%.2f Nm → chirp 幅度 %.2f Nm\n", extra, amp_bias);
                    broke = true;
                }
            }
            if (!broke) {
                printf("[WARN] 20s 未突破静摩擦 → 用幅度 4 Nm 试\n");
                amp_bias = 4.0f;
            }

            // 开环 chirp 采集
            const float AMP = amp_bias;
            std::vector<float> pos(N), vel(N), trq(N), tcmd(N);
            int collected = 0;
            for (int i = 0; i < N; i++) {
                float tt = i * DT;
                keep_others();
                float phase = 2.0f * (float)M_PI * (F0 * tt + 0.5f * ((F1 - F0) / DUR) * tt * tt);
                float tau = tau_steady + AMP * sinf(phase);
                mm.SendImpedance(cp, mi, 0.0f, 0.0f, 0.0f, 0.0f, tau);
                tcmd[i] = tau;
                MotorStatus st = mm.GetStatus(cp, mi);
                pos[i] = st.position;
                vel[i] = st.velocity;
                trq[i] = st.torque;
                collected++;
                if (fabsf(pos[i] - stand_q[cp * 3 + mi - 1]) > POS_LIMIT) {
                    printf("[WARN] 相对站立角超限，中止\n");
                    break;
                }
                usleep((useconds_t)(DT * 1e6f));
            }

            // 运动质量检测
            float pmin = pos[0], pmax = pos[0];
            for (int k = 1; k < collected; k++) {
                pmin = fminf(pmin, pos[k]);
                pmax = fmaxf(pmax, pos[k]);
            }
            float prange = pmax - pmin;
            if (prange < 0.02f)
                printf("[WARN] 位置变化仅 %.3f rad——电机几乎没动，结果不可靠\n", prange);

            // 最小二乘拟合（5 参数）：vel 平滑 + 隔帧差分，τ 用指令扭矩
            std::vector<float> vel_s(collected);
            for (int k = 0; k < collected; k++) {
                int lo = k - 2, hi = k + 2;
                if (lo < 0) lo = 0;
                if (hi >= collected) hi = collected - 1;
                float s = 0.0f;
                for (int j = lo; j <= hi; j++) s += vel[j];
                vel_s[k] = s / (hi - lo + 1);
            }
            const int DIFF = 5;
            float ATA[5][5] = {0}, ATb[5] = {0};
            int used = 0;
            for (int k = DIFF; k < collected; k++) {
                float acc = (vel_s[k] - vel_s[k - DIFF]) / (DIFF * DT);
                float v = vel_s[k];
                float sgn = (v > 0.02f) ? 1.0f : ((v < -0.02f) ? -1.0f : 0.0f);
                float row[5] = {acc, v, sgn, pos[k], 1.0f};
                float bval = tcmd[k];
                for (int r = 0; r < 5; r++) {
                    ATb[r] += row[r] * bval;
                    for (int c = 0; c < 5; c++) ATA[r][c] += row[r] * row[c];
                }
                used++;
            }
            float x[5];
            if (used > 50 && SysIdSolveN(ATA, ATb, x, 5)) {
                res[cp][mi - 1] = JointSysIdResult{x[0], x[1], x[2], x[3], x[4], AMP, prange, true};
                printf("[结果] J=%+.4f B=%+.4f f_c=%+.4f K_g=%+.4f b=%+.4f 位置范围%.3f\n",
                       x[0], x[1], x[2], x[3], x[4], prange);
            } else {
                printf("[ERROR] 拟合失败/数据不足（样本 %d）\n", used);
            }
        }
    }

    // ---- 汇总表 ----
    printf("\n\n========== 12 关节辨识汇总 ==========\n");
    printf("关节  J(kg·m²)  B(Nm·s/r)  f_c(Nm)  K_g(Nm/rad)  b(Nm)  幅度 位置范围\n");
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++) {
            auto& r = res[cp][mi - 1];
            if (r.ok)
                printf("C%d-%d %8.4f %9.3f %8.3f %11.2f %8.3f %5.1f %8.3f\n",
                       cp, mi, r.J, r.B, r.fc, r.Kg, r.b, r.amp, r.pos_range);
            else
                printf("C%d-%d   失败\n", cp, mi);
        }

    // 汇总 CSV
    FILE* f = fopen("log/sysid_all.csv", "w");
    if (f) {
        fprintf(f, "can,motor,J,B,fc,Kg,b,amp,pos_range,ok\n");
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                auto& r = res[cp][mi - 1];
                fprintf(f, "%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%d\n",
                        cp, mi, r.J, r.B, r.fc, r.Kg, r.b, r.amp, r.pos_range, r.ok ? 1 : 0);
            }
        fclose(f);
        printf("\n[INFO] 汇总 → log/sysid_all.csv\n");
    }

    // 失能 12 腿（安全回退）
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.DisableMotor(cp, mi);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    mm.Stop();
    printf("[INFO] 示例47 完成\n");
}

// ================= 示例 48：轮子扭矩方向安全验证（不开 RL，无失控风险）=================
// 目的：验证 pos_scale=+1 轮（FR/RR）的 send_tau 方向——排查 RL 站立中 FR/RR 疯转是否方向反。
// 方法：单轮悬空（狗吊起/轮离地），开环给固定扭矩（kp=kd=0），观察编码器 pos 变化方向：
//   +1.5 Nm → pos 增（正向转）→ 正扭矩=正向 → 方向对
//   +1.5 Nm → pos 减（反向转）→ 正扭矩=反向 → 方向反（需翻转 send_tau）
// 安全：单轮、小扭矩、悬空、位置保护，全程不使能其他电机、不跑策略。
void Example48_WheelDirectionVerify() {
    printf("\n========== 示例 48：轮子扭矩方向安全验证 ==========\n");
    printf("[INFO] 单轮悬空开环 ±1.5 Nm，观察 pos 方向，判断 send_tau 是否反向。\n");
    printf("[WARN] 轮子悬空会持续加速转，小扭矩安全；确认轮子离地/狗吊起。\n\n");

    int can_port = -1;
    printf("CAN 端口 (0~3, 回车默认 1=FR 轮): "); fflush(stdout);
    if (scanf("%d", &can_port) != 1 || can_port < 0 || can_port > 3) can_port = 1;
    const int motor_id = 4;   // 轮子
    printf("[INFO] 验证 CAN%d 轮子\n\n", can_port);

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    mm.SetTransport(&Usb2CanTransport::GetInstance());
    if (!mm.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    mm.SetControlMode(can_port, motor_id, IMPEDANCE);
    usleep(100000);
    mm.PreEnableZeroTorque(can_port, motor_id);
    usleep(100000);
    mm.EnableMotor(can_port, motor_id);
    usleep(300000);

    const float TORQ = 1.5f;   // 测试扭矩 (Nm)
    const float POS_LIMIT = 50.0f;  // 位置保护（rad，轮子悬空可转多圈）
    const int   DUR_MS = 500;  // 每方向 0.5s

    // 返回 {pos, cal_vel}，同时验证"正向转时 cal_vel 符号"（速度环反馈方向）
    auto apply_and_read = [&](float tau) -> std::pair<float,float> {
        for (int i = 0; i < DUR_MS; i++) {
            mm.SendImpedance(can_port, motor_id, 0, 0, 0, 0, tau);
            usleep(1000);   // 1ms
        }
        MotorStatus st = mm.GetStatus(can_port, motor_id);
        return {st.position, st.velocity};
    };

    // 阶段 0：零扭矩读初始
    auto [p0, v0] = apply_and_read(0.0f);
    printf("[INFO] 初始 pos=%+.2f vel=%+.2f\n", p0, v0);

    // 阶段 1：+1.5 Nm（正扭矩，应正向转；验证 pos 增且 cal_vel 同为正）
    auto [p1, v1] = apply_and_read(TORQ);
    float d1 = p1 - p0;
    printf("[+%.1fNm] pos %+.2f→%+.2f Δ=%+.3f vel=%+.2f  %s\n", TORQ, p0, p1, d1, v1,
           d1 > 0.05f ? "→ 正向转" : (d1 < -0.05f ? "→ 反向转" : "→ 几乎不动"));
    printf("[验证] 正向转时 cal_vel=%+.2f → %s（若 cal_vel 与 pos 方向相反，速度环会正反馈）\n",
           v1, (d1 > 0.05f && v1 > 0.05f) ? "同号，反馈方向正确"
             : (d1 > 0.05f && v1 < -0.05f) ? "反号！速度环正反馈风险"
             : "未观察到转动");

    // 阶段 2：零扭矩（惯性滑行）
    auto [p2, v2] = apply_and_read(0.0f);
    printf("[ 0 Nm ] pos %+.2f vel=%+.2f（惯性滑行）\n", p2, v2);

    // 阶段 3：-1.5 Nm（对称）
    auto [p3, v3] = apply_and_read(-TORQ);
    float d3 = p3 - p2;
    printf("[-%.1fNm] pos %+.2f→%+.2f Δ=%+.3f vel=%+.2f  %s\n", TORQ, p2, p3, d3, v3,
           d3 < -0.05f ? "→ 反向转" : (d3 > 0.05f ? "→ 正向转" : "→ 几乎不动"));

    // 判读（基于 +1.5 Nm 阶段）
    printf("\n[判读] 正扭矩(+1.5Nm) 下 pos %s\n", d1 > 0.05f ? "增大（正向转）" : (d1 < -0.05f ? "减小（反向转）" : "不变"));
    if (d1 > 0.05f)
        printf("[结论] 正扭矩=正向转 → send_tau 方向正确（pos_scale=%+.0f 不翻转 OK）\n",
               MOTOR_CALIBRATION[can_port][motor_id - 1].pos_scale);
    else if (d1 < -0.05f)
        printf("[结论] 正扭矩=反向转 → send_tau 方向反！pos_scale=%+.0f 轮需翻转扭矩\n",
               MOTOR_CALIBRATION[can_port][motor_id - 1].pos_scale);
    else
        printf("[结论] 未观察到转动，检查轮子是否悬空/使能/扭矩是否生效\n");

    mm.DisableMotor(can_port, motor_id);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    mm.Stop();
    printf("[INFO] 示例48 完成\n");
}

// ================= 示例 49：整狗站立 + 单轮 SPEED 模式速度环测试 =================
// 目的：验证固件 SPEED 速度环（SendSpeed 下发 vel/kvp/ki）能否稳定控制轮子，
//       解决轮子疯转问题。背景：固件阻抗模式的 vel_des 被忽略（kd 只是位置阻尼，
//       kp=0 时无驱动力，实测轮子不动），速度控制必须走 SPEED 模式。
// 疯转根因：轮使能后固件速度反馈存在假偏移（实测 -43 rad/s 或饱和 ±48），
//       满 kvp 会把假偏移当真实误差去追 → 疯转 ~1s。解法：软启动（低 kvp 起步，
//       Example23 已验证 kvp=0.3→3.0 渐变可行）。
// 流程：整狗起立（轮悬空）→ 选单轮 → SPEED 软启动使能 → 等假偏移消退 →
//       对每个 kvp 做目标速度阶跃（+2 → 0 → -2 rad/s 各 2s），记录收敛/振荡。
// 判读：轮速收敛到目标且无振荡 → 该 kvp 可用；振荡/发散 → kvp 过大或反馈问题。
// 安全：悬空、轮子离地、目标速度小（±2）、SendOnce SPEED 分支有轮速超 15 兜底。
void Example49_StandAndWheelSpeedLoopTest() {
    printf("\n========== 示例 49：整狗站立 + 单轮 SPEED 模式速度环测试 ==========\n");
    printf("[INFO] 下发 vel/kvp/ki 给固件速度环（SendSpeed），固件内部 1kHz 闭环。\n");
    printf("[INFO] 软启动使能（低 kvp）→ 等假速度偏移消退 → kvp 扫描 × 目标速度阶跃。\n");
    printf("[WARN] 悬空起立，轮子离地；轮速超 15 rad/s 自动 0 速 0 增益兜底。\n\n");

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    mm.SetTransport(&Usb2CanTransport::GetInstance());
    if (!mm.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 整狗起立到 STAND_*（12 腿使能 + 插值，轮子不使能悬空）----
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.SetControlMode(cp, mi, IMPEDANCE);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.PreEnableZeroTorque(cp, mi);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.EnableMotor(cp, mi);
    usleep(300000);

    float stand_q[12];
    for (int leg = 0; leg < 4; leg++) {
        stand_q[leg * 3 + 0] = deg2rad(STAND_HIP_DEG);
        stand_q[leg * 3 + 1] = deg2rad(STAND_THIGH_DEG);
        stand_q[leg * 3 + 2] = deg2rad(STAND_CALF_DEG);
    }
    float start_pos[12];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            start_pos[cp * 3 + mi - 1] = mm.GetStatus(cp, mi).position;

    printf("[INFO] 整狗起立中（10s 到 STAND_*，轮子悬空）...\n");
    const int STAND_FRAMES = 500;
    for (int f = 0; f <= STAND_FRAMES; f++) {
        float t = (float)f / STAND_FRAMES;
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++) {
                float pos = start_pos[cp * 3 + mi - 1]
                          + (stand_q[cp * 3 + mi - 1] - start_pos[cp * 3 + mi - 1]) * t;
                mm.SendImpedance(cp, mi, pos, 0.0f, 200.0f, 20.0f, 0.0f);
            }
        if (f % 100 == 0) printf("  起立 %4.0f%%\n", t * 100.0f);
        usleep(1000000 / 50);
    }
    printf("[INFO] 起立完成，狗保持站立（轮子悬空）。\n");

    // ---- 选测试轮子 ----
    int can_port = -1;
    printf("\n选测试轮子\nCAN 端口 (0~3, 回车默认 0=FL): "); fflush(stdout);
    if (scanf("%d", &can_port) != 1 || can_port < 0 || can_port > 3) can_port = 0;
    const int motor_id = 4;
    printf("[INFO] 测试 CAN%d 轮子（SPEED 固件速度环）\n\n", can_port);

    // ---- 轮子 SPEED 软启动使能 ----
    // 关键：先预写固件 SPEED 模式，再预置 0 速弱增益，最后才使能。
    // 使能瞬间固件速度反馈可能有假偏移（-43/±48），弱增益下力矩小，
    // 等偏移自行消退（~1s）后再升 kvp 扫描。满 kvp 使能会疯转（历史教训）。
    printf("[INFO] SPEED 软启动使能（预写模式 + 0 速 kvp=0.3）...\n");
    mm.SetControlMode(can_port, motor_id, SPEED);
    usleep(100000);
    mm.SendSpeed(can_port, motor_id, 0.0f, 0.3f, 0.03f);
    usleep(100000);
    mm.EnableMotor(can_port, motor_id);
    usleep(2000000);   // 等假偏移消退窗口（2s，SendOnce 持续发 0 速弱增益）

    MotorStatus w0 = mm.GetStatus(can_port, motor_id);
    printf("[INFO] 使能后轮速反馈 = %+.2f rad/s（|v|>5 说明假偏移未消退，需降 kvp 或加长窗口）\n\n",
           w0.velocity);

    // ---- SPEED 速度环测试：kvp 扫描 × 目标速度阶跃 ----
    const float KVP_LIST[]  = {0.5f, 1.0f, 2.0f, 3.0f, 5.0f};
    const float KI          = 0.05f;             // 小积分，克服摩擦稳态误差
    const float TARGETS[]   = {+2.0f, 0.0f, -2.0f};
    const char* tname[]     = {"正向+2", "刹停0", "反向-2"};
    const int   NSTEP       = 100;               // 每阶段 2s @ 50Hz

    printf("=== SPEED 固件速度环测试（SendSpeed: vel/kvp/ki）===\n");
    for (float kvp : KVP_LIST) {
        printf("\n===== kvp = %.1f (ki=%.2f) =====\n", kvp, KI);
        for (int ts = 0; ts < 3; ts++) {
            float tv = TARGETS[ts];
            float vmin = 1e9f, vmax = -1e9f;
            std::vector<float> vlog;
            for (int i = 0; i < NSTEP; i++) {
                mm.SendSpeed(can_port, motor_id, tv, kvp, KI);
                usleep(20000);
                MotorStatus st = mm.GetStatus(can_port, motor_id);
                vmin = fminf(vmin, st.velocity);
                vmax = fmaxf(vmax, st.velocity);
                vlog.push_back(st.velocity);
                if (i % 25 == 0)
                    printf("  [%s] t=%.1fs vel=%+.2f\n", tname[ts], i * 0.02f, st.velocity);
            }
            // 稳态评估：末 0.5s（25 帧）均值与极差。全段范围含阶跃反转过渡
            // （+2 段从上一阶段 -2 开始，范围必跨 ±2），不代表收敛质量。
            const int N_STEADY = 25;
            int nb = std::max(0, (int)vlog.size() - N_STEADY);
            float vsum = 0.0f, vsmin = 1e9f, vsmax = -1e9f;
            for (int i = nb; i < (int)vlog.size(); i++) {
                vsum += vlog[i];
                vsmin = fminf(vsmin, vlog[i]);
                vsmax = fmaxf(vsmax, vlog[i]);
            }
            float vmean = vsum / std::max(1, (int)vlog.size() - nb);
            float verr  = fabsf(vmean - tv);
            float vspan = vsmax - vsmin;
            printf("  [%s] 目标%+.1f → 全段[%+.2f~%+.2f] 稳态末0.5s均值%+.2f 极差%.2f  | %s\n",
                   tname[ts], tv, vmin, vmax, vmean, vspan,
                   (verr < 0.3f && vspan < 0.5f) ? "稳定到位" : "收敛差/振荡");
        }
    }

    // ---- 轮子急停演示（2026-08-29）----
    // WheelEmergencyStop()：SendOnce 对轮子强制发 SPEED 0 速 + kvp=3.0 制动帧，
    // 固件速度环闭环到 0 速（主动制动，非自由滑行）。验证制动是否快速可靠。
    printf("\n=== 轮子急停演示 ===\n");
    mm.SendSpeed(can_port, motor_id, +3.0f, 2.0f, 0.05f);   // 让轮子匀速转起来
    usleep(1500000);   // 1.5s 稳定到目标
    printf("  触发急停前轮速：\n");
    for (int i = 0; i < 3; i++) {
        MotorStatus st = mm.GetStatus(can_port, motor_id);
        printf("    t=%.1fs vel=%+.2f\n", i * 0.5f, st.velocity);
        usleep(500000);
    }
    printf("  触发 WheelEmergencyStop()（0 速 + kvp=3.0 制动）...\n");
    mm.WheelEmergencyStop();
    float vmin = 1e9f, vmax = -1e9f, vlast = 0.0f;
    for (int i = 0; i < 40; i++) {   // 2s @ 20Hz 采样
        usleep(50000);
        MotorStatus st = mm.GetStatus(can_port, motor_id);
        vmin = fminf(vmin, st.velocity);
        vmax = fmaxf(vmax, st.velocity);
        vlast = st.velocity;
        if (i % 10 == 0) printf("    [急停] t=%.1fs vel=%+.2f\n", i * 0.05f, st.velocity);
    }
    printf("  急停 2s：轮速范围 %+.2f~%+.2f 末值%+.2f  | %s\n",
           vmin, vmax, vlast,
           (fabsf(vlast) < 0.5f) ? "制动到位" : "未完全停住（需加大 kvp 或检查速度反馈）");
    mm.ClearWheelEmergency();

    // 复位轮子（0 速弱增益）
    mm.SendSpeed(can_port, motor_id, 0.0f, 0.5f, 0.0f);
    usleep(500000);
    mm.DisableMotor(can_port, motor_id);

    // 失能 12 腿
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.DisableMotor(cp, mi);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    mm.Stop();
    printf("[INFO] 示例49 完成\n");
}

// ================= 示例 50：趴下姿态手动标定（不使能电机） =================
// 目的：不使能电机（零扭矩），手动把狗腿摆到目标「趴下」姿态，按回车记录
//       当前 12 腿关节实际角度（标定后真机角），作为 LIE_DOWN_*_DEG 的目标值。
// 流程：初始化（只启接收线程，不使能电机）→ 用户摆腿 → 回车 → 读参刷新 →
//       打印每腿 hip/thigh/calf 角度（rad+deg）+ 四腿平均建议值。可多次记录，q 退出。
// 安全：全程不使能电机（自由摆腿），读参帧只读不改。电机驱动须供电（Example31 模式）。
void Example50_LieDownAngleRecord() {
    printf("\n========== 示例 50：趴下姿态手动标定（不使能电机） ==========\n");
    printf("[INFO] 电机未使能（零扭矩），可手动摆狗腿。\n");
    printf("[INFO] 把狗腿摆到目标「趴下」姿态，按回车记录当前 12 关节角。\n");
    printf("[INFO] 连续记录取稳态；q 退出。电机驱动须供电。\n\n");

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    if (!mm.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");   // 只启接收线程（读参回帧），不使能电机

    // 读取 12 腿关节角：发读参数帧触发回帧，等回帧后取标定后角度（Example31 模式）
    auto read_legs = [&](float out[12]) {
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++)
                mm.ReadParam(cp, mi, MOTOR_OR_angle);
        usleep(150000);   // 12 帧串行回帧等待
        for (int cp = 0; cp < 4; cp++)
            for (int mi = 1; mi <= 3; mi++)
                out[cp * 3 + mi - 1] = mm.GetStatus(cp, mi).position;
    };

    const char* legname[4] = {"FL", "FR", "RL", "RR"};
    const char* jname[3]   = {"hip", "thigh", "calf"};
    int record_cnt = 0;

    while (true) {
        printf("\n[操作] 摆好狗腿到目标趴下姿态 → 回车记录；q 退出\n  > ");
        fflush(stdout);
        char line[64];
        if (!fgets(line, sizeof line, stdin)) break;
        if (line[0] == 'q' || line[0] == 'Q') break;

        // 连续读 5 次取稳态均值（排除摆腿过程中抖动读数）
        const int N = 5;
        float sum[12] = {0};
        for (int k = 0; k < N; k++) {
            float pos[12];
            read_legs(pos);
            for (int i = 0; i < 12; i++) sum[i] += pos[i];
        }
        record_cnt++;
        printf("\n=== 记录 #%d（%d 次稳态均值）===\n", record_cnt, N);
        for (int leg = 0; leg < 4; leg++) {
            printf("  %s: ", legname[leg]);
            for (int j = 0; j < 3; j++) {
                float avg = sum[leg * 3 + j] / N;
                printf("%s %+.3f rad (%+.1f°)  ", jname[j], avg, avg * 180.0f / 3.14159f);
            }
            printf("\n");
        }
        // 四腿平均 → 建议 LIE_DOWN_*_DEG（LIE_DOWN 是全腿同构目标）
        float aj[3] = {0};
        for (int j = 0; j < 3; j++) {
            for (int leg = 0; leg < 4; leg++) aj[j] += sum[leg * 3 + j] / N;
            aj[j] /= 4.0f;
        }
        printf("  → 建议 LIE_DOWN: hip %+.1f°  thigh %+.1f°  calf %+.1f°\n",
               aj[0] * 180.0f / 3.14159f, aj[1] * 180.0f / 3.14159f, aj[2] * 180.0f / 3.14159f);
        printf("    （填入 robot_calibration.h 的 LIE_DOWN_*_DEG）\n");
    }

    mm.Stop();
    printf("[INFO] 示例50 完成\n");
}

// ================= 示例 54：吊装摩擦辨识（重力标定 + 前馈恒速 + 双向配对） =================
// 目的：辨识腿关节摩擦参数 [b 粘性, fc 库仑]，供摩擦前馈（rl_controller.h LEG_FF_FC/FV）。
// 流程（⚠ 狗必须吊起悬空、base 刚性固定、腿悬空不触地）：
//   [0] 初始化 USB2CAN + 使能 12 腿关节（轮子不使能）
//   [1] 读吊起后当前姿态 → 每关节扫掠中心 θ_c + 自适应半幅（不撞限位）
//   [2] 逐关节：其他 11 关节锁定 → 阶段C 重力标定(20点×4遍，位置保持读 τ_hold)
//             → 阶段D 前馈恒速扫掠(位置斜坡 + tau_ff=g_est，多档×往返圈×4回合)
//   [3] 数据落盘 log/fric_id/（grav/回合/info CSV），失能退出
//   [4] 离线回归 tool/friction_id_offline.py → b/fc ± σ
//
// 原理（real_robot_identification_plan.md 重力抵消法）：
//   同一 θ 下 τ(+v)=g(θ)+b·v+fc，τ(−v)=g(θ)−b·v−fc，相减消重力 → 2b·v+2fc。
//   g_est 只用于控制前馈（让 PD 有余量跟上斜坡恒速），不参与辨识（配对消掉）。
//   真机速度环在关节电机上危险（高减速比），故用阻抗位置斜坡 + 重力前馈实现恒速。
//
// ⚠ 安全：全程阻抗位置闭环；位置误差超 POS_ERR_LIMIT 立即失能退出。
// ⚠ 参数集中在下方的宏区，实测现场改这里重编译即可。
void Example54_FrictionSysId() {
    // ============ 辨识参数（实测现场可调，改这里重编译）============
    // 扫掠半幅 (rad，指令角)：hip / thigh / calf，会再自适应限位（不撞限位）
    const float SWEEP_AMP[3] = {0.35f, 0.50f, 0.45f};
    const int   N_VEL        = 3;                  // 速度档数
    const float VEL_STEPS[3] = {0.3f, 0.5f, 0.8f}; // 双向恒速档 (rad/s)，关节量程 3 的 10~27%
    const int   CYC_PER_VEL  = 5;                  // 每档往返圈数（1 圈 = 正+反两个单程）
    const int   ROUNDS       = 4;                  // 阶段D 独立回合数（每回合单独回归 → 误差带 σ）
    const int   GRID_PTS     = 20;                 // 阶段C 重力标定点数（区间大了要加密）
    const int   GRID_ROUNDS  = 4;                  // 阶段C 重复遍数（平均去静摩擦随机）
    const float GRAV_HOLD_S  = 1.5f;               // 阶段C 每点位置保持时长 (s)
    const float LOCK_KP      = 500.0f, LOCK_KD = 50.0f;  // 锁定关节 PD（扛腿自重）
    const float SWEEP_KP     = 500.0f, SWEEP_KD = 50.0f; // 目标关节扫掠 PD
    const float MOVE_RATE    = 0.15f;              // 移到扫掠中心斜坡速度 (rad/s)
    const float DT           = 0.002f;             // 控制节拍 500Hz
    const float POS_ERR_LIMIT = 0.20f;             // 位置误差保护（超过失能）
    const float MIN_AMP      = 0.12f;              // 最小扫掠半幅（限位太窄时）

    // 指令角限位 (rad)：θ₁/θ₂/θ₃（robot_calibration.h §4，站立姿态压边界须注意）
    const float LIM_LO[3] = {deg2rad(-60), deg2rad(-70), deg2rad(60)};
    const float LIM_HI[3] = {deg2rad(0),   deg2rad(90),  deg2rad(180)};

    printf("\n========== 示例 54：吊装摩擦辨识 ==========\n");
    printf("[WARN] 狗必须吊起悬空、base 刚性固定、腿悬空不触地！\n");
    printf("[INFO] 逐关节：重力标定(20点×4遍) → 前馈恒速扫掠(3档×5圈×4回合)，落盘 log/fric_id/\n");
    printf("[INFO] 12 关节约 50~70 分钟；异常立即 Ctrl+C 失能。\n\n");

    MotorManager& mm = MotorManager::GetInstance();
    ThreadManager thread_mgr;
    mm.SetTransport(&Usb2CanTransport::GetInstance());
    if (!mm.Initialize(thread_mgr)) { printf("[ERROR] 初始化失败\n"); return; }
    thread_mgr.start_thread("motor_receive");
    thread_mgr.start_thread("motor_send");
    sleep(1);

    // ---- 使能 12 腿关节（轮子不使能，悬空无负载）----
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.SetControlMode(cp, mi, IMPEDANCE);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.PreEnableZeroTorque(cp, mi);
    usleep(100000);
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.EnableMotor(cp, mi);
    usleep(300000);

    // 急停：Ctrl+C → g_rl_stop=1 → 各循环退出 → 统一失能（安全）
    g_rl_stop = 0;
    signal(SIGINT, rl_signal_handler);

    ::mkdir("log/fric_id", 0755);

    // ---- 读吊起后当前姿态 → 每关节扫掠中心 + 自适应半幅 ----
    float theta_c[12], amp[12];
    const char* jn[3] = {"hip", "thigh", "calf"};
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++) {
            int j = mi - 1;
            float th = mm.GetStatus(cp, mi).position;
            float a = fminf(SWEEP_AMP[j],
                            fminf((th - LIM_LO[j]) * 0.9f, (LIM_HI[j] - th) * 0.9f));
            if (a < MIN_AMP) {
                printf("[WARN] CAN%d-M%d(%s) 当前角 %+.2f 贴近限位，半幅压到 %.2f rad\n",
                       cp, mi, jn[j], th, MIN_AMP);
                a = MIN_AMP;
            }
            theta_c[cp * 3 + j] = th;
            amp[cp * 3 + j] = a;
            printf("[INFO] CAN%d-M%d(%s): θ_c=%+.2f amp=%.2f (限位[%.2f,%.2f])\n",
                   cp, mi, jn[j], th, a, LIM_LO[j], LIM_HI[j]);
        }

    // 锁定目标姿态 = 各关节 θ_c（逐关节辨识时其他关节锁定到这）
    float q_hold[12];
    for (int i = 0; i < 12; i++) q_hold[i] = theta_c[i];

    auto keep_others = [&](int cp, int mi) {
        for (int c2 = 0; c2 < 4; c2++)
            for (int m2 = 1; m2 <= 3; m2++) {
                if (c2 == cp && m2 == mi) continue;
                mm.SendImpedance(c2, m2, q_hold[c2 * 3 + m2 - 1],
                                 0.0f, LOCK_KP, LOCK_KD, 0.0f);
            }
    };

    auto t0 = std::chrono::steady_clock::now();
    auto ms = [&]() -> float {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count() / 1000.0f;
    };
    struct Sample { float t, th, w, tau, v; };

    int done = 0, failed = 0;
    for (int cp = 0; cp < 4 && !g_rl_stop; cp++) {
        for (int mi = 1; mi <= 3 && !g_rl_stop; mi++) {
            int idx = cp * 3 + mi - 1;
            int j = mi - 1;
            float c = theta_c[idx], a = amp[idx];
            printf("\n===== [%d/12] CAN%d-M%d(%s) θ_c=%+.2f amp=%.2f =====\n",
                   ++done, cp, mi, jn[j], c, a);

            // ---- 先把目标关节移到 θ_c（慢速闭环斜坡，读实际位置推进）----
            {
                for (int t = 0; t < 3000 && !g_rl_stop; t++) {   // 最多 6s
                    keep_others(cp, mi);
                    float actual = mm.GetStatus(cp, mi).position;
                    float e = c - actual;
                    if (fabsf(e) < 0.005f) break;
                    float pos = clamp(actual + (e > 0 ? 1.0f : -1.0f) * MOVE_RATE * DT,
                                      LIM_LO[j], LIM_HI[j]);
                    mm.SendImpedance(cp, mi, pos, 0.0f, SWEEP_KP, SWEEP_KD, 0.0f);
                    usleep((useconds_t)(DT * 1e6f));
                }
                float after = mm.GetStatus(cp, mi).position;
                if (fabsf(after - c) > 0.05f)
                    printf("[WARN] 移到 θ_c 偏差 %.3f rad（将靠重力标定拉回）\n", after - c);
                printf("[INFO] 移到 θ_c（%+.2f）完成\n", after);
            }

            bool abort = false;

            // ---- 阶段 C：重力标定（GRID_PTS 点 × GRID_ROUNDS 遍）----
            std::vector<float> g_theta(GRID_PTS), g_tau(GRID_PTS, 0.0f);
            for (int k = 0; k < GRID_PTS; k++)
                g_theta[k] = c - a + (2.0f * a) * k / (GRID_PTS - 1);
            for (int r = 0; r < GRID_ROUNDS && !abort && !g_rl_stop; r++) {
                for (int k = 0; k < GRID_PTS && !g_rl_stop; k++) {
                    keep_others(cp, mi);
                    mm.SendImpedance(cp, mi, g_theta[k], 0.0f, SWEEP_KP, SWEEP_KD, 0.0f);
                    int hold = (int)(GRAV_HOLD_S / DT);
                    float acc = 0.0f;
                    for (int h = 0; h < hold && !g_rl_stop; h++) {
                        MotorStatus st = mm.GetStatus(cp, mi);
                        acc += st.torque;
                        if (fabsf(st.position - g_theta[k]) > POS_ERR_LIMIT) abort = true;
                        usleep((useconds_t)(DT * 1e6f));
                    }
                    g_tau[k] += acc / hold;
                }
                printf("  重力标定第 %d/%d 遍完成\n", r + 1, GRID_ROUNDS);
            }
            if (abort || g_rl_stop) {
                printf("[ABORT] 位置误差超限或用户急停（Ctrl+C），失能退出\n");
                failed++;
                break;
            }
            for (int k = 0; k < GRID_PTS; k++) g_tau[k] /= GRID_ROUNDS;
            float gmin = g_tau[0], gmax = g_tau[0];
            for (int k = 1; k < GRID_PTS; k++) {
                gmin = fminf(gmin, g_tau[k]);
                gmax = fmaxf(gmax, g_tau[k]);
            }
            printf("[阶段C] 重力标定完成：θ∈[%+.2f,%+.2f], g∈[%+.1f,%+.1f] Nm (Δg=%.1f)\n",
                   g_theta[0], g_theta[GRID_PTS - 1], gmin, gmax, gmax - gmin);
            char gp[128];
            snprintf(gp, sizeof gp, "log/fric_id/C%dM%d_grav.csv", cp, mi);
            FILE* fg = fopen(gp, "w");
            if (fg) {
                fprintf(fg, "theta,g\n");
                for (int k = 0; k < GRID_PTS; k++)
                    fprintf(fg, "%.6f,%.6f\n", g_theta[k], g_tau[k]);
                fclose(fg);
            }

            // g_est 线性插值（扫掠前馈用）
            auto g_interp = [&](float th) -> float {
                if (th <= g_theta[0]) return g_tau[0];
                if (th >= g_theta[GRID_PTS - 1]) return g_tau[GRID_PTS - 1];
                int k = 0;
                while (k < GRID_PTS - 1 && g_theta[k + 1] < th) k++;
                float t = (th - g_theta[k]) / (g_theta[k + 1] - g_theta[k]);
                return g_tau[k] + t * (g_tau[k + 1] - g_tau[k]);
            };

            // ---- 阶段 D：前馈恒速扫掠（ROUNDS 回合 × N_VEL 档 × CYC_PER_VEL 圈）----
            for (int rd = 0; rd < ROUNDS && !abort && !g_rl_stop; rd++) {
                std::vector<Sample> buf;
                buf.reserve(40000);
                for (int vi = 0; vi < N_VEL && !abort && !g_rl_stop; vi++) {
                    float v = VEL_STEPS[vi];
                    int n_scan = (int)(2.0f * a / v / DT);   // 单程步数
                    float pos = c - a;
                    float dir = +1.0f;
                    for (int cyc = 0; cyc < CYC_PER_VEL * 2 && !abort && !g_rl_stop; cyc++) {
                        for (int s = 0; s < n_scan && !g_rl_stop; s++) {
                            keep_others(cp, mi);
                            pos = clamp(pos + v * dir * DT, c - a, c + a);
                            mm.SendImpedance(cp, mi, pos, 0.0f, SWEEP_KP, SWEEP_KD,
                                             g_interp(pos));
                            MotorStatus st = mm.GetStatus(cp, mi);
                            if (fabsf(st.position - pos) > POS_ERR_LIMIT) { abort = true; break; }
                            buf.push_back({ms(), st.position, st.velocity, st.torque, v * dir});
                            usleep((useconds_t)(DT * 1e6f));
                        }
                        dir = -dir;
                    }
                    printf("  回合%d 档v=%.1f 完成（%zu 样本）\n", rd + 1, v, buf.size());
                }
                char fp[128];
                snprintf(fp, sizeof fp, "log/fric_id/C%dM%d_R%d.csv", cp, mi, rd);
                FILE* f = fopen(fp, "w");
                if (f) {
                    fprintf(f, "elapsed_ms,theta,omega,tau,v_des\n");
                    for (auto& s : buf)
                        fprintf(f, "%.1f,%.6f,%.6f,%.6f,%.3f\n",
                                s.t, s.th, s.w, s.tau, s.v);
                    fclose(f);
                    printf("  回合%d 落盘 → %s\n", rd + 1, fp);
                }
            }
            if (abort || g_rl_stop) {
                printf("[ABORT] 位置误差超限或用户急停（Ctrl+C），失能退出\n");
                failed++;
                break;
            }

            // info 元数据（离线脚本读：θ_c/amp/速度档）
            char ip[128];
            snprintf(ip, sizeof ip, "log/fric_id/C%dM%d_info.txt", cp, mi);
            FILE* fi = fopen(ip, "w");
            if (fi) {
                fprintf(fi, "theta_c=%.6f\namp=%.6f\n", c, a);
                fprintf(fi, "n_vel=%d\n", N_VEL);
                for (int vi = 0; vi < N_VEL; vi++)
                    fprintf(fi, "vel%d=%.3f\n", vi, VEL_STEPS[vi]);
                fclose(fi);
            }
            printf("===== CAN%d-M%d(%s) 完成 =====\n", cp, mi, jn[j]);
        }
        if (failed) break;
    }

    // 失能 + 停机
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 3; mi++)
            mm.DisableMotor(cp, mi);
    thread_mgr.stop_thread("motor_receive");
    thread_mgr.stop_thread("motor_send");
    mm.Stop();
    signal(SIGINT, SIG_DFL);
    if (g_rl_stop)
        printf("[INFO] 用户急停，已失能。已完成的关节数据保留在 log/fric_id/。\n");
    printf("\n[INFO] Example54 完成：%d 关节完成，%d 失败。\n", done - failed, failed);
    printf("[INFO] 数据 → log/fric_id/，离线回归：python tool/friction_id_offline.py\n");
}
