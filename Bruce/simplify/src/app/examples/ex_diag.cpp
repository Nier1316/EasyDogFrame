// ===== 诊断/只读示例 24,26-29,33,34 =====
// 由 src/app/example.cpp 拆分而来（阶段3：示例拆包），公共 helper 见 app/examples_common.h
#include "app/examples/ex_diag.h"
#include "app/examples_common.h"
#include "transport/canet_transport.h"
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

using logctl::LogCat;
#include "strategy/imu_device.h"
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

// ================= 示例 25：RL 策略控制（dogurdf sim2real 部署） =================
// 部署 dogurdf_sim2sim_deploy 的 64 维观测 / 16 维动作策略到真机，控制 50 Hz。
// 注意：零位对齐暂用占位值（rl::DEFAULT_POSE），起立仍用真机实测站立指令角
// （STAND_*），避免起立到错误姿态。策略输出当前仅用于验证链路。
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
