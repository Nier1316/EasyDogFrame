#ifndef MOTOR_LOGGER_H
#define MOTOR_LOGGER_H

/**
 * @file    motor_logger.h
 * @brief   电机指令与状态日志系统
 *
 * 记录内容:
 *   - 发送路径: 用户层目标值 → 逆标定后原始值
 *   - 接收路径: 电机原始反馈 → 标定后状态值
 *
 * 输出: log/ 文件夹下的 CSV 文件，文件名带时间戳。
 * 各类文件的开关见下方 LogFileSwitch（改一处重编译即生效）。
 */

#include <cstdio>
#include <cstdint>
#include <mutex>
#include <chrono>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

// =====================================================================
//  log/ 下 CSV 文件的集中开关（2026-08-21）
//  改这里（true=记录 / false=不记录），重新编译生效。
//  关闭的分类：Init 不创建对应文件、LogXXX 自动跳过，
//  磁盘占用和写入开销一并消失。
// =====================================================================
struct LogFileSwitch {
    // —— 诊断 RL（轮子乱转）的推荐配置：RECV + RL，其余关 ——
    // ⚠ 2026-08-28：SENDCAN 改回 false——SendOnce 每 1ms×16 电机写 sendcan，
    //   与 ReceiveOnce 的 LogRecv 争 MotorLogger m_mutex，是接收线程卡死嫌疑。
    static constexpr bool SEND    = false;  // send_*.csv    发送指令（需核对下发目标/kp/kd 时才开）
    static constexpr bool RECV    = true;   // recv_*.csv    接收反馈（诊断核心，保持开）
    static constexpr bool SENDCAN = false;  // sendcan_*.csv CAN 原始帧（高频写入源，默认关，查帧时才开）
    static constexpr bool XBOX    = false;  // xbox_*.csv    手柄输入（测手柄/遥操时才开）
    static constexpr bool KEY     = false;  // key_*.csv     键盘事件（键盘交互示例才开）
    static constexpr bool RL      = true;   // rl_*.csv      RL 循环诊断（qrel/action/扭矩/姿态，离线分析）
};

class MotorLogger {
public:
    static MotorLogger& GetInstance() {
        static MotorLogger instance;
        return instance;
    }

    // 初始化：创建 log/ 目录，打开两个日志文件
    void Init() {
        if (m_initialized) return;
        std::lock_guard<std::mutex> lock(m_mutex);

        // 创建 log/ 目录
        mkdir("log", 0755);

        // 生成时间戳文件名: log/send_YYYYMMDD_HHMMSS.csv 等
        char stamp[64];
        time_t now = time(nullptr);
        strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", localtime(&now));

        std::string send_path = std::string("log/send_") + stamp + ".csv";
        std::string recv_path = std::string("log/recv_") + stamp + ".csv";

        std::string sendcan_path = std::string("log/sendcan_") + stamp + ".csv";
        std::string xbox_path = std::string("log/xbox_") + stamp + ".csv";
        std::string key_path = std::string("log/key_") + stamp + ".csv";
        std::string rl_path = std::string("log/rl_") + stamp + ".csv";

        // 按 LogFileSwitch 开关打开对应文件（关掉的不创建、不记录）
        if (LogFileSwitch::SEND)     m_send_file    = fopen(send_path.c_str(), "w");
        if (LogFileSwitch::RECV)     m_recv_file    = fopen(recv_path.c_str(), "w");
        if (LogFileSwitch::SENDCAN)  m_sendcan_file = fopen(sendcan_path.c_str(), "w");
        if (LogFileSwitch::XBOX)     m_xbox_file    = fopen(xbox_path.c_str(), "w");
        if (LogFileSwitch::KEY)      m_key_file     = fopen(key_path.c_str(), "w");
        if (LogFileSwitch::RL)       m_rl_file      = fopen(rl_path.c_str(), "w");

        if (m_send_file) {
            fprintf(m_send_file,
                "elapsed_ms,can_port,motor_id,"
                "tgt_pos_cal,tgt_vel_cal,"
                "send_pos_raw,send_vel_raw,"
                "kp,kd,mode\n");
            fflush(m_send_file);
        }

        if (m_sendcan_file) {
            // 编码后、真正上线的 8 字节 CAN 数据（十六进制），便于逐帧核对
            // mode 列含义：0/1/2 = 阻抗/速度/位置控制帧（set_motor_para_bt）
            //              -1 = 参数读写帧（float2bag）
            //              -2 = 使能帧，-3 = 失能帧
            fprintf(m_sendcan_file,
                "elapsed_ms,can_port,motor_id,mode,"
                "d0,d1,d2,d3,d4,d5,d6,d7\n");
            fflush(m_sendcan_file);
        }

        if (m_recv_file) {
            fprintf(m_recv_file,
                "wall_ms,elapsed_ms,can_port,motor_id,"
                "raw_pos,raw_vel,raw_torque,"
                "cal_pos,cal_vel,cal_torque\n");
            fflush(m_recv_file);
        }

        if (m_xbox_file) {
            fprintf(m_xbox_file,
                "elapsed_ms,left_stick_x,left_stick_y,"
                "right_stick_x,right_stick_y,"
                "left_trigger,right_trigger,"
                "a,b,x,y,lb,rb,back,start,ls,rs,"
                "dpad_up,dpad_down,dpad_left,dpad_right\n");
            fflush(m_xbox_file);
        }

        if (m_key_file) {
            // 键盘按键事件日志。key_code 与 KeyDir 枚举对应：1=↑ 2=↓ 3=← 4=→ 5=q；
            // 0 为特殊事件（如 CAN_SELECT，value 存所选路号）。frame 为主循环帧号。
            fprintf(m_key_file,
                "elapsed_ms,frame,key_code,key_name,value\n");
            fflush(m_key_file);
        }

        if (m_rl_file) {
            // RL 循环诊断（每控制步一行，供离线分析策略观测/动作/扭矩）
            fprintf(m_rl_file,
                "wall_ms,elapsed_ms,step,cmd_vx,cmd_vy,cmd_wz,"
                "qrel_hFL,qrel_tFL,qrel_cFL,"
                "qrel_hFR,qrel_tFR,qrel_cFR,"
                "qrel_hRL,qrel_tRL,qrel_cRL,"
                "qrel_hRR,qrel_tRR,qrel_cRR,"
                "av_x,av_y,av_z,pgr_x,pgr_y,pgr_z,"
                "vel_00,vel_01,vel_02,vel_03,vel_04,vel_05,"
                "vel_06,vel_07,vel_08,vel_09,vel_10,vel_11,"
                "vel_12,vel_13,vel_14,vel_15,"
                "act_00,act_01,act_02,act_03,act_04,act_05,"
                "act_06,act_07,act_08,act_09,act_10,act_11,"
                "act_12,act_13,act_14,act_15,"
                "tau_w0,tau_w1,tau_w2,tau_w3\n");
            fflush(m_rl_file);
        }

        m_start_time = std::chrono::steady_clock::now();
        m_initialized = true;
        printf("[INFO] MotorLogger initialized (开关见 LogFileSwitch):\n");
        printf("   send=%s recv=%s sendcan=%s xbox=%s key=%s\n",
               LogFileSwitch::SEND    ? "ON " : "OFF",
               LogFileSwitch::RECV    ? "ON " : "OFF",
               LogFileSwitch::SENDCAN ? "ON " : "OFF",
               LogFileSwitch::XBOX    ? "ON " : "OFF",
               LogFileSwitch::KEY     ? "ON " : "OFF");
        printf("   目录: %s%s\n", send_path.c_str(),
               LogFileSwitch::SENDCAN ? "" : "  (sendcan 已关)");
    }

    // 记录发送指令: 用户目标值 → 逆标定后实际发送值
    void LogSend(uint8_t can_port, uint8_t motor_id,
                 float tgt_pos_cal, float tgt_vel_cal,
                 float send_pos_raw, float send_vel_raw,
                 float kp, float kd, int mode) {
        if (!m_initialized || !m_send_file) return;
        std::lock_guard<std::mutex> lock(m_mutex);

        int64_t elapsed = ElapsedMs();
        fprintf(m_send_file,
            "%ld,%d,%d,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%d\n",
            elapsed, can_port, motor_id,
            tgt_pos_cal, tgt_vel_cal,
            send_pos_raw, send_vel_raw,
            kp, kd, mode);
    }

    // 记录发送到电机的原始 CAN 帧: 编码后、Can_Tx 前的 8 字节
    void LogSendCan(uint8_t can_port, uint8_t motor_id, int mode,
                    const uint8_t* data) {
        if (!m_initialized || !m_sendcan_file) return;
        std::lock_guard<std::mutex> lock(m_mutex);

        int64_t elapsed = ElapsedMs();
        fprintf(m_sendcan_file,
            "%ld,%d,%d,%d,"
            "0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",
            elapsed, can_port, motor_id, mode,
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7]);
    }

    // 记录接收反馈: 原始值 → 标定后值
    void LogRecv(uint8_t can_port, uint8_t motor_id,
                 float raw_pos, float raw_vel, float raw_torque,
                 float cal_pos, float cal_vel, float cal_torque) {
        if (!m_initialized || !m_recv_file) return;
        std::lock_guard<std::mutex> lock(m_mutex);

        int64_t elapsed = ElapsedMs();
        fprintf(m_recv_file,
            "%lld,%ld,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            (long long)WallMs(), elapsed, can_port, motor_id,
            raw_pos, raw_vel, raw_torque,
            cal_pos, cal_vel, cal_torque);
    }

    // 记录手柄输入（时间戳与 send/recv 共用同一 steady_clock 基准，便于对齐排查）
    // 摇杆/扳机为归一化值：左/右摇杆 [-1,1]，扳机 [0,1]；按键 0/1。
    void LogXbox(float lx, float ly, float rx, float ry,
                 float lt, float rt,
                 int a, int b, int x, int y, int lb, int rb,
                 int back, int start, int ls, int rs,
                 int du, int dd, int dl, int dr) {
        if (!m_initialized || !m_xbox_file) return;
        std::lock_guard<std::mutex> lock(m_mutex);

        int64_t elapsed = ElapsedMs();
        fprintf(m_xbox_file,
            "%ld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
            "%d,%d,%d,%d\n",
            elapsed, lx, ly, rx, ry, lt, rt,
            a, b, x, y, lb, rb, back, start, ls, rs,
            du, dd, dl, dr);
    }

    // 记录键盘按键事件（时间戳与 send/recv 共用同一 steady_clock 基准，便于对齐排查）。
    // key_code：1=↑ 2=↓ 3=← 4=→ 5=q；0=特殊事件（如 CAN_SELECT，value 存所选路号）。
    void LogKey(int frame, int key_code, const char* key_name, int value = 0) {
        if (!m_initialized || !m_key_file) return;
        std::lock_guard<std::mutex> lock(m_mutex);

        int64_t elapsed = ElapsedMs();
        fprintf(m_key_file, "%ld,%d,%d,%s,%d\n",
                elapsed, frame, key_code, key_name, value);
    }

    // 记录 RL 循环诊断（log/rl_*.csv）：观测残差 / 动作 / 下发扭矩 / 姿态。
    // obs 为 64 维观测（rl::build_observation 输出），action 16 维，tau_wheel 4 轮扭矩。
    // 字段布局见 Init 里 rl 表头；每控制步一行（50Hz）。
    void LogRL(int step, const float* cmd, const float* obs,
               const float* action, const float* tau_wheel) {
        if (!m_initialized || !m_rl_file) return;
        std::lock_guard<std::mutex> lock(m_mutex);

        int64_t elapsed = ElapsedMs();
        fprintf(m_rl_file, "%lld,%ld,%d,%.3f,%.3f,%.3f,",
                (long long)WallMs(), elapsed, step, cmd[0], cmd[1], cmd[2]);
        // joint_pos_rel[12] = obs[9..20]
        for (int i = 0; i < 12; i++) fprintf(m_rl_file, "%.4f,", obs[9 + i]);
        // base_ang_vel[3] = obs[3..5], projected_gravity[3] = obs[6..8]
        fprintf(m_rl_file, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",
                obs[3], obs[4], obs[5], obs[6], obs[7], obs[8]);
        // joint_vel[16] = obs[21..36]
        for (int i = 0; i < 16; i++) fprintf(m_rl_file, "%.4f,", obs[21 + i]);
        // action[16]
        for (int i = 0; i < 16; i++) fprintf(m_rl_file, "%.4f,", action[i]);
        // tau_wheel[4]
        fprintf(m_rl_file, "%.4f,%.4f,%.4f,%.4f\n",
                tau_wheel[0], tau_wheel[1], tau_wheel[2], tau_wheel[3]);
    }

    // 定期刷新缓冲区
    void Flush() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_send_file) fflush(m_send_file);
        if (m_recv_file) fflush(m_recv_file);
        if (m_sendcan_file) fflush(m_sendcan_file);
        if (m_xbox_file) fflush(m_xbox_file);
        if (m_key_file) fflush(m_key_file);
        if (m_rl_file) fflush(m_rl_file);
    }

    // 关闭日志文件
    void Shutdown() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_send_file) { fclose(m_send_file); m_send_file = nullptr; }
        if (m_recv_file) { fclose(m_recv_file); m_recv_file = nullptr; }
        if (m_sendcan_file) { fclose(m_sendcan_file); m_sendcan_file = nullptr; }
        if (m_xbox_file) { fclose(m_xbox_file); m_xbox_file = nullptr; }
        if (m_key_file) { fclose(m_key_file); m_key_file = nullptr; }
        if (m_rl_file) { fclose(m_rl_file); m_rl_file = nullptr; }
        m_initialized = false;
    }

    ~MotorLogger() { Shutdown(); }

private:
    MotorLogger() = default;

    int64_t ElapsedMs() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            now - m_start_time).count();
    }

    // 系统墙钟时间戳（epoch ms），用于 sim2real 对比时间对齐
    //（sim2sim --record 也记 wall_ms；两边 wall_ms 直接可对齐）
    static int64_t WallMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    std::mutex m_mutex;
    FILE* m_send_file = nullptr;
    FILE* m_recv_file = nullptr;
    FILE* m_sendcan_file = nullptr;
    FILE* m_xbox_file = nullptr;
    FILE* m_key_file = nullptr;
    FILE* m_rl_file = nullptr;
    std::chrono::steady_clock::time_point m_start_time;
    bool m_initialized = false;
};

#endif // MOTOR_LOGGER_H
