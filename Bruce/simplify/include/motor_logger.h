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
 * 输出: log/ 文件夹下的 CSV 文件，文件名带时间戳
 */

#include <cstdio>
#include <cstdint>
#include <mutex>
#include <chrono>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

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

        m_send_file = fopen(send_path.c_str(), "w");
        m_recv_file = fopen(recv_path.c_str(), "w");
        m_sendcan_file = fopen(sendcan_path.c_str(), "w");
        m_xbox_file = fopen(xbox_path.c_str(), "w");
        m_key_file = fopen(key_path.c_str(), "w");

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
                "elapsed_ms,can_port,motor_id,"
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

        m_start_time = std::chrono::steady_clock::now();
        m_initialized = true;
        printf("[INFO] MotorLogger initialized: %s, %s, %s, %s, %s\n",
               send_path.c_str(), recv_path.c_str(), sendcan_path.c_str(),
               xbox_path.c_str(), key_path.c_str());
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
            "%ld,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            elapsed, can_port, motor_id,
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

    // 定期刷新缓冲区
    void Flush() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_send_file) fflush(m_send_file);
        if (m_recv_file) fflush(m_recv_file);
        if (m_sendcan_file) fflush(m_sendcan_file);
        if (m_xbox_file) fflush(m_xbox_file);
        if (m_key_file) fflush(m_key_file);
    }

    // 关闭日志文件
    void Shutdown() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_send_file) { fclose(m_send_file); m_send_file = nullptr; }
        if (m_recv_file) { fclose(m_recv_file); m_recv_file = nullptr; }
        if (m_sendcan_file) { fclose(m_sendcan_file); m_sendcan_file = nullptr; }
        if (m_xbox_file) { fclose(m_xbox_file); m_xbox_file = nullptr; }
        if (m_key_file) { fclose(m_key_file); m_key_file = nullptr; }
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

    std::mutex m_mutex;
    FILE* m_send_file = nullptr;
    FILE* m_recv_file = nullptr;
    FILE* m_sendcan_file = nullptr;
    FILE* m_xbox_file = nullptr;
    FILE* m_key_file = nullptr;
    std::chrono::steady_clock::time_point m_start_time;
    bool m_initialized = false;
};

#endif // MOTOR_LOGGER_H
