/**
 * @file    imu_device.h
 * @brief   维特 HWT606 IMU 串口读取（WIT 标准协议）
 *
 * 通过 TTL 串口读取角速度（0x52 帧）与四元数（0x59 帧），
 * 供 RL 观测构建使用（base_ang_vel / projected_gravity）。
 *
 * 约定：
 *  - gyro 输出统一为机体系角速度，单位 rad/s。
 *  - quat 输出 [w, x, y, z]（body 相对 world 的姿态四元数）。
 *  - 模块应预先配好：RSW 开 GYRO+QUATER、RRATE=100Hz、AXIS6=6 轴、
 *    BAUD=115200（可用维特上位机软件，或用 Configure() 写寄存器）。
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

/** IMU 安装方向 */
enum class ImuMount {
    Z_UP = 0,      // 默认：Z 轴朝上，无变换
    Z_DOWN_X = 1,  // Z 轴朝下，绕 X 轴翻面（X 不变，Y/Z 反向）
};

class ImuDevice {
public:
    ImuDevice() = default;
    ~ImuDevice();

    ImuDevice(const ImuDevice&) = delete;
    ImuDevice& operator=(const ImuDevice&) = delete;

    /** @brief 设置安装方向（在 Initialize 前后均可调用，影响 GetGyro/GetQuat 输出） */
    void SetMount(ImuMount mount) { m_mount = mount; }
    ImuMount GetMount() const { return m_mount; }

    /**
     * @brief 打开串口并启动读线程
     * @param port 串口设备路径（如 /dev/ttyUSB0）
     * @param baud 波特率（默认 115200）
     * @return 成功返回 true
     */
    bool Initialize(const char* port, int baud = 115200);

    /** @brief 关闭串口、停止读线程 */
    void Shutdown();

    bool IsConnected() const { return m_connected.load(); }

    /** @brief 读最新机体系角速度 [rad/s] */
    void GetGyro(float& gx, float& gy, float& gz) const;
    /** @brief 读最新四元数 [w, x, y, z] */
    void GetQuat(float& w, float& x, float& y, float& z) const;

    /**
     * @brief 写寄存器配置（RSW/RRATE/AXIS6/BAUD），返回是否全部成功。
     *        修改 BAUD 会令模块切换波特率，调用后需用新波特率重新 Initialize。
     */
    bool Configure();

private:
    void ReadLoop();
    void ParseByte(uint8_t b);
    void ParseGyroFrame(const uint8_t* d);
    void ParseQuatFrame(const uint8_t* d);
    bool WriteReg(uint8_t addr, uint16_t value);  // 解锁 + 写 + 保存

    int m_fd = -1;
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_connected{false};

    // 帧状态机缓冲
    uint8_t m_buf[11];
    int m_buf_len = 0;

    // 最新数据（读线程写，控制线程读）
    mutable std::mutex m_mutex;
    float m_gyro[3] = {0.0f, 0.0f, 0.0f};
    float m_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    ImuMount m_mount = ImuMount::Z_UP;
};
