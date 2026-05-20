/**
 * @file    can_device.h
 * @brief   CANET 设备封装类声明
 * @details CanDevice 对 CANET 库的 VCI_* 系列 C 风格 API 做了面向对象封装：
 *          - 构造时绑定一个 CANET 设备索引（can0~can3）
 *          - Initialize() 完成打开设备 + 设置 TCP 参数 + InitCAN
 *          - Start()/Stop() 控制 CAN 通道的启停
 *          - SendFrame/ReceiveFrames 做单帧/批量收发
 *          所有公共方法加锁，可跨线程安全调用。
 */
#ifndef CAN_DEVICE_H_
#define CAN_DEVICE_H_

#include <cstdint>
#include <vector>
#include <mutex>
#include <thread>
#include "CANET.h"           // VCI_* API、VCI_CAN_OBJ、STATUS_OK 等
#include "data_types.h"      // CanDeviceConfig

class CanDevice {
public:
    /**
     * @param device_idx CANET 设备索引（0~3，对应 can0~can3）
     */
    CanDevice(uint8_t device_idx);
    ~CanDevice();   // 析构时自动 Shutdown，避免资源泄漏

    // ---------------- 生命周期管理 ----------------
    bool Initialize(const CanDeviceConfig& config);  // 打开 + 配置 + InitCAN
    bool Start();                                     // VCI_StartCAN
    bool Stop();                                      // VCI_ResetCAN
    void Shutdown();                                  // Stop + CloseDevice，整体收尾

    // ---------------- 数据收发 ----------------
    // 发送单个 CAN 帧，返回是否成功送入底层发送队列
    bool SendFrame(const VCI_CAN_OBJ& frame);
    // 批量接收 CAN 帧；timeout_ms 为阻塞等待时间（毫秒）
    bool ReceiveFrames(std::vector<VCI_CAN_OBJ>& frames, int timeout_ms = 100);

    // ---------------- 状态查询 ----------------
    bool     IsRunning() const { return m_is_running; }              // 是否已 Start
    uint32_t GetReceivedFrameCount() const { return m_received_count; } // 累计接收帧数
    uint8_t  GetDeviceIndex() const { return m_device_idx; }         // 设备索引

private:
    uint8_t  m_device_idx;       // CANET 设备索引（0~3）
    bool     m_is_running;       // 当前是否处于运行态（已 Start 未 Stop）
    bool     m_is_opened;        // 设备是否已成功打开（防止关闭未打开的设备）
    uint32_t m_received_count;   // 累计接收帧计数器（统计用）
    mutable std::mutex m_mutex;  // 保护所有公共方法的并发访问

    // ---------------- 对 CANET 库的底层调用封装 ----------------
    bool OpenDevice();                                   // VCI_OpenDevice
    bool CloseDevice();                                  // VCI_CloseDevice
    bool ConfigureDevice(const CanDeviceConfig& config); // 设置 TCP 模式/IP/端口 + InitCAN
};

#endif // CAN_DEVICE_H_
