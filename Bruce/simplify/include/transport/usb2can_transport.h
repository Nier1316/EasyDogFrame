/**
 * @file    usb2can_transport.h
 * @brief   达妙 USB2FDCAN 传输后端（L1，官方 SDK，支持多双路模块）
 * @details 通过达妙官方 DM Device SDK 驱动 N 个双通道模块，覆盖 4 路 CAN：
 *          - 逻辑路 idx → 物理 (设备索引 = idx/2, 通道 = idx%2)
 *          - 例：2 个双路模块 → CAN0=设备0ch0, CAN1=设备0ch1, CAN2=设备1ch0, CAN3=设备1ch1
 *          - dmcan_find_devices 只调一次（SDK 重复 find 会触发 device_finder 析构崩溃）
 *          - send   : dmcan_device_send_can(dev, ch, ...)
 *          - recv   : 回调按 (handle→设备索引, frame->head.channel) 定位逻辑路，推入队列
 *          - 依赖   : libdm_device.so + 新版 libusb/libstdc++（conda）
 *          ⚠ SDK context_destroy 有析构 bug，shutdown 仅 device_close，不 destroy context。
 */
#ifndef USB2CAN_TRANSPORT_H_
#define USB2CAN_TRANSPORT_H_

#include <cstdint>
#include <deque>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "transport/can_transport.h"
#include "damiao_sdk/dmcan.h"

class Usb2CanTransport : public CanTransport {
public:
    static Usb2CanTransport& GetInstance();

    // 诊断：SDK 回调收到的总帧数（onRecv 递增）。调用方据此判断"回调是否还在触发"，
    // 与 ReceiveOnce 心跳对比可区分：回调停（SDK 接收挂）vs 接收线程卡死（回调正常）。
    static std::atomic<uint64_t>& RxCount() { static std::atomic<uint64_t> c{0}; return c; }

    bool open(uint8_t idx, const TransportConfig& cfg) override;
    bool send(uint8_t idx, const CanFrame& f) override;
    bool sendBatch(uint8_t idx, const CanFrame* f, int n) override;
    bool recv(uint8_t idx, std::vector<CanFrame>& out, int timeout_ms) override;
    bool close(uint8_t idx) override;
    void shutdown() override;

private:
    struct Channel {
        uint8_t dev_idx = 0;   // 物理设备索引（idx/2）
        uint8_t ch = 0;        // 物理通道（idx%2，双路模块 0/1）
        std::deque<CanFrame> rx;
    };

    /** 打开物理设备 dev_idx（find 仅一次 + open + 双通道使能 + 波特率 + 回调） */
    bool ensureDevice(uint8_t dev_idx, const TransportConfig& cfg);

    static void onRecv(struct dmcan_device_handle* handle, usb_rx_frame_t* frame);
    void pushFrame(uint8_t idx, const CanFrame& f);

    Usb2CanTransport() = default;
    ~Usb2CanTransport();

    Usb2CanTransport(const Usb2CanTransport&) = delete;
    Usb2CanTransport& operator=(const Usb2CanTransport&) = delete;

    struct dmcan_context* m_ctx = nullptr;
    bool m_found = false;                  // find_devices 只调一次标志
    int  m_dev_count = 0;                  // 已发现设备数
    std::map<uint8_t, struct dmcan_device_handle*> m_devices;           // 设备索引 → handle
    std::map<struct dmcan_device_handle*, uint8_t> m_handle_to_devidx;  // handle → 设备索引
    std::map<uint8_t, Channel> m_channels; // 逻辑 idx → 设备+通道+队列
    std::mutex m_mutex;
    std::condition_variable m_cv;
};

#endif // USB2CAN_TRANSPORT_H_
