/**
 * @file   canet_transport.cpp
 * @brief  CANET TCP 传输层实现
 *         基于 CanDevice，提供 CANET TCP 的统一 CAN 收发接口
 */
#include "transport/canet_transport.h"
#include "transport/can_device.h"
#include <cstdio>
#include <cstring>

// ============ 单例实现 ============

CanetTransport& CanetTransport::GetInstance() {
    static CanetTransport instance;
    return instance;
}

CanetTransport::CanetTransport() {
}

CanetTransport::~CanetTransport() {
    ShutdownAll();
}

// ============ 设备生命周期 ============

bool CanetTransport::InitDevice(uint8_t device_idx, const CanDeviceConfig& config) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_devices.find(device_idx) != m_devices.end()) {
        printf("[WARNING] Device %d already initialized\n", device_idx);
        return false;
    }

    auto device = std::make_unique<CanDevice>(device_idx);
    if (!device->Initialize(config)) {
        printf("[ERROR] Failed to initialize device %d\n", device_idx);
        return false;
    }

    m_devices[device_idx] = std::move(device);
    printf("[INFO] BSP: Device %d initialized\n", device_idx);
    return true;
}

bool CanetTransport::StartDevice(uint8_t device_idx) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    if (!it->second->Start()) {
        printf("[ERROR] Failed to start device %d\n", device_idx);
        return false;
    }

    printf("[INFO] BSP: Device %d started\n", device_idx);
    return true;
}

bool CanetTransport::StopDevice(uint8_t device_idx) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    if (!it->second->Stop()) {
        printf("[ERROR] Failed to stop device %d\n", device_idx);
        return false;
    }

    printf("[INFO] BSP: Device %d stopped\n", device_idx);
    return true;
}

bool CanetTransport::CloseDevice(uint8_t device_idx) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    it->second->Shutdown();
    m_devices.erase(it);
    printf("[INFO] BSP: Device %d closed\n", device_idx);
    return true;
}

// ============ 私有辅助函数：格式转换 ============

/**
 * 将 BspCanFrame 转换为 VCI_CAN_OBJ
 * 这个函数封装了应用层格式到硬件库格式的转换逻辑
 */
void CanetTransport::ConvertFrameToVci(const BspCanFrame& frame, VCI_CAN_OBJ& vci_frame) {
    memset(&vci_frame, 0, sizeof(vci_frame));
    vci_frame.ID = frame.id;
    vci_frame.DataLen = frame.dlc;
    vci_frame.ExternFlag = frame.is_extended;
    vci_frame.RemoteFlag = 0;
    memcpy(vci_frame.Data, frame.data, 8);
}

/**
 * 将 VCI_CAN_OBJ 转换为 BspCanFrame
 * 这个函数封装了硬件库格式到应用层格式的转换逻辑
 */
void CanetTransport::ConvertFrameFromVci(const VCI_CAN_OBJ& vci_frame, BspCanFrame& frame) {
    frame.id = vci_frame.ID;
    frame.dlc = vci_frame.DataLen;
    frame.is_extended = vci_frame.ExternFlag;
    memcpy(frame.data, vci_frame.Data, 8);
}

// ============ 数据收发 ============

bool CanetTransport::SendFrame(uint8_t device_idx, const BspCanFrame& frame) {
    // 只在查 map 时短暂持锁，VCI_Transmit 调用交给 CanDevice 自己的发送锁。
    // 为什么：若这里整段持 m_mutex，接收线程在 VCI_Receive 阻塞 10ms 时
    // 会把发送线程也卡住（实测 40ms 的根源）。SDK 允许收发并行（并发实测 0.005ms）。
    // 安全性：m_devices 仅在 InitDevice/CloseDevice 增删，且都发生在
    // 收发线程停止之后（先 stop_thread 再 MotorManager::Stop），稳态下无并发改 map。
    CanDevice* dev;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_devices.find(device_idx);
        if (it == m_devices.end()) {
            printf("[ERROR] Device %d not found\n", device_idx);
            return false;
        }
        dev = it->second.get();
    }

    // 使用转换函数：BspCanFrame → VCI_CAN_OBJ
    VCI_CAN_OBJ vci_frame;
    ConvertFrameToVci(frame, vci_frame);

    if (!dev->SendFrame(vci_frame)) {
        printf("[ERROR] Failed to send frame on device %d\n", device_idx);
        return false;
    }

    return true;
}

bool CanetTransport::Can_Tx(uint8_t device_idx, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    BspCanFrame frame;
    frame.id = can_id;
    frame.dlc = dlc;
    frame.is_extended = 0;
    memcpy(frame.data, data, 8);
    return SendFrame(device_idx, frame);
}

bool CanetTransport::SendFramesBatch(uint8_t device_idx, const std::vector<BspCanFrame>& frames) {
    // 同 SendFrame：短暂查 map，VCI_Transmit 交给 CanDevice 自己的发送锁。
    CanDevice* dev;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_devices.find(device_idx);
        if (it == m_devices.end()) {
            printf("[ERROR] Device %d not found\n", device_idx);
            return false;
        }
        dev = it->second.get();
    }
    if (frames.empty()) return true;

    // 转成 VCI_CAN_OBJ 数组，一次 VCI_Transmit 发出去
    VCI_CAN_OBJ vci[16];
    int n = (int)frames.size();
    if (n > 16) n = 16;
    for (int i = 0; i < n; i++)
        ConvertFrameToVci(frames[i], vci[i]);

    return dev->SendFrames(vci, n);
}

bool CanetTransport::ReceiveFrames(uint8_t device_idx,
                           std::vector<BspCanFrame>& frames,
                           int timeout_ms) {
    // 同 SendFrame：短暂查 map，VCI_Receive（~10ms 阻塞）不持 CanetTransport 锁，
    // 否则会卡住发送线程。安全性见 SendFrame 注释（线程先停再关设备）。
    CanDevice* dev;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_devices.find(device_idx);
        if (it == m_devices.end()) {
            printf("[ERROR] Device %d not found\n", device_idx);
            return false;
        }
        dev = it->second.get();
    }

    std::vector<VCI_CAN_OBJ> vci_frames;
    if (!dev->ReceiveFrames(vci_frames, timeout_ms)) {
        return false;
    }

    // 使用转换函数：VCI_CAN_OBJ → BspCanFrame
    frames.clear();
    for (const auto& vci_frame : vci_frames) {
        BspCanFrame frame;
        ConvertFrameFromVci(vci_frame, frame);
        frames.push_back(frame);
    }

    return true;
}

// ============ 全局控制 ============

void CanetTransport::ShutdownAll() {
    std::lock_guard<std::mutex> lock(m_mutex);

    for (auto& pair : m_devices) {
        pair.second->Shutdown();
    }
    m_devices.clear();
    printf("[INFO] BSP: All devices shutdown\n");
}

// ============ CanTransport 接口实现 ============

bool CanetTransport::open(uint8_t idx, const TransportConfig& cfg) {
    CanDeviceConfig dev_cfg;
    dev_cfg.device_idx = idx;
    dev_cfg.port       = cfg.tcp_port;
    dev_cfg.server_ip  = cfg.tcp_ip;
    dev_cfg.work_mode  = cfg.tcp_mode;
    if (!InitDevice(idx, dev_cfg)) return false;
    return StartDevice(idx);
}

bool CanetTransport::send(uint8_t idx, const CanFrame& f) {
    return SendFrame(idx, f);
}

bool CanetTransport::sendBatch(uint8_t idx, const CanFrame* f, int n) {
    if (n <= 0 || f == nullptr) return true;
    std::vector<BspCanFrame> frames(f, f + n);
    return SendFramesBatch(idx, frames);
}

bool CanetTransport::recv(uint8_t idx, std::vector<CanFrame>& out, int timeout_ms) {
    return ReceiveFrames(idx, out, timeout_ms);
}

bool CanetTransport::close(uint8_t idx) {
    bool ok = StopDevice(idx);
    ok = CloseDevice(idx) && ok;
    return ok;
}

void CanetTransport::shutdown() {
    ShutdownAll();
}
