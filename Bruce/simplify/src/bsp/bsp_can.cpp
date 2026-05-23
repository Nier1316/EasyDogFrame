/**
 * @file   bsp_can.cpp
 * @brief  CAN 硬件抽象层（BSP）实现
 *         基于现有 CanDevice，提供统一的 CAN 收发接口
 */
#include "bsp/bsp_can.h"
#include "can_device.h"
#include <cstdio>
#include <cstring>

/**
 * @class CanDeviceWrapper
 * @brief 对 CanDevice 的包装，用于 BspCan 内部管理
 */
class BspCan::CanDeviceWrapper {
public:
    CanDeviceWrapper(uint8_t device_idx)
        : m_device(device_idx), m_sent_count(0) {}

    CanDevice m_device;
    uint32_t m_sent_count;
};

// ============ 单例实现 ============

BspCan& BspCan::GetInstance() {
    static BspCan instance;
    return instance;
}

BspCan::BspCan() {
}

BspCan::~BspCan() {
    ShutdownAll();
}

// ============ 设备生命周期 ============

bool BspCan::InitDevice(uint8_t device_idx, const CanDeviceConfig& config) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_devices.find(device_idx) != m_devices.end()) {
        printf("[WARNING] Device %d already initialized\n", device_idx);
        return false;
    }

    auto wrapper = std::make_unique<CanDeviceWrapper>(device_idx);
    if (!wrapper->m_device.Initialize(config)) {
        printf("[ERROR] Failed to initialize device %d\n", device_idx);
        return false;
    }

    m_devices[device_idx] = std::move(wrapper);
    printf("[INFO] BSP: Device %d initialized\n", device_idx);
    return true;
}

bool BspCan::StartDevice(uint8_t device_idx) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    if (!it->second->m_device.Start()) {
        printf("[ERROR] Failed to start device %d\n", device_idx);
        return false;
    }

    printf("[INFO] BSP: Device %d started\n", device_idx);
    return true;
}

bool BspCan::StopDevice(uint8_t device_idx) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    if (!it->second->m_device.Stop()) {
        printf("[ERROR] Failed to stop device %d\n", device_idx);
        return false;
    }

    printf("[INFO] BSP: Device %d stopped\n", device_idx);
    return true;
}

bool BspCan::CloseDevice(uint8_t device_idx) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    it->second->m_device.Shutdown();
    m_devices.erase(it);
    printf("[INFO] BSP: Device %d closed\n", device_idx);
    return true;
}

bool BspCan::IsDeviceRunning(uint8_t device_idx) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        return false;
    }

    return it->second->m_device.IsRunning();
}

// ============ 私有辅助函数：格式转换 ============

/**
 * 将 BspCanFrame 转换为 VCI_CAN_OBJ
 * 这个函数封装了应用层格式到硬件库格式的转换逻辑
 */
void BspCan::ConvertFrameToVci(const BspCanFrame& frame, VCI_CAN_OBJ& vci_frame) {
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
void BspCan::ConvertFrameFromVci(const VCI_CAN_OBJ& vci_frame, BspCanFrame& frame) {
    frame.id = vci_frame.ID;
    frame.dlc = vci_frame.DataLen;
    frame.is_extended = vci_frame.ExternFlag;
    memcpy(frame.data, vci_frame.Data, 8);
}

// ============ 数据收发 ============

bool BspCan::SendFrame(uint8_t device_idx, const BspCanFrame& frame) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    // 使用转换函数：BspCanFrame → VCI_CAN_OBJ
    VCI_CAN_OBJ vci_frame;
    ConvertFrameToVci(frame, vci_frame);

    if (!it->second->m_device.SendFrame(vci_frame)) {
        printf("[ERROR] Failed to send frame on device %d\n", device_idx);
        return false;
    }

    it->second->m_sent_count++;
    return true;
}

bool BspCan::Can_Tx(uint8_t device_idx, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    BspCanFrame frame;
    frame.id = can_id;
    frame.dlc = dlc;
    frame.is_extended = 0;
    memcpy(frame.data, data, 8);
    return SendFrame(device_idx, frame);
}

bool BspCan::ReceiveFrames(uint8_t device_idx,
                           std::vector<BspCanFrame>& frames,
                           int timeout_ms) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        printf("[ERROR] Device %d not found\n", device_idx);
        return false;
    }

    std::vector<VCI_CAN_OBJ> vci_frames;
    if (!it->second->m_device.ReceiveFrames(vci_frames, timeout_ms)) {
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

// ============ 状态查询 ============

uint32_t BspCan::GetReceivedFrameCount(uint8_t device_idx) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        return 0;
    }

    return it->second->m_device.GetReceivedFrameCount();
}

uint32_t BspCan::GetSentFrameCount(uint8_t device_idx) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_devices.find(device_idx);
    if (it == m_devices.end()) {
        return 0;
    }

    return it->second->m_sent_count;
}

// ============ 全局控制 ============

bool BspCan::InitAllDevices(const std::vector<CanDeviceConfig>& configs) {
    for (size_t i = 0; i < configs.size(); i++) {
        if (!InitDevice(i, configs[i])) {
            printf("[ERROR] Failed to initialize device %zu\n", i);
            return false;
        }
    }
    return true;
}

bool BspCan::StartAllDevices() {
    std::lock_guard<std::mutex> lock(m_mutex);

    for (auto& pair : m_devices) {
        if (!pair.second->m_device.Start()) {
            printf("[ERROR] Failed to start device %d\n", pair.first);
            return false;
        }
    }
    printf("[INFO] BSP: All devices started\n");
    return true;
}

bool BspCan::StopAllDevices() {
    std::lock_guard<std::mutex> lock(m_mutex);

    for (auto& pair : m_devices) {
        if (!pair.second->m_device.Stop()) {
            printf("[ERROR] Failed to stop device %d\n", pair.first);
            return false;
        }
    }
    printf("[INFO] BSP: All devices stopped\n");
    return true;
}

void BspCan::ShutdownAll() {
    std::lock_guard<std::mutex> lock(m_mutex);

    for (auto& pair : m_devices) {
        pair.second->m_device.Shutdown();
    }
    m_devices.clear();
    printf("[INFO] BSP: All devices shutdown\n");
}
