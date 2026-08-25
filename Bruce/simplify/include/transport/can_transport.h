/**
 * @file    can_transport.h
 * @brief   CAN 传输层抽象接口（L1）
 * @details 定义后端无关的 CAN 收发接口。上层（MotorManager 等）只依赖本接口，
 *          具体实现按硬件选择：
 *            - CanetTransport  ：CANET TCP（现状，默认）
 *            - Usb2CanTransport：USB2CAN ttyACM（达妙模块，协议见 tool/usb2can_probe）
 *            - SocketCanTransport：Linux canX（预留）
 *          换硬件不动上层：open/send/sendBatch/recv/close 语义一致。
 */
#ifndef CAN_TRANSPORT_H_
#define CAN_TRANSPORT_H_

#include <cstdint>
#include <vector>
#include "common/types.h"

/**
 * @brief 后端无关的传输配置（各实现按需取字段）
 * - CANET 后端：tcp_ip / tcp_port / tcp_mode
 * - USB2CAN 后端：usb_dev（如 "/dev/ttyACM0"）
 */
struct TransportConfig {
    uint8_t  device_idx = 0;   // 逻辑 CAN 通道索引（0~3）

    // CANET TCP 后端字段
    const char* tcp_ip   = nullptr;   // 远端 IP（客户端模式）
    uint16_t    tcp_port = 0;         // 端口
    uint8_t     tcp_mode = 0;         // TCP_CLIENT=0 / TCP_SERVER=1

    // USB2CAN 后端字段（预留）
    const char* usb_dev  = nullptr;   // USB 设备路径
};

/**
 * @brief CAN 传输层纯虚接口
 */
class CanTransport {
public:
    virtual ~CanTransport() = default;

    /** 打开并启动指定通道；cfg 后端相关字段见 TransportConfig */
    virtual bool open(uint8_t idx, const TransportConfig& cfg) = 0;

    /** 发送单帧 */
    virtual bool send(uint8_t idx, const CanFrame& f) = 0;

    /** 批量发送（后端可合并传输，避开逐帧延迟地板） */
    virtual bool sendBatch(uint8_t idx, const CanFrame* f, int n) = 0;

    /** 阻塞接收；有帧返回 true，超时返回 false */
    virtual bool recv(uint8_t idx, std::vector<CanFrame>& out, int timeout_ms) = 0;

    /** 停止并关闭通道 */
    virtual bool close(uint8_t idx) = 0;

    /** 关闭所有通道并释放资源 */
    virtual void shutdown() = 0;
};

#endif // CAN_TRANSPORT_H_
