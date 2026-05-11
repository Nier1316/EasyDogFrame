/**
 * @file   motor.cpp
 * @brief  Motor 类的实现：命令构造、编码解码、状态缓存
 */
#include "motor.h"
#include <cstring>
#include <cstdio>

// 构造函数：记录静态配置（CAN 口 / 电机 ID / 收发 ID），并把状态缓存清零
Motor::Motor(uint8_t can_port, uint8_t motor_id, uint16_t rx_id, uint16_t tx_id)
    : m_can_port(can_port), m_motor_id(motor_id), m_rx_id(rx_id), m_tx_id(tx_id) {
    memset(&m_current_status, 0, sizeof(m_current_status));
    m_current_status.motor_id = motor_id;   // 状态里回填 motor_id，便于打印识别
}

Motor::~Motor() {
    // 当前无动态资源；留作将来扩展（例如内部线程、socket 等）的清理入口
}

// ------------------------------------------------------------------
// 命令接口：仅构造 MotorCommand 结构体，实际发送由 MotorManager 完成
// 这里返回 true 只表示"命令构造合法"；是否成功送达需查看 Manager 的返回值
// ------------------------------------------------------------------
bool Motor::SetSpeed(uint16_t speed) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_SET_SPEED;
    cmd.speed = speed;
    return true;
}

bool Motor::SetTorque(uint16_t torque) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_SET_TORQUE;
    cmd.torque = torque;
    return true;
}

bool Motor::SetDirection(uint8_t direction) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_SET_DIRECTION;
    cmd.direction = direction;
    return true;
}

bool Motor::Stop() {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_STOP;
    return true;
}

bool Motor::Reset() {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_RESET;
    return true;
}

bool Motor::QueryStatus() {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_QUERY_STATUS;
    return true;
}

// ------------------------------------------------------------------
// 状态查询接口：全部加锁，保证与后台线程的 UpdateStatus 不冲突
// ------------------------------------------------------------------
MotorStatus Motor::GetStatus() const {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    return m_current_status;   // 按值返回副本，调用者无需再加锁
}

// 健康判定：错误码为 0 且不处于"故障"态（state != 2）
bool Motor::IsHealthy() const {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    return m_current_status.error_code == ERR_OK && m_current_status.state != 2;
}

uint8_t Motor::GetErrorCode() const {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    return m_current_status.error_code;
}

// 把错误码映射为人类可读的中文描述，便于日志和调试
const char* Motor::GetErrorMessage() const {
    uint8_t error_code = GetErrorCode();   // 内部已加锁，这里不需要重复加锁
    switch (error_code) {
        case ERR_OK:                return "正常";
        case ERR_MOTOR_OVERHEAT:    return "电机过热";
        case ERR_MOTOR_OVERCURRENT: return "电机过流";
        case ERR_MOTOR_STALL:       return "电机堵转";
        case ERR_COMMUNICATION:     return "通信错误";
        case ERR_TIMEOUT:           return "超时";
        case ERR_INVALID_PARAM:     return "无效参数";
        case ERR_DEVICE_OFFLINE:    return "设备离线";
        default:                    return "未知错误";
    }
}

// 后台接收线程收到新帧解析后，调用本函数覆盖缓存
void Motor::UpdateStatus(const MotorStatus& status) {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    m_current_status = status;
}

// ------------------------------------------------------------------
// 命令打包：构造一个 VCI_CAN_OBJ（CAN 帧），填 ID 与 8 字节数据区
// ------------------------------------------------------------------
VCI_CAN_OBJ Motor::EncodeCommand(const MotorCommand& cmd) {
    VCI_CAN_OBJ frame;
    memset(&frame, 0, sizeof(frame));  // 清零，避免脏数据触发未预期标志

    frame.ID         = m_tx_id;        // 发送 ID：上位机 → 电机
    frame.DataLen    = 8;              // 固定使用 8 字节数据区
    frame.ExternFlag = 0;              // 标准帧（11 位 ID）
    frame.RemoteFlag = 0;              // 数据帧（非远程请求帧）

    EncodeMotorCommand(frame.Data, cmd);  // 实际填充数据区
    return frame;
}

// 从接收帧解出 MotorStatus；调用者需保证帧 ID 确实是本电机的 m_rx_id
MotorStatus Motor::DecodeStatus(const VCI_CAN_OBJ& frame) {
    MotorStatus status;
    DecodeMotorStatus(frame.Data, status);
    return status;
}

// ------------------------------------------------------------------
// 底层字节序约定（上位机与电机必须保持一致）：
//   data[0]   : motor_id
//   data[1]   : cmd_type
//   data[2..3]: speed       （大端，高字节在前）
//   data[4..5]: torque      （大端，高字节在前）
//   data[6]   : direction
//   data[7]   : 保留/填 0
// ------------------------------------------------------------------
void Motor::EncodeMotorCommand(uint8_t* data, const MotorCommand& cmd) {
    data[0] = cmd.motor_id;
    data[1] = cmd.cmd_type;
    data[2] = (cmd.speed  >> 8) & 0xFF;   // speed 高字节
    data[3] =  cmd.speed        & 0xFF;   // speed 低字节
    data[4] = (cmd.torque >> 8) & 0xFF;   // torque 高字节
    data[5] =  cmd.torque       & 0xFF;   // torque 低字节
    data[6] = cmd.direction;
    data[7] = 0;
}

// 状态帧字节序约定：
//   data[0]   : motor_id
//   data[1..2]: speed       （大端）
//   data[3..4]: torque      （大端）
//   data[5]   : temperature
//   data[6]   : error_code
//   data[7]   : state
void Motor::DecodeMotorStatus(const uint8_t* data, MotorStatus& status) {
    status.motor_id    = data[0];
    status.speed       = ((uint16_t)data[1] << 8) | data[2];
    status.torque      = ((uint16_t)data[3] << 8) | data[4];
    status.temperature = data[5];
    status.error_code  = data[6];
    status.state       = data[7];
}
