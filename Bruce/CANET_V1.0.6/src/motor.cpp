/**
 * @file   motor.cpp
 * @brief  Motor 类的实现：命令构造、编码解码、状态缓存
 */
#include "motor.h"
#include "motor_param_validator.h"
#include <cstring>
#include <cstdio>

// 数据转换函数（从 motor_rw_api.c 移植）
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((x - offset) * (float)((unsigned int)(1 << bits) - 1) / span);
}

// 构造函数：记录静态配置（CAN 口 / 电机 ID / 收发 ID），并把状态缓存清零
Motor::Motor(uint8_t can_port, uint8_t motor_id, uint16_t rx_id, uint16_t tx_id)
    : m_can_port(can_port), m_motor_id(motor_id), m_rx_id(rx_id), m_tx_id(tx_id) {
    m_current_status = MotorStatus();  // 使用默认构造函数初始化
    m_current_status.motor_id = motor_id;   // 状态里回填 motor_id，便于打印识别
}

Motor::~Motor() {
    // 当前无动态资源；留作将来扩展（例如内部线程、socket 等）的清理入口
}

// ------------------------------------------------------------------
// 命令接口：仅构造 MotorCommand 结构体，实际发送由 MotorManager 完成
// ------------------------------------------------------------------

// 特殊指令
bool Motor::Enable() {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_ENABLE;
    return true;
}

bool Motor::Disable() {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_DISABLE;
    return true;
}

bool Motor::SetZero() {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_SET_ZERO;
    return true;
}

bool Motor::ClearError() {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_CLEAR_ERROR;
    return true;
}

bool Motor::SetControlMode(ControlMode mode) {
    // 控制模式通过后续的控制指令隐含设置，这里仅记录
    return true;
}

// 控制指令
bool Motor::ImpedanceControl(float pos, float vel, float kp, float kd, float torque) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_IMPEDANCE_CTRL;
    cmd.mode = IMPEDANCE;

    // 参数验证和截断
    cmd.pos = MotorParamValidator::ValidatePosition(pos);
    cmd.vel = MotorParamValidator::ValidateVelocity(vel);
    cmd.kp = MotorParamValidator::ValidateKp(kp);
    cmd.kd = MotorParamValidator::ValidateKd(kd);
    cmd.torque = MotorParamValidator::ValidateTorque(torque);

    return true;
}

bool Motor::SpeedControl(float vel, float kp, float ki) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_SPEED_CTRL;
    cmd.mode = SPEED;

    // 参数验证和截断
    cmd.vel = MotorParamValidator::ValidateVelocity(vel);
    cmd.kp_speed = MotorParamValidator::ValidateKp(kp);
    cmd.ki_speed = MotorParamValidator::ValidateKi(ki);

    return true;
}

bool Motor::PositionControl(float pos, float kvp, float kp, float kd, float kvi) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_POSITION_CTRL;
    cmd.mode = POSITION;

    // 参数验证和截断
    cmd.pos = MotorParamValidator::ValidatePosition(pos);
    cmd.kp_speed = MotorParamValidator::ValidateKp(kvp);
    cmd.kp = MotorParamValidator::ValidateKp(kp);
    cmd.kd = MotorParamValidator::ValidateKd(kd);
    cmd.ki_speed = MotorParamValidator::ValidateKi(kvi);

    return true;
}

// 参数读写
bool Motor::ReadParam(uint8_t param_type) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_READ_PARAM;
    cmd.param_type = param_type;
    cmd.param_rw = 0;
    return true;
}

bool Motor::WriteParam(uint8_t param_type, float value) {
    MotorCommand cmd;
    cmd.motor_id = m_motor_id;
    cmd.cmd_type = CMD_WRITE_PARAM;
    cmd.param_type = param_type;
    cmd.param_value = value;
    cmd.param_rw = 1;
    return true;
}

// ------------------------------------------------------------------
// 状态查询接口：全部加锁，保证与后台线程的 UpdateStatus 不冲突
// ------------------------------------------------------------------
MotorStatus Motor::GetStatus() const {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    return m_current_status;   // 按值返回副本，调用者无需再加锁
}

// 健康判定：错误码为 0 且未故障
bool Motor::IsHealthy() const {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    return m_current_status.error_code == ERR_OK && !m_current_status.fault;
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
// 命令打包：构造一个 BspCanFrame（CAN 帧），填 ID 与 8 字节数据区
// ------------------------------------------------------------------
BspCanFrame Motor::EncodeCommand(const MotorCommand& cmd) {
    BspCanFrame frame;
    memset(&frame, 0, sizeof(frame));  // 清零，避免脏数据触发未预期标志

    frame.id = m_tx_id;                // 发送 ID：上位机 → 电机
    frame.dlc = 8;                     // 固定使用 8 字节数据区
    frame.is_extended = 0;             // 标准帧（11 位 ID）

    EncodeMotorCommand(frame.data, cmd);  // 实际填充数据区
    return frame;
}

// 从接收帧解出 MotorStatus；调用者需保证帧 ID 确实是本电机的 m_rx_id
MotorStatus Motor::DecodeStatus(const BspCanFrame& frame) {
    MotorStatus status;
    DecodeMotorStatus(frame.data, status);
    return status;
}

// ------------------------------------------------------------------
// 命令打包：按 cmd_type 分支处理不同的编码格式
// ------------------------------------------------------------------
void Motor::EncodeMotorCommand(uint8_t* data, const MotorCommand& cmd) {
    memset(data, 0, 8);

    switch (cmd.cmd_type) {
        // 特殊指令：{0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, special_byte}
        case CMD_ENABLE:
        case CMD_DISABLE:
        case CMD_SET_ZERO:
        case CMD_CLEAR_ERROR:
        case CMD_ANGLE_CORRECT: {
            data[0] = 0x80;
            data[1] = 0xFF;
            data[2] = 0xFF;
            data[3] = 0xFF;
            data[4] = 0xFF;
            data[5] = 0xFF;
            data[6] = 0xFF;
            // data[7] 根据命令类型设置
            if (cmd.cmd_type == CMD_ENABLE)        data[7] = 0xFC;
            else if (cmd.cmd_type == CMD_DISABLE)  data[7] = 0xFD;
            else if (cmd.cmd_type == CMD_SET_ZERO) data[7] = 0xFE;
            else if (cmd.cmd_type == CMD_CLEAR_ERROR) data[7] = 0xF4;
            else if (cmd.cmd_type == CMD_ANGLE_CORRECT) data[7] = 0xF7;
            break;
        }

        // 参数指令：{0x80, val[0], val[1], val[2], val[3], RW, param_type, 0xEC}
        case CMD_READ_PARAM:
        case CMD_WRITE_PARAM: {
            data[0] = 0x80;
            unsigned char* pdata = (unsigned char*)&cmd.param_value;
            data[1] = *pdata++;
            data[2] = *pdata++;
            data[3] = *pdata++;
            data[4] = *pdata++;
            data[5] = cmd.param_rw;
            data[6] = cmd.param_type;
            data[7] = 0xEC;
            break;
        }

        // 阻抗控制：5参数位域打包
        case CMD_IMPEDANCE_CTRL: {
            // 再次验证参数（防御性编程）
            float pos = MotorParamValidator::ValidatePosition(cmd.pos);
            float vel = MotorParamValidator::ValidateVelocity(cmd.vel);
            float kp = MotorParamValidator::ValidateKp(cmd.kp);
            float kd = MotorParamValidator::ValidateKd(cmd.kd);
            float torque = MotorParamValidator::ValidateTorque(cmd.torque);

            uint16_t p_int  = float_to_uint(pos,    -12.5f, 12.5f, 15);
            uint16_t v_int  = float_to_uint(vel,    -14.0f, 14.0f, 12);
            uint16_t kp_int = float_to_uint(kp,     0.0f,   500.0f, 12);
            uint16_t kd_int = float_to_uint(kd,     0.0f,   100.0f, 12);
            uint16_t t_int  = float_to_uint(torque, -200.0f, 200.0f, 12);

            data[0] = (uint8_t)((p_int >> 8) & 0x7F);
            data[1] = (uint8_t)(p_int & 0xFF);
            data[2] = (uint8_t)(v_int >> 4);
            data[3] = (uint8_t)(((v_int & 0xF) << 4) | (kp_int >> 8));
            data[4] = (uint8_t)(kp_int & 0xFF);
            data[5] = (uint8_t)(kd_int >> 4);
            data[6] = (uint8_t)(((kd_int & 0xF) << 4) | (t_int >> 8));
            data[7] = (uint8_t)(t_int & 0xFF);
            break;
        }

        // 速度控制：vel/kvp/kvi 位域打包
        case CMD_SPEED_CTRL: {
            uint32_t v_int   = float_to_uint(cmd.vel,      -14.0f, 14.0f, 31);
            uint16_t kvp_int = float_to_uint(cmd.kp_speed, 0.0f,   500.0f, 16);
            uint16_t kvi_int = float_to_uint(cmd.ki_speed, 0.0f,   10000.0f, 16);

            data[0] = (uint8_t)((v_int >> 24) & 0x7F);
            data[1] = (uint8_t)((v_int >> 16) & 0xFF);
            data[2] = (uint8_t)((v_int >> 8) & 0xFF);
            data[3] = (uint8_t)(v_int & 0xFF);
            data[4] = (uint8_t)((kvp_int >> 8) & 0xFF);
            data[5] = (uint8_t)(kvp_int & 0xFF);
            data[6] = (uint8_t)((kvi_int >> 8) & 0xFF);
            data[7] = (uint8_t)(kvi_int & 0xFF);
            break;
        }

        // 位置控制：pos/kvp/kp/kd/kvi 位域打包
        case CMD_POSITION_CTRL: {
            uint16_t p_int   = float_to_uint(cmd.pos,      -12.5f, 12.5f, 15);
            uint16_t kvp_int = float_to_uint(cmd.kp_speed, 0.0f,   500.0f, 12);
            uint16_t kp_int  = float_to_uint(cmd.kp,       0.0f,   500.0f, 12);
            uint16_t kd_int  = float_to_uint(cmd.kd,       0.0f,   100.0f, 12);
            uint16_t kvi_int = float_to_uint(cmd.ki_speed, 0.0f,   10000.0f, 12);

            data[0] = (uint8_t)((p_int >> 8) & 0x7F);
            data[1] = (uint8_t)(p_int & 0xFF);
            data[2] = (uint8_t)(kvp_int >> 4);
            data[3] = (uint8_t)(((kvp_int & 0xF) << 4) | (kp_int >> 8));
            data[4] = (uint8_t)(kp_int & 0xFF);
            data[5] = (uint8_t)(kd_int >> 4);
            data[6] = (uint8_t)(((kd_int & 0xF) << 4) | (kvi_int >> 8));
            data[7] = (uint8_t)(kvi_int & 0xFF);
            break;
        }

        default:
            break;
    }
}

// 状态帧字节序约定（硬件实际 6 字节返回帧）：
//   data[0]   : ACK(bit7) + FAULT(bit6) + *(bit5) + ENABLE(bit4) + CAN_ID(bit3-0)
//   data[1..2]: position       （16位）
//   data[3]   : velocity高4位 + current高4位
//   data[4]   : current低8位
//   data[5]   : velocity低8位
void Motor::DecodeMotorStatus(const uint8_t* data, MotorStatus& status) {
    status.motor_id = data[0] & 0x0F;
    status.ack      = (data[0] >> 7) & 1;
    status.fault    = (data[0] >> 6) & 1;
    status.enable   = (data[0] >> 4) & 1;

    uint16_t p_int = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t i_int = (((uint16_t)data[4] & 0xF) << 8) | data[5];

    status.position = uint_to_float(p_int, -12.5f, 12.5f, 16);
    status.velocity = uint_to_float(v_int, -14.0f, 14.0f, 12);
    status.torque   = uint_to_float(i_int, -200.0f, 200.0f, 12);
    status.error_code = ERR_OK;
}
