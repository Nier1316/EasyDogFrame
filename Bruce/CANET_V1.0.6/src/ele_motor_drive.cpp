/**
 * @file    ele_motor_drive.cpp
 * @brief   电机驱动扩展模块实现 - 参数读写、验证、编解码等高级功能
 */
#include "ele_motor_drive.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

// =====================================================================
//                    MotorParamValidator 实现
// =====================================================================

float MotorParamValidator::Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float MotorParamValidator::ValidatePosition(float pos) {
    return Clamp(pos, P_MIN, P_MAX);
}

float MotorParamValidator::ValidateVelocity(float vel) {
    return Clamp(vel, V_MIN, V_MAX);
}

float MotorParamValidator::ValidateTorque(float torque) {
    return Clamp(torque, T_MIN, T_MAX);
}

float MotorParamValidator::ValidateKp(float kp) {
    return Clamp(kp, KP_MIN, KP_MAX);
}

float MotorParamValidator::ValidateKd(float kd) {
    return Clamp(kd, KD_MIN, KD_MAX);
}

float MotorParamValidator::ValidateKi(float ki) {
    return Clamp(ki, KI_MIN, KI_MAX);
}

bool MotorParamValidator::IsInRange(float value, float min, float max) {
    return value >= min && value <= max;
}

bool MotorParamValidator::IsPositionValid(float pos) {
    return IsInRange(pos, P_MIN, P_MAX);
}

bool MotorParamValidator::IsVelocityValid(float vel) {
    return IsInRange(vel, V_MIN, V_MAX);
}

bool MotorParamValidator::IsTorqueValid(float torque) {
    return IsInRange(torque, T_MIN, T_MAX);
}

bool MotorParamValidator::IsKpValid(float kp) {
    return IsInRange(kp, KP_MIN, KP_MAX);
}

bool MotorParamValidator::IsKdValid(float kd) {
    return IsInRange(kd, KD_MIN, KD_MAX);
}

bool MotorParamValidator::IsKiValid(float ki) {
    return IsInRange(ki, KI_MIN, KI_MAX);
}

// =====================================================================
//                    MotorParamCodec 实现
// =====================================================================

void MotorParamCodec::Float2Bag(float parameter, uint8_t RW, uint8_t type, uint8_t id, uint8_t* data) {
    if (!data) return;

    data[0] = 0x80;

    // 将float转换为4字节
    unsigned char* pdata = (unsigned char*)&parameter;
    data[1] = *pdata++;
    data[2] = *pdata++;
    data[3] = *pdata++;
    data[4] = *pdata++;
    data[5] = RW;           // 读写标志
    data[6] = type;         // 参数类型
    data[7] = 0xEC;         // 参数指令标识

    printf("[INFO] Float2Bag: parameter=%.2f, RW=%d, type=0x%02x, id=%d\n", parameter, RW, type, id);
}

float MotorParamCodec::U8Array2Float(const uint8_t* data, uint8_t key) {
    if (!data) return 0.0f;

    float result = 0.0f;
    unsigned char* presult = (unsigned char*)&result;

    // 根据key选择不同的解析方式
    switch (key) {
        case 0:  // 标准float解析（4字节）
            presult[0] = data[0];
            presult[1] = data[1];
            presult[2] = data[2];
            presult[3] = data[3];
            break;
        case 1:  // 从data[1]开始解析
            presult[0] = data[1];
            presult[1] = data[2];
            presult[2] = data[3];
            presult[3] = data[4];
            break;
        default:
            presult[0] = data[0];
            presult[1] = data[1];
            presult[2] = data[2];
            presult[3] = data[3];
            break;
    }

    return result;
}

unsigned int MotorParamCodec::FloatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;

    // 自动截断超出范围的值
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;

    return (unsigned int)((x - offset) * (float)((1U << bits) - 1) / span);
}

float MotorParamCodec::UintToFloat(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

float MotorParamCodec::UnpackCmd(const uint8_t* data) {
    if (!data) return 0.0f;

    // 检查命令格式
    if (data[0] != 0x80) {
        return 0.0f;  // 无效的命令格式
    }

    // 根据data[7]判断命令类型
    uint8_t cmd_type = data[7];

    switch (cmd_type) {
        case 0xEC:  // 参数指令
            // 从data[1-4]解析float参数
            return U8Array2Float(data, 1);

        case 0xFC:  // 使能指令
        case 0xFD:  // 失能指令
        case 0xFE:  // 置零指令
        case 0xF4:  // 清错误指令
        case 0xF7:  // 角度矫正指令
            return 0.0f;  // 特殊指令无参数

        default:
            return 0.0f;
    }
}

void MotorParamCodec::SetMotorParaBt(float p1, float p2, float p3, float p4, float p5, int model, int id) {
    printf("[INFO] SetMotorParaBt: model=%d, id=%d\n", model, id);
    printf("       p1=%.2f, p2=%.2f, p3=%.2f, p4=%.2f, p5=%.2f\n", p1, p2, p3, p4, p5);
}

// =====================================================================
//                    MotorParameterManager 实现
// =====================================================================

MotorParameterManager::MotorParameterManager() {
    // 初始化默认参数
    m_params = MotorParameter();
}

MotorParameterManager::~MotorParameterManager() {
    // 清理资源
}

float MotorParameterManager::ReadParameter(uint8_t param_type) {
    switch (param_type) {
        case MOTOR_WR_LD:
            return m_params.ld;
        case MOTOR_WR_LQ:
            return m_params.lq;
        case MOTOR_WR_FLUX:
            return m_params.flux;
        case MOTOR_WR_RESISTANCE:
            return m_params.resistance;
        case MOTOR_WR_GR:
            return m_params.gr;
        case MOTOR_WR_J:
            return m_params.j;
        case MOTOR_WR_B:
            return m_params.b;
        case MOTOR_WR_P:
            return (float)m_params.p;
        case MOTOR_WR_Tf:
            return m_params.tf;
        case MOTOR_WR_KT_OUT:
            return m_params.kt_out;
        case MOTOR_WR_CAN_ID:
            return (float)m_params.can_id;
        case MOTOR_WR_Current_Limit:
            return m_params.current_limit;
        case MOTOR_WR_temp_Protection:
            return m_params.temp_protection;
        case MOTOR_WR_CONTROL_MODE:
            return (float)m_params.control_mode;
        default:
            printf("[WARNING] Unknown parameter type: 0x%02x\n", param_type);
            return 0.0f;
    }
}

bool MotorParameterManager::WriteParameter(uint8_t param_type, float value) {
    switch (param_type) {
        case MOTOR_WR_LD:
            m_params.ld = value;
            break;
        case MOTOR_WR_LQ:
            m_params.lq = value;
            break;
        case MOTOR_WR_FLUX:
            m_params.flux = value;
            break;
        case MOTOR_WR_RESISTANCE:
            m_params.resistance = value;
            break;
        case MOTOR_WR_GR:
            m_params.gr = MotorParamValidator::Clamp(value, 1.0f, 32.0f);
            break;
        case MOTOR_WR_J:
            m_params.j = value;
            break;
        case MOTOR_WR_B:
            m_params.b = value;
            break;
        case MOTOR_WR_P:
            m_params.p = (uint8_t)value;
            break;
        case MOTOR_WR_Tf:
            m_params.tf = value;
            break;
        case MOTOR_WR_KT_OUT:
            m_params.kt_out = MotorParamValidator::Clamp(value, 0.1f, 10.0f);
            break;
        case MOTOR_WR_CAN_ID:
            m_params.can_id = (uint8_t)MotorParamValidator::Clamp(value, 1.0f, 15.0f);
            break;
        case MOTOR_WR_Current_Limit:
            m_params.current_limit = MotorParamValidator::Clamp(value, 0.0f, 60.0f);
            break;
        case MOTOR_WR_temp_Protection:
            m_params.temp_protection = value;
            break;
        case MOTOR_WR_CONTROL_MODE:
            m_params.control_mode = (uint8_t)value;
            break;
        default:
            printf("[WARNING] Unknown parameter type: 0x%02x\n", param_type);
            return false;
    }

    printf("[INFO] Parameter 0x%02x written: %.2f\n", param_type, value);
    return true;
}
