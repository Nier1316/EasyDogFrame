#ifndef MOTOR_DECODE_H
#define MOTOR_DECODE_H
// CAN 帧 → 人类可读含义。三种控制模式各给一种解读；
// 特殊指令帧 / float2bag 参数帧单独识别。逻辑复刻官方位布局的逆过程。
#include "motor_codec.h"
#include <cstdio>
#include <string>

namespace motorcodec {

// 特殊指令帧命令字（末字节）——与官方 motor_rw_api.h / ele_motor_def.h 一致
inline const char* special_name(uint8_t cmd) {
    switch (cmd) {
        case 0xFC: return "MOTOR_STRAT 电机启动/使能";
        case 0xFD: return "MOTOR_STOP 电机停止/失能";
        case 0xFE: return "MOTOR_ANGLE_ZERO 角度归零";
        case 0xF9: return "MOTOR_CHANGE_ID 修改ID";
        case 0xF7: return "MOTOR_ANGLE_CORRECTION 角度校正";
        case 0xF6: return "MOTOR_ABSOLUTE_POSITION_CORRECT 绝对位置校正";
        case 0xF4: return "MOTOR_CLEAR_ERROR 清除错误";
        default:   return nullptr;
    }
}

inline std::string hex2(uint8_t b) {
    char s[8]; std::snprintf(s, sizeof(s), "0x%02X", b); return s;
}

inline std::string line(const char* label, float val, const char* unit) {
    char s[128];
    std::snprintf(s, sizeof(s), "    %-14s = %.4f %s\n", label, val, unit);
    return s;
}

// 按指定模式把 8 字节解读为参数。d 指向至少 8 字节。
inline std::string decode_as(Mode model, const uint8_t* d) {
    std::string r;
    if (model == IMPEDANCE) {
        uint32_t p  = ((uint32_t)(d[0] & 0x7f) << 8) | d[1];
        uint32_t v  = ((uint32_t)d[2] << 4) | (d[3] >> 4);
        uint32_t kp = ((uint32_t)(d[3] & 0xF) << 8) | d[4];
        uint32_t kd = ((uint32_t)d[5] << 4) | (d[6] >> 4);
        uint32_t tq = ((uint32_t)(d[6] & 0xF) << 8) | d[7];
        r += "  【阻抗模式 IMPEDANCE】\n";
        r += line("期望角度 pos", uint_to_float(p, P_MIN, P_MAX, 15), "rad");
        r += line("期望角速度 vel", uint_to_float(v, V_MIN, V_MAX, 12), "rad/s");
        r += line("刚度 kp", uint_to_float(kp, KP_MIN, KP_MAX, 12), "");
        r += line("阻尼 kd", uint_to_float(kd, KD_MIN, KD_MAX, 12), "");
        r += line("前馈扭矩 tau", uint_to_float(tq, T_MIN, T_MAX, 12), "Nm");
    } else if (model == SPEED) {
        uint32_t v   = ((uint32_t)(d[0] & 0x7f) << 24) | ((uint32_t)d[1] << 16)
                     | ((uint32_t)d[2] << 8) | d[3];
        uint32_t kvp = ((uint32_t)d[4] << 8) | d[5];
        uint32_t kvi = ((uint32_t)d[6] << 8) | d[7];
        r += "  【速度模式 SPEED】\n";
        r += line("期望角速度 vel", uint_to_float(v, V_MIN, V_MAX, 31), "rad/s");
        r += line("速度环 kvp", uint_to_float(kvp, KP_MIN, KP_MAX, 16), "");
        r += line("速度环 kvi", uint_to_float(kvi, KI_MIN, KI_MAX, 16), "");
    } else { // POSITION
        uint32_t p   = ((uint32_t)(d[0] & 0x7f) << 8) | d[1];
        uint32_t kvp = ((uint32_t)d[2] << 4) | (d[3] >> 4);
        uint32_t kp  = ((uint32_t)(d[3] & 0xF) << 8) | d[4];
        uint32_t kd  = ((uint32_t)d[5] << 4) | (d[6] >> 4);
        uint32_t kvi = ((uint32_t)(d[6] & 0xF) << 8) | d[7];
        r += "  【位置模式 POSITION】\n";
        r += line("期望角度 pos", uint_to_float(p, P_MIN, P_MAX, 15), "rad");
        r += line("位置环 kvp", uint_to_float(kvp, KP_MIN, KP_MAX, 12), "");
        r += line("速度环 kp", uint_to_float(kp, KP_MIN, KP_MAX, 12), "");
        r += line("位置环 kd", uint_to_float(kd, KD_MIN, KD_MAX, 12), "");
        r += line("速度环 kvi", uint_to_float(kvi, KI_MIN, KI_MAX, 12), "");
    }
    return r;
}

// 顶层解码：先判特殊帧，否则按给定模式解读（mode<0 时给出全部三种解读）。
inline std::string decode(const uint8_t* d, int mode /* -1=全部 */) {
    std::string r;
    // 特殊指令帧：0x80 开头 + 末字节为命令字，中间多为 0xFF
    const char* sp = special_name(d[7]);
    if (d[0] == 0x80 && sp) {
        r += "  识别为【特殊指令帧】\n";
        r += std::string("    命令: ") + sp + "\n";
        r += "    (data0=0x80, data7=" + hex2(d[7]) + ")\n\n";
    }
    // float2bag 参数读写帧：末字节 0xEC
    if (d[7] == 0xEC) {
        unionFloatLE u; u.b[0]=d[1]; u.b[1]=d[2]; u.b[2]=d[3]; u.b[3]=d[4];
        r += "  疑似【float2bag 参数帧】(data7=0xEC)\n";
        char s[96];
        std::snprintf(s, sizeof(s), "    RW=%s  type=0x%02X  value(小端float)=%.4f\n\n",
                      (d[5] ? "写" : "读"), d[6], u.f);
        r += s;
    }
    if (d[0] & 0x80) {
        r += "  ⚠ data0 的 bit7=1：正常控制帧该位恒为 0，"
             "此帧更可能是特殊/参数帧，或字节对齐有误。\n\n";
    }
    if (mode < 0) {
        r += decode_as(IMPEDANCE, d) + "\n";
        r += decode_as(SPEED, d) + "\n";
        r += decode_as(POSITION, d);
    } else {
        r += decode_as((Mode)mode, d);
    }
    return r;
}

} // namespace motorcodec
#endif // MOTOR_DECODE_H
