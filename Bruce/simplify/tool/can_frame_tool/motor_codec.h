#ifndef MOTOR_CODEC_H
#define MOTOR_CODEC_H
// 电机 CAN 帧编解码 —— 位布局与量程严格复刻官方 tool/motor_rw_api.c
// 纯逻辑、无任何第三方/工程依赖，可单独编译测试。
#include <cstdint>
#include <array>
#include <string>

namespace motorcodec {

// 电机类型（对齐运行层 ele_motor_def.h 的 MotorType，HIP/THIGH/CALF/WHEEL）
enum MotorType { HIP = 0, THIGH = 1, CALF = 2, WHEEL = 3 };

// 位置/刚度/阻尼量程各类型相同
constexpr float P_MIN = -12.5f,  P_MAX = 12.5f;    // 位置 rad
constexpr float KP_MIN = 0.0f,   KP_MAX = 500.0f;  // 刚度
constexpr float KD_MIN = 0.0f,   KD_MAX = 100.0f;  // 阻尼

// 速度/扭矩/积分量程【按类型】——真值源 = 运行层 ele_motor_def.h MOTOR_LIMITS。
// ⚠ 2026-08-07 固件实测修正：关节 v 量程 ±3（官方默认 ±14 差 4.7 倍！）；
//   2026-09-04 固件扭矩限幅：Hip/Thigh ±120、Calf ±200、Wheel ±52；
//   ki_max 固件无寄存器可读，暂 500（与运行层一致）。
//   若运行层 MOTOR_LIMITS 再改，此处须同步。
struct TypeRange { float v_lo, v_hi; float t_lo, t_hi; float ki_hi; };
inline constexpr TypeRange TYPE_RANGE[WHEEL + 1] = {
    /* HIP   */ {-3.0f,   3.0f, -120.0f, 120.0f, 500.0f},
    /* THIGH */ {-3.0f,   3.0f, -120.0f, 120.0f, 500.0f},
    /* CALF  */ {-3.0f,   3.0f, -200.0f, 200.0f, 500.0f},
    /* WHEEL */ {-48.0f, 48.0f,  -52.0f,  52.0f, 500.0f},
};

enum Mode { IMPEDANCE = 0, SPEED = 1, POSITION = 2 };

// 小端 4 字节 ↔ float（float2bag 参数帧用）
union unionFloatLE { uint8_t b[4]; float f; };

// 官方换算：float→uint（此处加范围钳位，越界会置真并提示）
inline uint32_t float_to_uint(float x, float xmin, float xmax, int bits, bool* clamped = nullptr) {
    if (clamped) *clamped = (x < xmin || x > xmax);
    if (x < xmin) x = xmin;
    if (x > xmax) x = xmax;
    float span = xmax - xmin;
    return (uint32_t)((x - xmin) * (float)((1u << bits) - 1u) / span);
}

// 官方换算：uint→float
inline float uint_to_float(uint32_t xi, float xmin, float xmax, int bits) {
    float span = xmax - xmin;
    return (float)xi * span / (float)((1u << bits) - 1u) + xmin;
}

using Frame = std::array<uint8_t, 8>;

// 参数 → 8 字节 CAN 帧。p1~p5 含义随模式而变（见官方注释）。
// type 指定电机类型（Hip/Thigh/Calf/Wheel），决定速度/扭矩/积分量程。
// clamped 若非空，返回是否有参数越界被钳位。
inline Frame encode(Mode model, float p1, float p2, float p3, float p4, float p5,
                    MotorType type = HIP, bool* clamped = nullptr) {
    Frame t{};
    const TypeRange& tr = TYPE_RANGE[type];
    bool any = false; bool c;
    auto f2u = [&](float x, float lo, float hi, int b) {
        uint32_t v = float_to_uint(x, lo, hi, b, &c); any |= c; return v;
    };
    if (model == IMPEDANCE) {
        uint32_t p  = f2u(p1, P_MIN, P_MAX, 15);         // 期望角度
        uint32_t v  = f2u(p2, tr.v_lo, tr.v_hi, 12);     // 期望角速度
        uint32_t kp = f2u(p3, KP_MIN, KP_MAX, 12);       // 刚度
        uint32_t kd = f2u(p4, KD_MIN, KD_MAX, 12);       // 阻尼
        uint32_t tq = f2u(p5, tr.t_lo, tr.t_hi, 12);     // 前馈扭矩
        t[0] = (uint8_t)(p >> 8 & 0x7f);
        t[1] = (uint8_t)(p & 0xFF);
        t[2] = (uint8_t)(v >> 4);
        t[3] = (uint8_t)(((v & 0xF) << 4) | (kp >> 8));
        t[4] = (uint8_t)(kp & 0xFF);
        t[5] = (uint8_t)(kd >> 4);
        t[6] = (uint8_t)(((kd & 0xF) << 4) | (tq >> 8));
        t[7] = (uint8_t)(tq & 0xff);
    } else if (model == SPEED) {
        uint32_t v   = f2u(p1, tr.v_lo, tr.v_hi, 31);    // 期望角速度
        uint32_t kvp = f2u(p2, KP_MIN, KP_MAX, 16);      // 速度环Kp
        uint32_t kvi = f2u(p5, 0.0f, tr.ki_hi, 16);      // 速度环Ki
        t[0] = (uint8_t)(v >> 24 & 0x7f);
        t[1] = (uint8_t)(v >> 16 & 0xFF);
        t[2] = (uint8_t)(v >> 8 & 0xFF);
        t[3] = (uint8_t)(v & 0xFF);
        t[4] = (uint8_t)(kvp >> 8 & 0xFF);
        t[5] = (uint8_t)(kvp & 0xff);
        t[6] = (uint8_t)(kvi >> 8 & 0xFF);
        t[7] = (uint8_t)(kvi & 0xff);
    } else { // POSITION
        uint32_t p   = f2u(p1, P_MIN, P_MAX, 15);        // 期望角度
        uint32_t kvp = f2u(p2, KP_MIN, KP_MAX, 12);      // 位置环Kp
        uint32_t kp  = f2u(p3, KP_MIN, KP_MAX, 12);      // 速度环Kp
        uint32_t kd  = f2u(p4, KD_MIN, KD_MAX, 12);      // 位置环Kd
        uint32_t kvi = f2u(p5, 0.0f, tr.ki_hi, 12);      // 速度环Ki
        t[0] = (uint8_t)(p >> 8 & 0x7f);
        t[1] = (uint8_t)(p & 0xFF);
        t[2] = (uint8_t)(kvp >> 4);
        t[3] = (uint8_t)(((kvp & 0xF) << 4) | (kp >> 8));
        t[4] = (uint8_t)(kp & 0xFF);
        t[5] = (uint8_t)(kd >> 4);
        t[6] = (uint8_t)(((kd & 0xF) << 4) | (kvi >> 8));
        t[7] = (uint8_t)(kvi & 0xff);
    }
    if (clamped) *clamped = any;
    return t;
}

} // namespace motorcodec
#endif // MOTOR_CODEC_H
