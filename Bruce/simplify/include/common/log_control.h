#ifndef LOG_CONTROL_H
#define LOG_CONTROL_H

/**
 * @file    log_control.h
 * @brief   控制台日志分类开关（集中管理）
 *
 * 目的：散落在各示例里的 printf 诊断太多、太乱，需要一处集中管理。
 * 用法：
 *   1) 只改下面 LOG_SWITCH 数组（true=开 / false=关），重新编译即可开关某类日志。
 *   2) 需要按类控制的日志用 LOG(LogCat::XXX, fmt, ...) 代替 printf(...)。
 *   3) MotorLogger（log/*.csv）负责数据记录（帧级 CSV），与本模块互补。
 *
 * 约定：分类 + 开关 = 只控制"诊断类"输出；关键流程提示（初始化失败等）请保持 printf。
 */
#include <cstdio>
#include <cstdint>

namespace logctl {

// 日志分类（顺序必须与 LOG_SWITCH 一一对应）
enum class LogCat : uint8_t {
    SYSTEM = 0,   // 初始化 / 起立 / 失能等流程
    MOTOR,        // 电机状态 / 参数
    RL,           // RL 循环：action / 下发扭矩 / 观测关键量
    IMU,          // IMU 数据 / 姿态 / projected_gravity
    WHEEL,        // 轮子反馈 / 扭矩 / 转向诊断
    CAN,          // CAN 通信 / 帧诊断
    DIAG,         // 详细诊断（默认关，定位问题时开）
    COUNT,
};

// =====================================================================
//  集中开关：true = 开，false = 关。改下面数组元素即可（重新编译生效）。
//  顺序与 LogCat 枚举一致，注释标了分类名。
// =====================================================================
static constexpr bool LOG_SWITCH[static_cast<int>(LogCat::COUNT)] = {
    false,   // SYSTEM
    false,   // MOTOR
    true,    // RL
    true,    // IMU
    true,    // WHEEL
    false,   // CAN
    false,   // DIAG
};

inline bool log_enabled(LogCat c) {
    int i = static_cast<int>(c);
    return i >= 0 && i < static_cast<int>(LogCat::COUNT) && LOG_SWITCH[i];
}

} // namespace logctl

// 日志宏：LOG(LogCat::RL, "fmt %d\n", ...)；未开启的分类不输出（编译期可优化掉）
#define LOG(cat, ...) \
    do { if (::logctl::log_enabled(cat)) printf(__VA_ARGS__); } while (0)

#endif // LOG_CONTROL_H
