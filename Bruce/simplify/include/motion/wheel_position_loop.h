/**
 * @file    wheel_position_loop.h
 * @brief   轮电机上位机位置环（纯头文件）
 *
 * ⚠ 已废弃（2026-08-29 SPEED 迁移）：轮速控制已改走固件 SPEED 速度环
 *   （SendSpeed vel/kvp/ki），不再用阻抗前馈扭矩通道。本文件为历史实现，
 *   仅 ex_basic 老示例引用，不再用于 RL 部署。
 *
 * 历史背景：曾因轮电机固件自带的速度/位置控制模式存在问题，改为只借用阻抗
 *   模式的前馈扭矩通道 —— 下发时令 kp=kd=0，电机端 PD 环失效，退化为纯扭矩
 *   执行器；真正的位置/速度环在上位机用反馈重算。
 *
 * 本结构封装单个轮子的位置环，内部维护：
 *   - 多圈累加（unwrap）：编码器位置在 ±half_span 回绕，跨界补 ±full_span，
 *     累加成连续多圈角度。回绕周期来自协议编码量程（轮子实测 ±12.5rad），
 *     不是 2π。
 *   - 目标角积分：摇杆归一化输入积分成目标角，松手时目标角保持 → 锁定当前角。
 *   - PD 位置环：τ = kp·(θ_tgt − θ_cont) − kd·v_fb，输出钳位到 ±max_torque。
 *
 * 用法：
 *   WheelPositionLoop wheel[4];
 *   for (auto& w : wheel) w.configure(4.0f, 0.2f, 3.0f, 3.0f);   // kp,kd,rate,tmax
 *   // 每周期：
 *   float tau = wheel[cp].update(fb_pos, fb_vel, stick, dt);
 *   motor_mgr.SendImpedance(cp, 4, 0,0,0,0, tau);
 */
#pragma once

#include <cmath>

struct WheelPositionLoop {
    // --- 可整定增益/上限（用 configure 设置） ---
    float kp         = 4.0f;    // 位置环刚度 (Nm/rad)
    float kd         = 0.2f;    // 位置环阻尼 (Nm/(rad/s))
    float rate       = 3.0f;    // 满摇杆时目标角推进速率 (rad/s)
    float max_torque = 3.0f;    // 输出扭矩钳位 (Nm)，安全上限

    // --- unwrap 回绕量程（来自轮子编码量程 p_min~p_max） ---
    float full_span  = 25.0f;   // 回绕周期 (p_max - p_min)
    float half_span  = 12.5f;   // 半量程，跨界判据

    // --- 内部状态 ---
    float cont_pos   = 0.0f;    // 连续多圈角度 (rad)
    float tgt_pos    = 0.0f;    // 位置环目标角 (rad)
    float prev_raw   = 0.0f;    // 上一帧原始编码器角度 (±half_span)
    bool  initialized = false;  // 是否已用首帧反馈对齐

    /// 配置增益与钳位（span 用编码量程，默认 ±12.5）
    void configure(float kp_, float kd_, float rate_, float max_torque_,
                   float full_span_ = 25.0f) {
        kp = kp_; kd = kd_; rate = rate_; max_torque = max_torque_;
        full_span = full_span_;
        half_span = full_span_ * 0.5f;
    }

    /// 用当前反馈把连续角/目标角对齐到实际位置，避免上电瞬间大扭矩。
    /// 首次 update 会自动调用；显式调用可用于重新对齐。
    void reset(float feedback_pos) {
        prev_raw    = feedback_pos;
        cont_pos    = feedback_pos;
        tgt_pos     = feedback_pos;
        initialized = true;
    }

    /**
     * 单周期更新，返回应下发的前馈扭矩 (Nm)。
     * @param feedback_pos 编码器原始位置反馈 (rad，±half_span 回绕)
     * @param feedback_vel 速度反馈 (rad/s)
     * @param stick_norm   摇杆归一化输入 [-1,1]（调用方已去死区）
     * @param dt           控制周期 (s)
     */
    float update(float feedback_pos, float feedback_vel, float stick_norm, float dt) {
        if (!initialized) {
            reset(feedback_pos);
        }

        // 反馈 unwrap：跨 ±half_span 补 ±full_span，累加成连续多圈角
        float draw = feedback_pos - prev_raw;
        if (draw >  half_span) draw -= full_span;
        if (draw < -half_span) draw += full_span;
        cont_pos += draw;
        prev_raw  = feedback_pos;

        // 摇杆积分推进目标角（松手 stick=0 → 目标角不变 → 锁定当前角度）
        tgt_pos += stick_norm * rate * dt;

        // PD 位置环 + 扭矩钳位
        float torque = kp * (tgt_pos - cont_pos) - kd * feedback_vel;
        if (torque >  max_torque) torque =  max_torque;
        if (torque < -max_torque) torque = -max_torque;
        return torque;
    }
};
