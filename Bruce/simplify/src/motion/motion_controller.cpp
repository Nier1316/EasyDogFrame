/**
 * @file    motion_controller.cpp
 * @brief   运控层实现（站立/回位 + RL 下发 + 安全兜底）
 * @details 从 Example36/37 抽取：起立插值、RL 主循环单步、优雅回位、急停。
 *          关节角约定见 rl_controller.h（策略在 URDF 约定，下发转真机指令角）。
 */
#include "motion/motion_controller.h"
#include "motion/robot_calibration.h"   // LIE_DOWN_*_DEG / deg2rad
#include "strategy/rl_controller.h"
#include "strategy/mlp.h"
#include <cstdio>
#include <cmath>
#include <unistd.h>

bool MotionController::init(MotorManager& mm, ImuDevice* imu) {
    mm_  = &mm;
    imu_ = imu;
    return true;
}

void MotionController::setConfig(const Config& cfg) { cfg_ = cfg; }
const MotionController::Config& MotionController::config() const { return cfg_; }

// 插值下发一轮：from/to 为真机指令角 [12]（腿），轮电机 0 速弱增益（SPEED 软启动）
void MotionController::sendInterpFrame(const float* from, const float* to, float t) {
    for (int leg = 0; leg < 4; leg++) {
        for (int j = 0; j < 3; j++) {
            float pos = from[leg * 3 + j] + (to[leg * 3 + j] - from[leg * 3 + j]) * t;
            mm_->SendImpedance(leg, j + 1, pos, 0.0f, cfg_.stand_kp, cfg_.stand_kd, 0.0f);
        }
        // 轮：SPEED 0 速 + 软启动弱增益（固件阻抗忽略 vel_des，轮速度控制走 SPEED 模式）
        mm_->SendSpeed(leg, 4, 0.0f, rl::WHEEL_SOFT_KVP, 0.0f);
    }
}

bool MotionController::standTo(const float target_urdf[12], float duration_s,
                               const std::function<bool()>& is_stopped) {
    // 记录起始腿位（真机指令角），供 returnToStart 回位
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            start_pos_[leg * 3 + j] = mm_->GetStatus(leg, j + 1).position;
    // 目标转真机指令角
    for (int i = 0; i < 12; i++)
        target_urdf_[i] = rl::urdf_to_status(target_urdf[i], i);

    const int total = (int)(duration_s * cfg_.hz);
    for (int f = 0; f <= total; f++) {
        if (is_stopped && is_stopped()) return false;
        sendInterpFrame(start_pos_, target_urdf_, (float)f / total);
        usleep(1000000 / cfg_.hz);
    }
    return true;
}

bool MotionController::returnToStart(float duration_s,
                                     const std::function<bool()>& is_stopped) {
    disableWheels();   // 失能轮，防回位过程被带转

    float cur_pos[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            cur_pos[leg * 3 + j] = mm_->GetStatus(leg, j + 1).position;

    const int total = (int)(duration_s * cfg_.hz);
    for (int f = 0; f <= total; f++) {
        if (is_stopped && is_stopped()) return false;
        sendInterpFrame(cur_pos, start_pos_, (float)f / total);
        usleep(1000000 / cfg_.hz);
    }
    return true;
}

bool MotionController::lieDown(float duration_s,
                               const std::function<bool()>& is_stopped) {
    // 记录当前腿位（真机指令角），作为插值起点
    float cur_pos[12];
    for (int leg = 0; leg < 4; leg++)
        for (int j = 0; j < 3; j++)
            cur_pos[leg * 3 + j] = mm_->GetStatus(leg, j + 1).position;

    // 趴下目标（真机角，LIE_DOWN_* 见 robot_calibration.h）
    float down_pos[12];
    for (int leg = 0; leg < 4; leg++) {
        down_pos[leg * 3 + 0] = deg2rad(LIE_DOWN_HIP_DEG);
        down_pos[leg * 3 + 1] = deg2rad(LIE_DOWN_THIGH_DEG);
        down_pos[leg * 3 + 2] = deg2rad(LIE_DOWN_CALF_DEG);
    }

    // 慢速插值：身体缓降，全程 PD 保持 + 轮子 0 速弱增益（sendInterpFrame 已处理）
    const int total = (int)(duration_s * cfg_.hz);
    for (int f = 0; f <= total; f++) {
        if (is_stopped && is_stopped()) return false;
        sendInterpFrame(cur_pos, down_pos, (float)f / total);
        usleep(1000000 / cfg_.hz);
    }
    return true;
}

void MotionController::beginRL(const float init_cmd[3]) {
    for (int i = 0; i < 16; i++) last_action_[i] = 0.0f;
    for (int i = 0; i < 4; i++) wheel_v_lp_[i] = 0.0f;   // 轮速低通从 0 起步（起立期间轮 0 速）
    cmd_[0] = init_cmd[0]; cmd_[1] = init_cmd[1]; cmd_[2] = init_cmd[2];
    step_      = 0;
    rl_active_ = true;
}

void MotionController::setCmd(const float cmd[3]) {
    cmd_[0] = cmd[0]; cmd_[1] = cmd[1]; cmd_[2] = cmd[2];
}

bool MotionController::rlStep() {
    if (!rl_active_) return true;

    // 1) 读 16 电机（CAN 顺序）
    float pos_can[16], vel_can[16];
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) {
            MotorStatus st = mm_->GetStatus(cp, mi);
            int mjx = cp * 4 + (mi - 1);
            pos_can[mjx] = st.position;
            vel_can[mjx] = st.velocity;
        }

    // 2) CAN -> policy -> URDF
    float pos_policy[16], vel_policy[16];
    for (int i = 0; i < 16; i++) {
        pos_policy[i] = rl::status_to_urdf(pos_can[rl::MJX_TO_POLICY[i]], i);
        vel_policy[i] = rl::status_vel_to_urdf(vel_can[rl::MJX_TO_POLICY[i]], i);
    }

    // 3) IMU
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (imu_) {
        imu_->GetGyro(gyro[0], gyro[1], gyro[2]);
        imu_->GetQuat(quat[0], quat[1], quat[2], quat[3]);
    }

    // 4) 观测 -> 推理 -> 下发
    rl::build_observation(gyro, quat, pos_policy, vel_policy,
                          last_action_, cmd_, step_, last_obs_);
    float action[16];
    rl::mlp_forward(last_obs_, action);

    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++) {
            int mjx = cp * 4 + (mi - 1);
            int p = rl::POLICY_TO_MJX[mjx];
            if (mi <= 3) {
                float q_target = rl::urdf_to_status(rl::leg_pos_target(action[p], p), p);
                const JointImpedanceParam& ip = GetJointImpedance(cp, mi);
                mm_->SendImpedance(cp, mi, q_target, 0.0f, rl::LEG_KP, rl::LEG_KD, ip.tau_ff);
            } else {
                int w_idx = p - rl::NUM_LEG_JOINTS;
                // 轮子：SPEED 固件速度环（2026-08-29 迁移）。固件阻抗忽略 vel_des 实测轮不动，
                // 速度控制改走 SendSpeed(vel/kvp/ki)，固件内部 1kHz 闭环（无上位机延迟振荡）。
                // 目标速度一阶低通：抑制推杆/松杆 cmd 骤变导致的轮子振荡（τ≈100ms @50Hz）。
                float tv_raw = rl::WHEEL_VEL_SCALE * action[p];
                wheel_v_lp_[w_idx] += rl::WHEEL_CMD_ALPHA * (tv_raw - wheel_v_lp_[w_idx]);
                float cmd_v = wheel_v_lp_[w_idx];
                // 站立门控：无移动指令（cmd vx/wz 均低于阈值）时轮子强制静止（防溜车）。
                // ⚠ 移动时不再削死区：死区 0.5 会削减转向差速 action（target 0.5~1.0），
                //   导致真机转向严重不到位（035837 双开对比实证）。门控已覆盖站立锁轮，
                //   移动时策略轮 action 是真实意图，完整执行。
                if (fabsf(cmd_[0]) < rl::WHEEL_CMD_MOVE_THR && fabsf(cmd_[2]) < rl::WHEEL_CMD_MOVE_THR)
                    cmd_v = 0.0f;
                mm_->SendSpeed(cp, mi, cmd_v, rl::WHEEL_KVP, rl::WHEEL_KVI);
                tau_wheel_[w_idx] = rl::WHEEL_KD * (cmd_v - vel_policy[p]);  // 诊断：预估速度环扭矩
            }
        }

    // 5) last_action
    for (int i = 0; i < 16; i++) last_action_[i] = action[i];

    // 6) 跌倒检测：proj_grav_z 正常 ≈ -1（机体水平，重力朝下）；抬升到 > 阈值
    //    （机体严重倾斜/倾倒）判跌倒。正常返回 true 继续，跌倒返回 false。
    bool ok = last_obs_[8] <= cfg_.fall_pgr_z_thr;
    step_++;
    return ok;
}

void MotionController::emergencyStop() {
    if (!mm_) return;
    for (int cp = 0; cp < 4; cp++)
        for (int mi = 1; mi <= 4; mi++)
            mm_->DisableMotor(cp, mi);
}

void MotionController::disableWheels() {
    if (!mm_) return;
    for (int cp = 0; cp < 4; cp++)
        mm_->DisableMotor(cp, 4);
}

const float* MotionController::lastAction() const { return last_action_; }
const float* MotionController::lastObs() const { return last_obs_; }
const float* MotionController::lastTauWheel() const { return tau_wheel_; }
float MotionController::lastGravZ() const { return last_obs_[8]; }
int   MotionController::step() const { return step_; }
