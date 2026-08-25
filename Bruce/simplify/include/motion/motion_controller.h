/**
 * @file    motion_controller.h
 * @brief   运控层：站立/回位插值 + RL 下发循环 + 安全兜底（L4）
 * @details 从 Example36/37 抽取的公共运控能力：
 *          - standTo / returnToStart：读初始位 → 插值 → 下发（起立/回位）
 *          - RL 循环（beginRL + setCmd + rlStep）：读状态→obs→策略推理→目标下发→跌倒检测
 *          - 急停/失能轮
 *          设备初始化、手柄/键盘输入、日志仍留在应用层（示例），本类只做运动控制。
 *          关节角约定：URDF 约定（策略坐标系），下发前经 rl::urdf_to_status 转真机指令角。
 */
#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_

#include <cstdint>
#include <functional>
#include "motor/motor_manager.h"
#include "strategy/imu_device.h"

class MotionController {
public:
    struct Config {
        int   hz             = 50;    // 控制频率（RL 循环）
        float stand_kp       = 200.0f;// 起立/回位插值 PD 刚度
        float stand_kd       = 20.0f; // 起立/回位插值 PD 阻尼
        float fall_pgr_z_thr = -0.34f;// 跌倒检测：projected_gravity.z 高于此值判跌倒
        float max_vx         = 1.0f;  // 命令 vx 量程（手柄映射用）
        float max_wz         = 1.0f;  // 命令 wz 量程
        float cmd_bias_vx    = 0.0f;  // 向前溜车抵消偏置（叠加在 cmd[0]）
    };

    /**
     * 注入依赖（需在调用起立/RL 前调用）
     * @param mm  MotorManager（传输/电机已初始化）
     * @param imu IMU（可为 nullptr，则 gyro/quat 用默认值，机器人可能失控）
     */
    bool init(MotorManager& mm, ImuDevice* imu = nullptr);
    void setConfig(const Config& cfg);
    const Config& config() const;

    // ============ 插值控制（起立 / 回位，阻塞式） ============

    /**
     * 从当前腿位插值起立到目标姿态（URDF 约定 12 腿关节角）。
     * 记录起始位置供 returnToStart 回位。轮电机全程 0 扭矩。
     * @param target_urdf [12] URDF 约定目标（如 rl::DEFAULT_POSE）
     * @param duration_s  起立时长（秒）
     * @param is_stopped  每周期查询的中止条件（返回 true 中断；可为 nullptr）
     * @return true 正常完成；false 被中止
     */
    bool standTo(const float target_urdf[12], float duration_s,
                 const std::function<bool()>& is_stopped = nullptr);

    /**
     * 失能轮电机，并把腿从当前位置插值回 standTo 记录的起始位。
     * @return true 正常完成；false 被中止
     */
    bool returnToStart(float duration_s,
                       const std::function<bool()>& is_stopped = nullptr);

    // ============ RL 循环（50Hz 主循环单步，非阻塞） ============

    /** 初始化 RL 状态（last_action 清零、step 归零、记录 cmd 初值） */
    void beginRL(const float init_cmd[3]);

    /** 每周期更新速度命令 [vx, vy, wz]（例：手柄映射后调用） */
    void setCmd(const float cmd[3]);

    /**
     * 执行一步 RL：读 16 电机 → CAN→URDF → IMU → obs → mlp → 目标下发。
     * 内含跌倒检测。返回 false 表示触发跌倒急停，调用方应退出循环。
     */
    bool rlStep();

    // ============ 安全 ============

    /** 失能所有 16 电机（急停） */
    void emergencyStop();

    /** 只失能 4 个轮电机（优雅回位前） */
    void disableWheels();

    // ============ 状态读取（供示例日志） ============

    const float* lastAction() const;   // [16] 最近一步策略动作
    const float* lastObs() const;      // [64] 最近一步观测（供日志/诊断）
    const float* lastTauWheel() const; // [4]  最近一步轮扭矩（FL,FR,RL,RR）
    float lastGravZ() const;           // 最近一步 projected_gravity.z（跌倒检测判据）
    int   step() const;

private:
    // 插值下发一轮（16 电机，腿 PD + 轮 0 扭矩）
    void sendInterpFrame(const float* from, const float* to, float t);

    MotorManager* mm_ = nullptr;
    ImuDevice*    imu_ = nullptr;
    Config  cfg_{};
    float   start_pos_[12]  = {0.0f};  // standTo 记录的起始腿位（真机指令角）
    float   target_urdf_[12] = {0.0f}; // 当前目标（URDF）
    float   last_action_[16] = {0.0f};
    float   cmd_[3]          = {0.0f, 0.0f, 0.0f};
    float   last_obs_[64]    = {0.0f};
    float   tau_wheel_[4]    = {0.0f};
    int     step_            = 0;
    bool    rl_active_       = false;
};

#endif // MOTION_CONTROLLER_H_
