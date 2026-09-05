#include "app/examples_common.h"
#include "motor/motor_manager.h"
#include "strategy/rl_controller.h"

volatile std::sig_atomic_t g_rl_stop = 0;
void rl_signal_handler(int) { g_rl_stop = 1; }

// RL 腿摩擦前馈（status 坐标）：每 2ms 由 SendOnce 用最新反馈调用。
// 腿 fc·tanh(τ_pd)+fv·q̇ 因 CONV_A=±1（thigh=-1），在 status 坐标用
// τ_pd=kp(qdes−q)−kd·q̇、q̇ 直接套 leg_friction_ff 即得 status 坐标摩擦（thigh 双翻转抵消）。
void EnableRlFrictionFF(MotorManager& mm) {
    mm.SetLegTauFFOverride(
        [](uint8_t can_port, uint8_t motor_id,
           float pos, float vel, float qdes, float kp, float kd) -> float {
            if (motor_id > 3) return 0.0f;          // 仅腿（轮走 SPEED/独立安全段）
            int p = can_port * 3 + (motor_id - 1);  // 腿 policy 序 == CAN 腿序（FL..RR×hip/thigh/calf）
            float tau_pd = kp * (qdes - pos) - kd * vel;   // status 坐标 PD 扭矩（方向给摩擦）
            return rl::leg_friction_ff(tau_pd, vel, p);     // 内部受 LEG_FF_ENABLE 控制
        });
}

void DisableRlFrictionFF(MotorManager& mm) {
    mm.ClearLegTauFFOverride();
}

// 非阻塞读取一个方向键（方向键是 ESC [ A/B/C/D 三字节序列）。读到 'q' 返回 QUIT。
KeyDir poll_key() {
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return KeyDir::NONE;
    if (c == 'q' || c == 'Q') return KeyDir::QUIT;
    if (c != 0x1b) return KeyDir::NONE;
    unsigned char seq[2];
    if (read(STDIN_FILENO, &seq[0], 1) != 1) return KeyDir::NONE;
    if (read(STDIN_FILENO, &seq[1], 1) != 1) return KeyDir::NONE;
    if (seq[0] != '[') return KeyDir::NONE;
    switch (seq[1]) {
        case 'A': return KeyDir::UP;
        case 'B': return KeyDir::DOWN;
        case 'C': return KeyDir::RIGHT;
        case 'D': return KeyDir::LEFT;
        default:  return KeyDir::NONE;
    }
}
