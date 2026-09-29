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

// 非阻塞读取一个方向键（方向键是 ESC [ A/B/C/D 三字节序列）。读到 'q'/'Q' 返回 QUIT。
//
// ⚠ 2026-09-29 修丢键：原实现"读到 ESC 就立刻再 read 两次"，但 stdin 是非阻塞的，
//   三字节序列可能分多次到达（首次 read 只拿到 ESC），后两次 read 返回 -1 →
//   整个方向键被静默丢弃。现改为带待解析缓冲：把当前可读字节全部吸进缓冲，
//   凑齐一个完整记号才返回，不足则保留等下次调用。
KeyDir poll_key() {
    static unsigned char buf[16];
    static int len = 0;

    // 1) 把当前可读的字节尽量吸进缓冲（非阻塞，读空即止）
    unsigned char tmp[16];
    ssize_t n;
    while (len < (int)sizeof(buf) &&
           (n = read(STDIN_FILENO, tmp, sizeof(tmp))) > 0) {
        for (ssize_t i = 0; i < n && len < (int)sizeof(buf); ++i)
            buf[len++] = tmp[i];
    }
    if (len == 0) return KeyDir::NONE;

    // 2) 从缓冲头部解析一个完整记号（多余字节保留给下次调用）
    auto drop = [](int k) {
        for (int i = k; i < len; ++i) buf[i - k] = buf[i];
        len -= k;
    };

    if (buf[0] == 'q' || buf[0] == 'Q') { drop(1); return KeyDir::QUIT; }
    if (buf[0] != 0x1b)                 { drop(1); return KeyDir::NONE; }
    if (len < 3) return KeyDir::NONE;                 // ESC 序列尚未到齐，保留
    if (buf[1] != '[')                  { drop(1); return KeyDir::NONE; }

    KeyDir dir = KeyDir::NONE;
    switch (buf[2]) {
        case 'A': dir = KeyDir::UP;    break;
        case 'B': dir = KeyDir::DOWN;  break;
        case 'C': dir = KeyDir::RIGHT; break;
        case 'D': dir = KeyDir::LEFT;  break;
        default:  break;                              // 其它扩展序列，丢弃这 3 字节
    }
    drop(3);
    return dir;
}
