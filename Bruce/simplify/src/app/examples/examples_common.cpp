#include "app/examples_common.h"

volatile std::sig_atomic_t g_rl_stop = 0;
void rl_signal_handler(int) { g_rl_stop = 1; }

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
