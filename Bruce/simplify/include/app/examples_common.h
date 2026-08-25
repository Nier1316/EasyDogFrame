#pragma once
// 示例公共 helper：终端 raw 模式 / 方向键解析 / 全局急停标志（阶段3 从 example.cpp 提取）
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>

// 终端 raw 模式管理：进入时关闭行缓冲/回显并设非阻塞，析构自动恢复。
// 请从集成终端（Run Program 任务）运行，stdin 才是真实终端。
struct RawTerminal {
    termios old_tio{};
    int     old_flags = 0;
    bool    ok = false;
    RawTerminal() {
        if (tcgetattr(STDIN_FILENO, &old_tio) != 0) return;
        termios raw = old_tio;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
        old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
        ok = true;
    }
    ~RawTerminal() {
        if (!ok) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }
};

// 方向键解析结果
enum class KeyDir { NONE, UP, DOWN, LEFT, RIGHT, QUIT };
KeyDir poll_key();

// 全局急停标志 + 信号处理（Ctrl+C）
extern volatile std::sig_atomic_t g_rl_stop;
extern void rl_signal_handler(int);
