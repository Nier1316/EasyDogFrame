/**
 * @file    main.cpp
 * @brief   达妙 DM-USB2FDCAN 探针工具（独立小工具，不碰主工程）
 *
 * 通过 /dev/ttyACM0（CDC 虚拟串口）与达妙 USB2CAN 模块收发 CAN 帧，
 * 用于验证「达妙 USB2CAN 替代 CANET」的链路可行性。
 *
 * 协议参考 Deadline039/DM-USB2CAN 固件源码（STM32, USB-CDC）：
 *   主机→设备（发 CAN 数据帧，29 字节）:
 *     [0]=0x55 [1]=0xAA [2]=rsv [3]=rsv
 *     [4..7]=send_times(LE) [8..11]=gap(LE, /10=ms)
 *     [12]=IDE(0=std) [13..16]=CAN ID(LE) [17]=RTR(0=data)
 *     [18]=DLC [19]=idAcc [20]=dataAcc [21..28]=data×8
 *   主机→设备（波特率）: 0x55 0x05 <idx> 0xAA 0x55   (idx: 0=1000k 1=800k 3=500k ...)
 *   主机→设备（心跳）:   0x55 0x04 0x00 0xAA 0x55
 *   设备→主机（上报, 16 字节）:
 *     [0]=0xAA [1]=cmd(0x11=收到CAN,0x12=发送成功,0x01=收失败,0x02=发失败,0x00=心跳)
 *     [2]=dlc(bit0-5)|idType(bit6)|dataType(bit7)
 *     [3..6]=CAN ID(LE) [7..14]=data×8 [15]=0x55
 *
 * 交互命令：
 *   b <idx>    设 CAN 波特率（0=1000k 3=500k，默认 0）
 *   h          发心跳（验证协议是否吻合）
 *   r <id_hex> 发「读控制模式」只读参数帧到电机 CAN ID（安全，不动电机）
 *   c <id_hex> <d0..d7>  发自定义标准 CAN 数据帧
 *   q          退出
 *
 * 安全提示：r 只读安全；c 若发的是控制命令（如阻抗位置指令）电机可能动作，
 *           测试前请确保电机未使能或处于安全姿态。
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cstdint>
#include <vector>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

// 波特率索引表（固件 USB_CAN_SetBaudRate）
static const char* BAUD_NAMES[] = {
    "1000k", "800k", "666.6k", "500k", "400k", "250k", "200k", "125k",
    "100k", "80k", "50k", "40k", "20k", "10k", "5k",
};

static int open_serial(const char* dev) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { std::perror("[open]"); return -1; }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { std::perror("[tcgetattr]"); ::close(fd); return -1; }
    cfmakeraw(&tty);
    // CDC 虚拟串口经 USB bulk 传输，波特率无实际意义，设 115200 即可
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;   // 100ms 超时
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { std::perror("[tcsetattr]"); ::close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static bool write_all(int fd, const std::vector<uint8_t>& buf) {
    ssize_t off = 0;
    while (off < (ssize_t)buf.size()) {
        ssize_t n = ::write(fd, buf.data() + off, buf.size() - off);
        if (n < 0) { std::perror("[write]"); return false; }
        off += n;
    }
    return true;
}

// 构建标准 CAN 数据帧（29 字节，见文件头注释）
static std::vector<uint8_t> build_can_frame(uint32_t id, const uint8_t* data, uint8_t dlc) {
    std::vector<uint8_t> f(29, 0);
    f[0] = 0x55; f[1] = 0xAA;
    f[12] = 0;                                 // IDE: 标准帧
    f[13] = (uint8_t)(id & 0xff);
    f[14] = (uint8_t)((id >> 8) & 0xff);
    f[15] = (uint8_t)((id >> 16) & 0xff);
    f[16] = (uint8_t)((id >> 24) & 0xff);
    f[17] = 0;                                 // RTR: 数据帧
    f[18] = dlc;
    if (dlc > 8) dlc = 8;
    for (int i = 0; i < dlc; i++) f[21 + i] = data[i];
    return f;
}

// 解析并打印设备上报帧（0xAA ... 0x55, 16 字节）
static void print_report(const uint8_t* f) {
    uint8_t  cmd = f[1];
    uint8_t  b2  = f[2];
    uint8_t  dlc = b2 & 0x3f;
    int      id_type  = (b2 >> 6) & 1;
    int      data_type = (b2 >> 7) & 1;
    uint32_t id = (uint32_t)f[3] | ((uint32_t)f[4] << 8) |
                  ((uint32_t)f[5] << 16) | ((uint32_t)f[6] << 24);
    const char* name = cmd == 0x11 ? "RX_CAN" : cmd == 0x12 ? "TX_OK"
                     : cmd == 0x01 ? "RX_FAIL" : cmd == 0x02 ? "TX_FAIL"
                     : cmd == 0x00 ? "HEARTBEAT" : "?";
    std::printf("[%s] id=0x%03X dlc=%u %s %s data=",
                name, id, dlc, id_type ? "EXT" : "STD", data_type ? "RTR" : "DATA");
    for (int i = 0; i < dlc; i++) std::printf("%02X ", f[7 + i]);
    std::printf("\n");
}

int main(int argc, char** argv) {
    const char* dev = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int         baud_idx = argc > 2 ? std::atoi(argv[2]) : 0;

    std::printf("== 达妙 USB2CAN 探针 ==\n");
    std::printf("设备: %s   初始波特率: [%d]=%s\n\n", dev, baud_idx,
                baud_idx >= 0 && baud_idx < 15 ? BAUD_NAMES[baud_idx] : "?");

    int fd = open_serial(dev);
    if (fd < 0) return 1;
    std::printf("[OK] 已打开 %s\n", dev);

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    // 启动即设波特率 + 发心跳（验证协议是否吻合）
    auto set_baud = [&](int idx) {
        std::vector<uint8_t> b = {0x55, 0x05, (uint8_t)idx, 0xAA, 0x55};
        write_all(fd, b);
        std::printf("[cmd] 设波特率 [%d]=%s\n", idx, BAUD_NAMES[idx]);
    };
    auto heartbeat = [&]() {
        std::vector<uint8_t> h = {0x55, 0x04, 0x00, 0xAA, 0x55};
        write_all(fd, h);
        std::printf("[cmd] 心跳\n");
    };
    set_baud(baud_idx);
    usleep(200000);
    heartbeat();

    std::printf("\n命令: b <idx>波特率 | h心跳 | r <id_hex>读控制模式 | "
                "c <id_hex> <d0..d7>发帧 | q退出\n\n");

    std::vector<uint8_t> rx;   // 串口接收缓冲
    rx.reserve(512);

    while (!g_stop) {
        pollfd pfd[2] = {
            {fd, POLLIN, 0},
            {STDIN_FILENO, POLLIN, 0},
        };
        int pr = ::poll(pfd, 2, 200);
        if (pr < 0) break;

        // ---- 读串口 ----
        if (pfd[0].revents & POLLIN) {
            uint8_t tmp[256];
            ssize_t n = ::read(fd, tmp, sizeof(tmp));
            if (n > 0) {
                rx.insert(rx.end(), tmp, tmp + n);
                // 解析缓冲中的帧
                size_t i = 0;
                while (i < rx.size()) {
                    if (rx[i] == 0xAA && i + 15 < rx.size()) {
                        if (rx[i + 15] == 0x55) {          // 完整 16 字节上报帧
                            print_report(&rx[i]);
                            i += 16;
                            continue;
                        } else {
                            i++;                            // 0xAA 后无 0x55 尾，向前挪
                            continue;
                        }
                    } else if (rx[i] == 0x55 && i + 15 < rx.size()) {
                        // 0x55 开头：可能是发送帧回显（29字节）或帧尾不完整，仅 hex 提示
                        // 这里做粗粒度：打印 29 字节回显
                        bool is_echo = (rx[i + 1] == 0xAA) && (i + 28 < rx.size());
                        if (is_echo) {
                            std::printf("[echo] 发送帧回显 %d 字节:", 29);
                            for (int k = 0; k < 29; k++) std::printf("%02X ", rx[i + k]);
                            std::printf("\n");
                            i += 29;
                            continue;
                        }
                        i++;
                        continue;
                    } else {
                        i++;
                    }
                }
                // 保留未完成的尾部（最多 64 字节）
                if (rx.size() > 64) {
                    size_t keep = rx.size() >= 64 ? 64 : rx.size();
                    rx.erase(rx.begin(), rx.end() - keep);
                }
            }
        }

        // ---- 读 stdin 命令 ----
        if (pfd[1].revents & POLLIN) {
            char line[128];
            if (!std::fgets(line, sizeof(line), stdin)) break;
            if (line[0] == 'q' || line[0] == 'Q') break;

            if (line[0] == 'b') {
                int idx = std::atoi(line + 1);
                if (idx < 0 || idx > 14) { std::printf("[!] 波特率索引 0-14\n"); continue; }
                set_baud(idx);
            } else if (line[0] == 'h') {
                heartbeat();
            } else if (line[0] == 'r') {
                unsigned mid = 0;
                if (std::sscanf(line + 1, "%x", &mid) != 1 || mid < 1 || mid > 15) {
                    std::printf("[!] r 用法: r <电机CAN_ID_hex> (1-F)\n"); continue;
                }
                // 读控制模式(0x5B) 只读参数帧：{0x80,0,0,0,0, RW=0, 0x5B, 0xEC}
                uint8_t d[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5B, 0xEC};
                std::vector<uint8_t> f = build_can_frame(mid, d, 8);
                std::printf("[cmd] 读电机 %u 控制模式参数帧 (CAN ID=0x%X)\n", mid, mid);
                write_all(fd, f);
            } else if (line[0] == 'c') {
                unsigned id = 0;
                uint8_t  d[8] = {0};
                int      nd = std::sscanf(line + 1, "%x %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                                         &id, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7]);
                if (nd < 1 || id > 0x7FF) { std::printf("[!] c 用法: c <id_hex> <d0..d7>\n"); continue; }
                int dlc = nd - 1;
                std::vector<uint8_t> f = build_can_frame(id, d, (uint8_t)dlc);
                std::printf("[cmd] 发 CAN 帧 id=0x%03X dlc=%d\n", id, dlc);
                write_all(fd, f);
            }
        }
    }

    ::close(fd);
    std::printf("[OK] 退出\n");
    return 0;
}
