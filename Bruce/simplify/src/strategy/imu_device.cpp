/**
 * @file    imu_device.cpp
 * @brief   维特 HWT606 IMU 串口读取实现（WIT 标准协议）
 */
#include "strategy/imu_device.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// WIT 标准协议：帧头 0x55，帧长 11 字节，末字节为 SUM（低 8 位）
static const uint8_t FRAME_HEAD = 0x55;
static const int     FRAME_LEN  = 11;
static const uint8_t TYPE_GYRO  = 0x52;   // 角速度
static const uint8_t TYPE_QUAT  = 0x59;   // 四元数

static const float DEG2RAD = 3.14159265358979323846f / 180.0f;

// 命令写寄存器三步：解锁 -> 写 -> 保存
static const uint8_t CMD_UNLOCK[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static const uint8_t CMD_SAVE[]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};

namespace {

int open_serial(const char* port, int baud) {
    int fd = ::open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::fprintf(stderr, "[IMU] open %s failed: %s\n", port, std::strerror(errno));
        return -1;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        std::fprintf(stderr, "[IMU] tcgetattr failed: %s\n", std::strerror(errno));
        ::close(fd);
        return -1;
    }

    // raw mode：8N1，无流控
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    speed_t speed;
    switch (baud) {
        case 9600:    speed = B9600;    break;
        case 19200:   speed = B19200;   break;
        case 38400:   speed = B38400;   break;
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 921600:  speed = B921600;  break;
        default:      speed = B115200;  break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::fprintf(stderr, "[IMU] tcsetattr failed: %s\n", std::strerror(errno));
        ::close(fd);
        return -1;
    }

    // 清空收发缓冲
    tcflush(fd, TCIOFLUSH);
    return fd;
}

} // namespace

ImuDevice::~ImuDevice() {
    Shutdown();
}

bool ImuDevice::Initialize(const char* port, int baud) {
    if (m_connected.load()) {
        return true;
    }
    m_fd = open_serial(port, baud);
    if (m_fd < 0) {
        return false;
    }

    m_running.store(true);
    m_thread = std::thread(&ImuDevice::ReadLoop, this);
    m_connected.store(true);
    std::printf("[IMU] opened %s @ %d baud\n", port, baud);
    return true;
}

void ImuDevice::Shutdown() {
    if (!m_connected.load()) {
        return;
    }
    m_running.store(false);
    if (m_fd >= 0) {
        ::close(m_fd);   // 让阻塞 read 返回错误退出读线程
        m_fd = -1;
    }
    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_connected.store(false);
    std::printf("[IMU] closed\n");
}

void ImuDevice::GetGyro(float& gx, float& gy, float& gz) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    gx = m_gyro[0];
    gy = m_gyro[1];
    gz = m_gyro[2];
}

void ImuDevice::GetQuat(float& w, float& x, float& y, float& z) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    w = m_quat[0];
    x = m_quat[1];
    y = m_quat[2];
    z = m_quat[3];
}

void ImuDevice::ReadLoop() {
    uint8_t byte;
    while (m_running.load()) {
        ssize_t n = ::read(m_fd, &byte, 1);
        if (n <= 0) {
            if (m_running.load()) {
                // 短暂停顿避免忙等
                usleep(1000);
            }
            continue;
        }
        ParseByte(byte);
    }
}

void ImuDevice::ParseByte(uint8_t b) {
    if (m_buf_len == 0) {
        if (b == FRAME_HEAD) {
            m_buf[m_buf_len++] = b;
        }
        return;
    }
    m_buf[m_buf_len++] = b;
    if (m_buf_len < FRAME_LEN) {
        return;
    }

    // 校验 SUM（低 8 位）
    uint8_t sum = 0;
    for (int i = 0; i < FRAME_LEN - 1; ++i) {
        sum += m_buf[i];
    }
    m_buf_len = 0;
    if (sum != m_buf[FRAME_LEN - 1]) {
        return;   // 校验失败，丢弃
    }

    const uint8_t type = m_buf[1];
    const uint8_t* d = &m_buf[2];
    if (type == TYPE_GYRO) {
        ParseGyroFrame(d);
    } else if (type == TYPE_QUAT) {
        ParseQuatFrame(d);
    }
}

void ImuDevice::ParseGyroFrame(const uint8_t* d) {
    // Wx = ((WxH<<8)|WxL)/32768*2000 (°/s) → rad/s
    auto to_rad = [](uint8_t lo, uint8_t hi) -> float {
        int16_t raw = (int16_t)((uint16_t)hi << 8 | lo);
        return (float)raw / 32768.0f * 2000.0f * DEG2RAD;
    };
    float gx = to_rad(d[0], d[1]);
    float gy = to_rad(d[2], d[3]);
    float gz = to_rad(d[4], d[5]);
    if (m_mount == ImuMount::Z_DOWN_X) {
        // 绕 X 轴翻 180°：X 不变，Y/Z 反向
        gy = -gy;
        gz = -gz;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gyro[0] = gx;
    m_gyro[1] = gy;
    m_gyro[2] = gz;
}

void ImuDevice::ParseQuatFrame(const uint8_t* d) {
    // q = ((QH<<8)|QL)/32768，输出顺序 q0(w), q1(x), q2(y), q3(z)
    auto to_q = [](uint8_t lo, uint8_t hi) -> float {
        int16_t raw = (int16_t)((uint16_t)hi << 8 | lo);
        return (float)raw / 32768.0f;
    };
    float w = to_q(d[0], d[1]);
    float x = to_q(d[2], d[3]);
    float y = to_q(d[4], d[5]);
    float z = to_q(d[6], d[7]);
    if (m_mount == ImuMount::Z_DOWN_X) {
        // 绕 X 轴翻 180°：q_body = (-x, w, -z, y)
        float nw = -x;
        float nx = w;
        float ny = -z;
        float nz = y;
        w = nw; x = nx; y = ny; z = nz;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    m_quat[0] = w;
    m_quat[1] = x;
    m_quat[2] = y;
    m_quat[3] = z;
}

bool ImuDevice::WriteReg(uint8_t addr, uint16_t value) {
    if (m_fd < 0) {
        return false;
    }
    uint8_t lo = (uint8_t)(value & 0xFF);
    uint8_t hi = (uint8_t)(value >> 8);
    uint8_t cmd[5] = {0xFF, 0xAA, addr, lo, hi};
    // 解锁 -> 写 -> 保存（每步之间给模块消化时间）
    if (::write(m_fd, CMD_UNLOCK, sizeof(CMD_UNLOCK)) != (ssize_t)sizeof(CMD_UNLOCK)) return false;
    usleep(20000);
    if (::write(m_fd, cmd, sizeof(cmd)) != (ssize_t)sizeof(cmd)) return false;
    usleep(20000);
    if (::write(m_fd, CMD_SAVE, sizeof(CMD_SAVE)) != (ssize_t)sizeof(CMD_SAVE)) return false;
    usleep(20000);
    return true;
}

bool ImuDevice::Configure() {
    if (m_fd < 0) {
        return false;
    }
    // RSW(0x02)=0x0204: 开 QUATER(bit9) + GYRO(bit2)
    bool ok = WriteReg(0x02, 0x0204);
    // RRATE(0x03)=0x09: 100Hz
    ok = WriteReg(0x03, 0x0009) && ok;
    // AXIS6(0x24)=0x01: 6 轴算法（避电机磁场干扰磁力计）
    ok = WriteReg(0x24, 0x0001) && ok;
    // BAUD(0x04)=0x06: 115200（修改后需重新 Initialize）
    ok = WriteReg(0x04, 0x0006) && ok;
    std::printf("[IMU] configure %s\n", ok ? "ok (BAUD 变更后请用 115200 重新 Initialize)" : "failed");
    return ok;
}
