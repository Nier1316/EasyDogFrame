#pragma once
#include <cstring>
#include <cstdint>
#include <cmath>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    constexpr socket_t INVALID_SOCK = INVALID_SOCKET;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    using socket_t = int;
    constexpr socket_t INVALID_SOCK = -1;
    inline void closesocket(int fd) { close(fd); }
#endif

/// @brief 与 MATLAB 四足仿真 (quadruped_realtime) 的 TCP 同步客户端
///
/// 用法:
///   SimSync sim("127.0.0.1", 12345);
///   sim.send_deg(joints_deg);   // 传度
///   sim.send_rad(joints_rad);   // 传弧度（自动转度）
class SimSync {
public:
    SimSync(const char* ip = "127.0.0.1", uint16_t port = 12345) {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2), &wsa);
#endif
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == INVALID_SOCK) return;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        inet_pton(AF_INET, ip, &addr.sin_addr);

        if (connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            closesocket(sock_);
            sock_ = INVALID_SOCK;
        }
    }

    ~SimSync() {
        if (sock_ != INVALID_SOCK) closesocket(sock_);
#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool connected() const { return sock_ != INVALID_SOCK; }

    /// @brief 发送一帧关节角 (单位: 度)
    bool send_deg(const float joints_deg[12]) {
        if (sock_ == INVALID_SOCK) return false;
        int ret = ::send(sock_, (const char*)joints_deg, 48, 0);
        return ret == 48;
    }

    /// @brief 发送一帧关节角 (单位: 弧度, 自动转度)
    bool send_rad(const float joints_rad[12]) {
        float joints_deg[12];
        for (int i = 0; i < 12; i++) {
            joints_deg[i] = joints_rad[i] * (180.0f / M_PI);
        }
        return send_deg(joints_deg);
    }

private:
    socket_t sock_ = INVALID_SOCK;
};
