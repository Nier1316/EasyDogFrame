# 四足机器狗仿真 — C++ 实时控制 API

## 概述

MATLAB 仿真程序 `quadruped_realtime.m` 作为 **TCP 服务器** 启动，
你的 C++ 程序作为 **TCP 客户端** 连接，以 **48 字节二进制帧** 发送 12 个关节角度，
仿真 3D 模型实时响应。

---

## 快速启动

### MATLAB 端（先启动）

```matlab
% 默认端口 12345
quadruped_realtime

% 自定义端口
quadruped_realtime(12346)
```

启动后图窗出现，标题显示"等待 C++ 客户端连接..."
此时 C++ 端即可连接。

### C++ 端（后启动）

```cpp
#include "SimSync.h"

SimSync sim("127.0.0.1", 12345);

// 每帧发送
float joints[12] = { 0, -30, 60,    // FL θ1, θ2, θ3
                     0, -30, 60,    // FR
                     0, -30, 60,    // RL
                     0, -30, 60 };  // RR
sim.send(joints);
```

---

## 通信协议

### 传输层

| 项目 | 值 |
|------|----|
| 传输层 | TCP |
| MATLAB 角色 | 服务器 (Server) |
| C++ 角色 | 客户端 (Client) |
| 默认端口 | **12345** |
| 编码 | 二进制, Little-Endian |
| 每帧大小 | **48 字节** (12 × float32) |

### 数据格式

**每帧 48 字节**，12 个 `float32`，顺序如下：

| 偏移 | 字节 | 类型 | 符号 | 范围 | 说明 |
|------|------|------|------|------|------|
| 0 | 4 | float32 | FL_θ₁ | -60 ~ 0 | FL 髋外摆 |
| 4 | 4 | float32 | FL_θ₂ | -45 ~ 90 | FL 大腿 |
| 8 | 4 | float32 | FL_θ₃ | 60 ~ 180 | FL 小腿 |
| 12 | 4 | float32 | FR_θ₁ | -60 ~ 0 | FR 髋外摆 |
| 16 | 4 | float32 | FR_θ₂ | -45 ~ 90 | FR 大腿 |
| 20 | 4 | float32 | FR_θ₃ | 60 ~ 180 | FR 小腿 |
| 24 | 4 | float32 | RL_θ₁ | -60 ~ 0 | RL 髋外摆 |
| 28 | 4 | float32 | RL_θ₂ | -45 ~ 90 | RL 大腿 |
| 32 | 4 | float32 | RL_θ₃ | 60 ~ 180 | RL 小腿 |
| 36 | 4 | float32 | RR_θ₁ | -60 ~ 0 | RR 髋外摆 |
| 40 | 4 | float32 | RR_θ₂ | -45 ~ 90 | RR 大腿 |
| 44 | 4 | float32 | RR_θ₃ | 60 ~ 180 | RR 小腿 |

**角度单位：度（°）**，不是弧度。

> 所有角度单位均为**度**。MATLAB 内部会自动转换并钳位到限位范围。

---

## C++ 示例代码

### `SimSync.h` — 单头文件，直接包含使用

```cpp
// SimSync.h — 四足仿真 TCP 同步客户端
// 使用: #include "SimSync.h"
// 依赖: 标准库 + POSIX socket (Linux) / Winsock (Windows)

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

class SimSync {
public:
    /// @brief 连接 MATLAB 仿真
    /// @param ip    MATLAB 所在 IP, 本地仿真填 "127.0.0.1"
    /// @param port  端口号, 默认 12345
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

    /// @brief 发送一帧关节角
    /// @param joints 长度为 12 的 float 数组
    ///        [FLθ1, FLθ2, FLθ3, FRθ1, FRθ2, FRθ3,
    ///         RLθ1, RLθ2, RLθ3, RRθ1, RRθ2, RRθ3]
    ///        单位: 度 (°)
    bool send(const float joints[12]) {
        if (sock_ == INVALID_SOCK) return false;
        int ret = ::send(sock_, (const char*)joints, 48, 0);
        return ret == 48;
    }

private:
    socket_t sock_ = INVALID_SOCK;
};
```

### 使用示例

```cpp
#include "SimSync.h"
#include <thread>
#include <chrono>

int main() {
    SimSync sim("127.0.0.1", 12345);

    if (!sim.connected()) {
        printf("无法连接 MATLAB 仿真\n");
        return -1;
    }
    printf("已连接仿真\n");

    // 站立姿态
    float joints[12] = {
         0, -30, 60,    // FL
         0, -30, 60,    // FR
         0, -30, 60,    // RL
         0, -30, 60,    // RR
    };
    sim.send(joints);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 前摆: 抬起 FL 腿
    for (int i = 0; i < 50; i++) {
        float t = i / 50.0f;  // 0→1
        float lift = (1 - cos(t * 3.14159f)) * 0.5f;  // 平滑起落

        joints[0] = 5;                    // FL θ1 (略外展)
        joints[1] = -60 * lift;           // FL θ2 (前摆)
        joints[2] = 60 + 40 * lift;       // FL θ3 (抬腿时小腿收)

        sim.send(joints);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
```

---

## 坐标系对照

| 坐标系 | X+ | Y+ | Z+ |
|--------|----|----|----|
| **身体坐标系** (狗体中心) | 向前 | 向左 | 向上 |
| **髋坐标系** (每条腿) | 向后 | 向外翻 | 向上 |

你的 C++ 代码中关节角定义与 `leg_kinematics.m` / `quadruped_kinematics.m` 完全一致：

| 关节 | 正方向 | 零位 |
|------|--------|------|
| θ₁ 髋外摆 | 向外翻 | 30°（相对身体竖直向下） |
| θ₂ 大腿 | 向后摆 | 0°（竖直向下） |
| θ₃ 小腿 | 向后弯 | 0°（竖直向下） |

---

## 测试方法（无 C++ 环境）

用 bash 发送测试帧：

```bash
# 发送站立姿态
python3 -c "
import socket, struct
s = socket.socket()
s.connect(('127.0.0.1', 12345))
# 12 个 float32
angles = [0,-30,60, 0,-30,60, 0,-30,60, 0,-30,60]
s.send(struct.pack('12f', *angles))
s.close()
"
```

--- 

## 关闭

- **关闭图窗** 即可停止 TCP 服务器
- 或 **Ctrl+C** 终止 MATLAB 脚本
- C++ 端调用 `SimSync` 析构自动断开