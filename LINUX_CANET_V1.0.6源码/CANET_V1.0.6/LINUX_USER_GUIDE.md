# CANET V1.0.6 Linux 用户使用指南

## 目录
1. [项目概述](#项目概述)
2. [系统要求](#系统要求)
3. [编译和安装](#编译和安装)
4. [快速开始](#快速开始)
5. [API 参考](#api-参考)
6. [示例代码](#示例代码)
7. [常见问题](#常见问题)

---

## 项目概述

**CANET** 是一个跨平台的 CAN 总线通信库，支持通过 TCP/IP 网络进行远程 CAN 设备通信。

### 主要特性
- ✅ 跨平台支持（Linux x64、Windows x86/x64）
- ✅ TCP 客户端/服务器模式
- ✅ 线程安全的数据队列
- ✅ 支持多设备管理
- ✅ 灵活的服务层架构
- ✅ 完整的日志系统

### 应用场景
- 远程 CAN 设备通信
- CAN 数据采集和转发
- 分布式 CAN 网络
- 工业控制系统

---

## 系统要求

### 硬件要求
- CPU: x86_64 架构
- 内存: 最少 256MB
- 网络: 支持 TCP/IP

### 软件要求
- **操作系统**: Linux (Ubuntu 16.04+, CentOS 7+, Debian 9+)
- **编译工具**: 
  - GCC/G++ 5.0 或更高版本
  - CMake 3.8 或更高版本
  - Make
- **依赖库**:
  - pthread (通常已包含)
  - C++11 标准库

### 安装编译工具（Ubuntu/Debian）
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git
```

### 安装编译工具（CentOS/RHEL）
```bash
sudo yum groupinstall -y "Development Tools"
sudo yum install -y cmake
```

---

## 编译和安装

### 1. 获取源代码
```bash
cd /path/to/CANET_V1.0.6
```

### 2. 创建编译目录
```bash
mkdir -p build
cd build
```

### 3. 配置编译
```bash
# Debug 版本
cmake -DCMAKE_BUILD_TYPE=Debug ..

# 或 Release 版本（推荐用于生产环境）
cmake -DCMAKE_BUILD_TYPE=Release ..
```

### 4. 编译
```bash
make -j$(nproc)
```

### 5. 编译输出
编译完成后，输出文件位置：
```
bin/CANET_TCP/linux_x64/Debug/    # Debug 版本
bin/CANET_TCP/linux_x64/Release/  # Release 版本
```

输出文件包括：
- `libCANET_TCP.so` - 动态库
- `libCANET_TCP.a` - 静态库
- `canet_test` - 测试程序

### 6. 安装到系统（可选）
```bash
# 复制库文件到系统目录
sudo cp bin/CANET_TCP/linux_x64/Release/libCANET_TCP.so /usr/local/lib/
sudo cp bin/CANET_TCP/linux_x64/Release/libCANET_TCP.a /usr/local/lib/

# 复制头文件
sudo mkdir -p /usr/local/include/canet
sudo cp src/CANET/*.h /usr/local/include/canet/
sudo cp src/base/*.h /usr/local/include/canet/

# 更新库缓存
sudo ldconfig
```

---

## 快速开始

### 最简单的例子：TCP 服务器模式

```cpp
#include <stdio.h>
#include "CANET.h"
#include "common.h"

int main() {
    // 1. 打开设备
    if (VCI_OpenDevice(VCI_CANETE, 0, 0) != STATUS_OK) {
        printf("打开设备失败\n");
        return -1;
    }

    // 2. 设置为 TCP 服务器模式
    DWORD workMode = TCP_SERVER;
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_TCP_TYPE, &workMode);

    // 3. 设置监听端口
    DWORD port = 4001;
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_SRCPORT, &port);

    // 4. 初始化 CAN
    VCI_InitCAN(VCI_CANETE, 0, 0, NULL);

    // 5. 启动 CAN
    if (VCI_StartCAN(VCI_CANETE, 0, 0) != STATUS_OK) {
        printf("启动 CAN 失败\n");
        return -1;
    }

    printf("服务器启动，监听端口 %d\n", port);

    // 6. 接收数据
    VCI_CAN_OBJ canFrames[100];
    while (1) {
        ULONG cnt = VCI_Receive(VCI_CANETE, 0, 0, canFrames, 100);
        if (cnt > 0) {
            printf("接收到 %d 个 CAN 帧\n", cnt);
            
            // 处理接收到的数据
            for (ULONG i = 0; i < cnt; i++) {
                printf("  帧 %d: ID=0x%x, 长度=%d\n", 
                    i, canFrames[i].ID, canFrames[i].DataLen);
            }
        }
        cc_sleep(10);
    }

    // 7. 清理资源
    VCI_ResetCAN(VCI_CANETE, 0, 0);
    VCI_CloseDevice(VCI_CANETE, 0);

    return 0;
}
```

### 编译和运行
```bash
# 编译
g++ -std=c++11 -o my_app my_app.cpp \
    -I/path/to/CANET/src/CANET \
    -I/path/to/CANET/src/base \
    -L/path/to/CANET/bin/CANET_TCP/linux_x64/Release \
    -lCANET_TCP -lpthread

# 运行
./my_app
```

---

## API 参考

### 核心函数

#### 设备管理

| 函数 | 说明 |
|------|------|
| `VCI_OpenDevice(type, idx, reserved)` | 打开 CAN 设备 |
| `VCI_CloseDevice(type, idx)` | 关闭 CAN 设备 |
| `VCI_InitCAN(type, idx, chn, pInitConfig)` | 初始化 CAN 通道 |
| `VCI_StartCAN(type, idx, chn)` | 启动 CAN 通道 |
| `VCI_ResetCAN(type, idx, chn)` | 重置 CAN 通道 |

#### 数据收发

| 函数 | 说明 |
|------|------|
| `VCI_Transmit(type, idx, chn, pSend, len)` | 发送 CAN 数据 |
| `VCI_Receive(type, idx, chn, pReceive, len, waitTime)` | 接收 CAN 数据 |

#### 参数配置

| 函数 | 说明 |
|------|------|
| `VCI_SetReference(type, idx, chn, refType, pData)` | 设置参数 |
| `VCI_GetReference(type, idx, chn, refType, pData)` | 获取参数 |

### 常用参数

#### 工作模式 (CMD_TCP_TYPE)
```cpp
#define TCP_SERVER  1  // TCP 服务器模式
#define TCP_CLIENT  0  // TCP 客户端模式
```

#### 配置命令
```cpp
CMD_TCP_TYPE        // 设置 TCP 工作模式
CMD_SRCPORT         // 设置源端口（服务器模式）
CMD_DESIP           // 设置目标 IP（客户端模式）
CMD_DESPORT         // 设置目标端口（客户端模式）
CMD_CLIENT_COUNT    // 获取客户端数量（服务器模式）
CMD_CLIENT          // 获取客户端信息（服务器模式）
CMD_DISCONN_CLINET  // 断开客户端连接（服务器模式）
```

### 数据结构

#### VCI_CAN_OBJ - CAN 帧结构
```cpp
struct VCI_CAN_OBJ {
    DWORD ID;           // CAN 帧 ID
    DWORD TimeStamp;    // 时间戳
    BYTE TimeFlag;      // 时间标志
    BYTE SendType;      // 发送类型
    BYTE RemoteFlag;    // 远程帧标志
    BYTE ExternFlag;    // 扩展帧标志
    BYTE DataLen;       // 数据长度（0-8）
    BYTE Data[8];       // 数据内容
    DWORD Reserved;     // 保留字段
};
```

#### REMOTE_CLIENT - 远程客户端信息
```cpp
struct REMOTE_CLIENT {
    int iIndex;         // 客户端索引
    char szip[32];      // 客户端 IP 地址
    DWORD port;         // 客户端端口
};
```

---

## 示例代码

### 示例 1: TCP 客户端模式

```cpp
#include <stdio.h>
#include "CANET.h"
#include "common.h"

int main() {
    // 打开设备
    if (VCI_OpenDevice(VCI_CANETE, 0, 0) != STATUS_OK) {
        printf("打开设备失败\n");
        return -1;
    }

    // 设置为 TCP 客户端模式
    DWORD workMode = TCP_CLIENT;
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_TCP_TYPE, &workMode);

    // 设置服务器地址和端口
    const char* serverIp = "192.168.1.100";
    DWORD serverPort = 4001;
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_DESIP, (void*)serverIp);
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_DESPORT, &serverPort);

    // 初始化和启动
    VCI_InitCAN(VCI_CANETE, 0, 0, NULL);
    VCI_StartCAN(VCI_CANETE, 0, 0);

    printf("客户端已连接到 %s:%d\n", serverIp, serverPort);

    // 发送数据
    VCI_CAN_OBJ sendFrame;
    sendFrame.ID = 0x123;
    sendFrame.DataLen = 8;
    sendFrame.Data[0] = 0x01;
    sendFrame.Data[1] = 0x02;
    // ... 设置其他数据

    ULONG sent = VCI_Transmit(VCI_CANETE, 0, 0, &sendFrame, 1);
    printf("发送了 %d 个帧\n", sent);

    // 接收数据
    VCI_CAN_OBJ recvFrame[10];
    ULONG cnt = VCI_Receive(VCI_CANETE, 0, 0, recvFrame, 10);
    printf("接收了 %d 个帧\n", cnt);

    // 清理
    VCI_ResetCAN(VCI_CANETE, 0, 0);
    VCI_CloseDevice(VCI_CANETE, 0);

    return 0;
}
```

### 示例 2: 服务器模式 - 管理客户端

```cpp
#include <stdio.h>
#include "CANET.h"
#include "common.h"

void PrintClientInfo() {
    // 获取客户端数量
    DWORD clientCount = 0;
    VCI_GetReference(VCI_CANETE, 0, 0, CMD_CLIENT_COUNT, &clientCount);
    printf("当前连接的客户端数: %d\n", clientCount);

    // 获取每个客户端的信息
    REMOTE_CLIENT client;
    for (DWORD i = 0; i < clientCount; i++) {
        memset(&client, 0, sizeof(client));
        client.iIndex = i;
        
        if (VCI_GetReference(VCI_CANETE, 0, 0, CMD_CLIENT, &client) == STATUS_OK) {
            printf("  客户端 %d: %s:%d\n", i, client.szip, client.port);
        }
    }
}

int main() {
    // 打开设备
    VCI_OpenDevice(VCI_CANETE, 0, 0);

    // 设置为服务器模式
    DWORD workMode = TCP_SERVER;
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_TCP_TYPE, &workMode);

    // 设置监听端口
    DWORD port = 4001;
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_SRCPORT, &port);

    // 初始化和启动
    VCI_InitCAN(VCI_CANETE, 0, 0, NULL);
    VCI_StartCAN(VCI_CANETE, 0, 0);

    printf("服务器启动，监听端口 %d\n", port);

    // 主循环
    while (1) {
        // 打印客户端信息
        PrintClientInfo();

        // 接收数据
        VCI_CAN_OBJ frames[100];
        ULONG cnt = VCI_Receive(VCI_CANETE, 0, 0, frames, 100);
        if (cnt > 0) {
            printf("接收到 %d 个帧\n", cnt);
            
            // 转发给所有客户端
            VCI_Transmit(VCI_CANETE, 0, 0, frames, cnt);
        }

        cc_sleep(1000);  // 每秒检查一次
    }

    VCI_ResetCAN(VCI_CANETE, 0, 0);
    VCI_CloseDevice(VCI_CANETE, 0);

    return 0;
}
```

### 示例 3: 多设备管理

```cpp
#include <stdio.h>
#include "CANET.h"
#include "common.h"

int main() {
    const int DEVICE_COUNT = 2;

    // 打开多个设备
    for (int i = 0; i < DEVICE_COUNT; i++) {
        if (VCI_OpenDevice(VCI_CANETE, i, 0) != STATUS_OK) {
            printf("打开设备 %d 失败\n", i);
            return -1;
        }

        // 配置每个设备
        DWORD workMode = TCP_SERVER;
        VCI_SetReference(VCI_CANETE, i, 0, CMD_TCP_TYPE, &workMode);

        DWORD port = 4001 + i;  // 不同的端口
        VCI_SetReference(VCI_CANETE, i, 0, CMD_SRCPORT, &port);

        VCI_InitCAN(VCI_CANETE, i, 0, NULL);
        VCI_StartCAN(VCI_CANETE, i, 0);

        printf("设备 %d 启动，监听端口 %d\n", i, port);
    }

    // 接收数据
    VCI_CAN_OBJ frames[100];
    while (1) {
        for (int i = 0; i < DEVICE_COUNT; i++) {
            ULONG cnt = VCI_Receive(VCI_CANETE, i, 0, frames, 100);
            if (cnt > 0) {
                printf("设备 %d 接收到 %d 个帧\n", i, cnt);
            }
        }
        cc_sleep(10);
    }

    // 关闭所有设备
    for (int i = 0; i < DEVICE_COUNT; i++) {
        VCI_ResetCAN(VCI_CANETE, i, 0);
        VCI_CloseDevice(VCI_CANETE, i);
    }

    return 0;
}
```

---

## 常见问题

### Q1: 编译时出现 "undefined reference to pthread_create"

**A:** 需要链接 pthread 库。编译时添加 `-lpthread` 选项：
```bash
g++ -std=c++11 -o my_app my_app.cpp -lCANET_TCP -lpthread
```

### Q2: 运行时出现 "libCANET_TCP.so: cannot open shared object file"

**A:** 库文件不在系统路径中。解决方案：

方案 1: 设置 LD_LIBRARY_PATH
```bash
export LD_LIBRARY_PATH=/path/to/lib:$LD_LIBRARY_PATH
./my_app
```

方案 2: 安装到系统目录
```bash
sudo cp libCANET_TCP.so /usr/local/lib/
sudo ldconfig
```

方案 3: 使用静态库
```bash
g++ -std=c++11 -o my_app my_app.cpp \
    /path/to/libCANET_TCP.a -lpthread
```

### Q3: 客户端无法连接到服务器

**A:** 检查以下几点：
1. 服务器是否正常运行
2. 防火墙是否开放了相应端口
3. IP 地址和端口是否正确
4. 网络连接是否正常

```bash
# 测试网络连接
ping <server_ip>

# 检查端口是否开放
netstat -tlnp | grep 4001

# 使用 telnet 测试连接
telnet <server_ip> 4001
```

### Q4: 数据接收为空

**A:** 可能的原因：
1. 没有数据发送
2. 接收超时设置不当
3. 缓冲区已满

```cpp
// 使用带超时的接收
int waitTime = 1000;  // 等待 1000ms
ULONG cnt = VCI_Receive(VCI_CANETE, 0, 0, frames, 100, waitTime);
```

### Q5: 如何在后台运行程序

**A:** 使用 nohup 或 systemd 服务

方案 1: nohup
```bash
nohup ./my_app > app.log 2>&1 &
```

方案 2: systemd 服务
```bash
# 创建服务文件
sudo nano /etc/systemd/system/canet.service
```

```ini
[Unit]
Description=CANET Service
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/path/to/app
ExecStart=/path/to/app/my_app
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# 启用和启动服务
sudo systemctl daemon-reload
sudo systemctl enable canet
sudo systemctl start canet

# 查看状态
sudo systemctl status canet
```

### Q6: 如何调试程序

**A:** 使用 GDB 调试器

```bash
# 编译时添加调试符号
g++ -std=c++11 -g -o my_app my_app.cpp -lCANET_TCP -lpthread

# 启动 GDB
gdb ./my_app

# GDB 命令
(gdb) run                    # 运行程序
(gdb) break main             # 设置断点
(gdb) continue               # 继续执行
(gdb) print variable_name    # 打印变量
(gdb) backtrace              # 查看调用栈
(gdb) quit                   # 退出
```

### Q7: 性能优化建议

**A:** 
1. 使用 Release 版本编译
2. 增加接收缓冲区大小
3. 使用多线程处理数据
4. 调整 cc_sleep 的延迟时间

```cpp
// 增加缓冲区
VCI_CAN_OBJ frames[1000];  // 从 100 增加到 1000

// 使用多线程
std::thread recvThread([](){ 
    // 接收数据
});
std::thread sendThread([](){ 
    // 发送数据
});
```

---

## 技术支持

如有问题，请检查：
1. 源代码中的 `test.cpp` 示例
2. 库文件中的头文件注释
3. 系统日志文件

---

**最后更新**: 2026-05-10  
**版本**: CANET V1.0.6
