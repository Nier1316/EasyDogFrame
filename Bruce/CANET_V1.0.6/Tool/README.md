# 电机控制上位机 GUI 应用

## 📋 项目概述

**MotorControlTool** 是一个基于 Qt 5 的电机控制上位机 GUI 应用，用于计算和显示电机控制的 CAN 指令。该应用不需要实际控制电机，只需通过 `ele_motor_drive` 中的接口计算对应的 CAN 指令并在 GUI 中实时显示。

### 主要特性

- ✅ **多电机支持**：支持 4 个 CAN 口（CAN0~CAN3），每个口可控制 3 个电机
- ✅ **三种控制模式**：阻抗模式、速度模式、位置模式
- ✅ **特殊指令**：使能、失能、置零、清错误
- ✅ **参数读写**：支持读写电机参数
- ✅ **参数验证**：自动验证和截断参数到允许范围
- ✅ **实时显示**：以十六进制格式显示计算出的 CAN 指令
- ✅ **友好界面**：清晰的分组布局，易于使用

---

## 🏗️ 项目结构

```
Tool/
├── bin/
│   └── MotorControlTool          # 可执行文件
├── build/
│   ├── obj/                      # 对象文件
│   └── moc/                      # MOC生成文件
├── main.cpp                      # 主程序入口
├── motor_control_gui.h           # GUI头文件
├── motor_control_gui.cpp         # GUI实现
├── MotorControlTool.pro          # Qt项目配置
├── README.md                     # 本文件
└── TASK_LIST.md                  # 任务列表
```

---

## 📦 依赖项

### 系统要求

- **操作系统**：Linux（x86_64）
- **Qt 版本**：Qt 5.x 或更高版本
- **C++ 编译器**：支持 C++14 标准的编译器（如 g++ 5.0+）
- **构建工具**：qmake、make

### 库依赖

- `libmotor_framework.a` - 电机控制框架静态库
- `ele_motor_drive.h/cpp` - 电机驱动扩展模块
- `pthread` - POSIX 线程库
- Qt 5 Core、Gui、Widgets 库

---

## 🔨 编译方法

### 前置条件

确保已编译主项目的静态库：

```bash
cd /home/sysu/Desktop/Dog_Train/Bruce/Bruce/CANET_V1.0.6
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

### 编译 GUI 应用

```bash
cd /home/sysu/Desktop/Dog_Train/Bruce/Bruce/CANET_V1.0.6/Tool
qmake MotorControlTool.pro
make -j$(nproc)
```

编译完成后，可执行文件位于 `bin/MotorControlTool`。

### 清理编译文件

```bash
cd /home/sysu/Desktop/Dog_Train/Bruce/Bruce/CANET_V1.0.6/Tool
make clean
```

---

## 🚀 运行应用

### 启动应用

```bash
./bin/MotorControlTool
```

或者直接双击可执行文件。

### 应用窗口

应用启动后会显示一个 GUI 窗口，包含以下主要区域：

1. **电机选择区**：选择 CAN 口和电机 ID
2. **电机控制区**：特殊指令按钮（使能、失能、置零、清错误）
3. **控制模式区**：选择控制模式（阻抗、速度、位置）
4. **参数设置区**：根据选择的模式输入相应参数
5. **参数读写区**：读写电机参数
6. **结果显示区**：显示计算出的 CAN 指令

---

## 📖 使用指南

### 1. 选择电机

在**电机选择**区域：
- 从 **CAN 口** 下拉框选择 CAN0~CAN3
- 从 **电机 ID** 微调框选择 1~3
- 当前选择会显示在 **当前选择** 标签中

### 2. 电机控制

点击以下按钮发送特殊指令：
- **使能**：启用电机
- **失能**：禁用电机
- **置零**：将电机位置置零
- **清错误**：清除电机错误状态

对应的 CAN 指令会显示在**结果显示**区域。

### 3. 控制模式选择

选择以下三种控制模式之一：

#### 🔹 阻抗模式
用于力控制，需要设置以下参数：
- **位置 (rad)**：目标位置，范围 ±12.5
- **速度 (rad/s)**：目标速度，范围 ±14.0
- **Kp**：位置增益，范围 0~500
- **Kd**：阻尼增益，范围 0~100
- **扭矩 (Nm)**：目标扭矩，范围 ±10.0

点击**计算阻抗指令**按钮生成 CAN 指令。

#### 🔹 速度模式
用于速度控制，需要设置以下参数：
- **速度 (rad/s)**：目标速度，范围 ±14.0
- **Kp**：速度增益，范围 0~500
- **Ki**：积分增益，范围 0~10000

点击**计算速度指令**按钮生成 CAN 指令。

#### 🔹 位置模式
用于位置控制，需要设置以下参数：
- **位置 (rad)**：目标位置，范围 ±12.5
- **Kvp**：速度前馈增益，范围 0~500
- **Kp**：位置增益，范围 0~500
- **Kd**：阻尼增益，范围 0~100
- **Kvi**：速度积分增益，范围 0~10000

点击**计算位置指令**按钮生成 CAN 指令。

### 4. 参数读写

在**参数读写**区域：
- 从**参数类型**下拉框选择要读写的参数
- 输入**参数值**（仅写参数时需要）
- 点击**读参数**或**写参数**按钮

支持的参数类型：
- CAN_ID
- Current_Limit
- Max_Angle
- Min_Angle

### 5. 查看结果

所有计算出的 CAN 指令都会显示在**结果显示**区域，格式为：

```
[指令名称]
CAN ID: 0x[ID]
Data: [HEX] [HEX] [HEX] [HEX] [HEX] [HEX] [HEX] [HEX]
```

例如：
```
阻抗控制指令
CAN ID: 0x001
Data: 00 01 02 03 04 05 06 07
```

---

## 📊 参数范围参考

| 参数 | 最小值 | 最大值 | 单位 |
|------|--------|--------|------|
| 位置 (pos) | -12.5 | 12.5 | rad |
| 速度 (vel) | -14.0 | 14.0 | rad/s |
| Kp | 0 | 500 | - |
| Kd | 0 | 100 | - |
| Ki | 0 | 10000 | - |
| 扭矩 (torque) | -10.0 | 10.0 | Nm |

**注意**：所有参数会自动截断到允许范围内，无需手动检查。

---

## 🔧 技术细节

### 架构设计

应用采用分层架构：

```
GUI 层 (MotorControlGUI)
    ↓
参数验证层 (MotorParamValidator)
    ↓
编解码层 (MotorParamCodec)
    ↓
电机驱动层 (ele_motor_drive)
    ↓
框架层 (libmotor_framework)
```

### 关键类

#### MotorControlGUI
- 主窗口类，继承自 `QMainWindow`
- 管理所有 UI 组件和事件处理
- 调用 `ele_motor_drive` 接口计算 CAN 指令

#### MotorParamValidator
- 参数验证器，提供参数范围检查和截断功能
- 确保所有参数在允许范围内

#### MotorParamCodec
- 参数编解码器，提供参数打包和解包功能
- 支持 float 和整数之间的转换

### CAN 指令格式

#### 特殊指令
```
{0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, special_byte}
```
- 0xFE：使能
- 0xFD：失能
- 0xFC：置零
- 0xFB：清错误

#### 参数指令
```
{0x80, val[0], val[1], val[2], val[3], RW, param_type, 0xEC}
```
- RW：0 = 读，1 = 写
- param_type：参数类型（MOTOR_WR_* 宏）

#### 控制指令
根据控制模式进行位域打包，包含位置、速度、增益等参数。

---

## 🐛 故障排除

### 编译错误

**问题**：找不到 Qt 头文件
```
fatal error: QMainWindow: No such file or directory
```

**解决**：
```bash
# 检查 Qt 是否安装
qmake --version

# 如果未安装，使用包管理器安装
sudo apt-get install qt5-default
```

**问题**：找不到 libmotor_framework.a
```
cannot find -lmotor_framework
```

**解决**：
1. 确保已编译主项目
2. 检查 `../lib` 目录是否存在库文件
3. 修改 MotorControlTool.pro 中的库路径

### 运行错误

**问题**：应用无法启动
```
./bin/MotorControlTool: command not found
```

**解决**：
```bash
# 检查文件是否存在
ls -l ./bin/MotorControlTool

# 检查权限
chmod +x ./bin/MotorControlTool

# 运行应用
./bin/MotorControlTool
```

**问题**：缺少 Qt 库
```
error while loading shared libraries: libQt5Core.so.5
```

**解决**：
```bash
# 安装 Qt 运行库
sudo apt-get install libqt5core5a libqt5gui5 libqt5widgets5
```

---

## 📝 开发指南

### 添加新的控制模式

1. 在 `motor_control_gui.h` 中添加新的 UI 组件
2. 在 `motor_control_gui.cpp` 中实现 `createParameterGroup()` 的新分组
3. 添加对应的槽函数处理计算逻辑
4. 连接信号槽

### 添加新的参数类型

1. 在 `motor_control_gui.cpp` 的 `createParameterRWGroup()` 中添加新的参数类型到下拉框
2. 参数类型定义在 `ele_motor_drive.h` 中的 `MOTOR_WR_*` 宏

### 修改 CAN 帧显示格式

编辑 `motor_control_gui.cpp` 中的 `displayCanFrame()` 函数。

---

## 📄 许可证

本项目是 CANET 电机控制框架的一部分。

---

## 👥 贡献

欢迎提交 Issue 和 Pull Request。

---

## 📞 联系方式

如有问题或建议，请联系项目维护者。

---

## 🔄 更新日志

### v1.0.0 (2026-05-12)
- ✅ 初始版本发布
- ✅ 支持三种控制模式
- ✅ 支持特殊指令和参数读写
- ✅ 完整的参数验证和编解码功能
