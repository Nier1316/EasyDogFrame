# 电机控制上位机GUI开发任务列表

## 项目概述
创建一个C++ + Qt GUI应用，用于计算和显示电机控制的CAN指令。该应用不需要实际控制电机，只需通过ele_motor_drive中的接口计算对应的CAN指令并在GUI中显示。

**项目位置**: `/home/sysu/Desktop/Dog_Train/Bruce/Bruce/CANET_V1.0.6/Tool/`

---

## 任务1：创建Qt项目配置文件

**文件**: `MotorControlTool.pro`

**状态**: ✅ 已创建

**内容要求**:
- Qt配置（QT += core gui widgets）
- C++14标准
- 包含路径指向 `../include`
- 链接静态库 `../lib/libmotor_framework.a`
- 链接pthread库

---

## 任务2：创建GUI头文件

**文件**: `motor_control_gui.h`

**状态**: ✅ 已创建

**内容要求**:
- 继承QMainWindow
- 包含以下UI组件：
  - **电机选择**: CAN口下拉框(0-3)、电机ID微调框(1-3)
  - **电机控制**: 使能、失能、置零、清错误按钮
  - **控制模式**: 阻抗、速度、位置单选按钮
  - **阻抗模式参数**: pos、vel、kp、kd、torque输入框
  - **速度模式参数**: vel、kp、ki输入框
  - **位置模式参数**: pos、kvp、kp、kd、kvi输入框
  - **参数读写**: 参数类型下拉框、参数值输入框、读/写按钮
  - **结果显示**: 文本框显示CAN指令（16进制）

---

## 任务3：创建GUI实现文件

**文件**: `motor_control_gui.cpp`

**内容要求**:

### 3.1 构造函数和初始化
- 初始化所有UI组件
- 设置默认值（CAN口=0, 电机ID=1, 模式=阻抗）
- 连接所有信号槽
- 设置窗口大小和标题

### 3.2 电机选择相关槽函数
- `onCanPortChanged()`: CAN口改变时更新当前选择
- `onMotorIdChanged()`: 电机ID改变时更新当前选择

### 3.3 电机控制槽函数
- `onEnableMotor()`: 调用ele_motor_drive生成使能指令，显示CAN帧
- `onDisableMotor()`: 调用ele_motor_drive生成失能指令，显示CAN帧
- `onSetZero()`: 调用ele_motor_drive生成置零指令，显示CAN帧
- `onClearError()`: 调用ele_motor_drive生成清错误指令，显示CAN帧

### 3.4 模式选择槽函数
- `onControlModeChanged()`: 切换模式时显示/隐藏对应的参数组

### 3.5 参数计算槽函数
- `onCalculateImpedanceCommand()`: 
  - 读取阻抗模式参数
  - 调用MotorParamValidator验证参数
  - 调用MotorParamCodec计算CAN指令
  - 显示结果

- `onCalculateSpeedCommand()`:
  - 读取速度模式参数
  - 调用MotorParamValidator验证参数
  - 调用MotorParamCodec计算CAN指令
  - 显示结果

- `onCalculatePositionCommand()`:
  - 读取位置模式参数
  - 调用MotorParamValidator验证参数
  - 调用MotorParamCodec计算CAN指令
  - 显示结果

### 3.6 参数读写槽函数
- `onReadParameter()`:
  - 读取参数类型
  - 调用ele_motor_drive生成读参数指令
  - 显示CAN帧

- `onWriteParameter()`:
  - 读取参数类型和参数值
  - 调用ele_motor_drive生成写参数指令
  - 显示CAN帧

### 3.7 辅助函数
- `initUI()`: 初始化所有UI组件
- `createMotorSelectionGroup()`: 创建电机选择组
- `createMotorControlGroup()`: 创建电机控制组
- `createControlModeGroup()`: 创建控制模式组
- `createParameterGroup()`: 创建参数设置组
- `createParameterRWGroup()`: 创建参数读写组
- `createResultGroup()`: 创建结果显示组
- `displayCanFrame()`: 显示CAN帧（16进制格式）
- `clearResults()`: 清空结果显示
- `updateParameterUI()`: 根据模式更新参数UI

---

## 任务4：创建main.cpp

**文件**: `main.cpp`

**内容要求**:
```cpp
#include <QApplication>
#include "motor_control_gui.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    MotorControlGUI window;
    window.show();
    
    return app.exec();
}
```

---

## 任务5：编译和测试

### 5.1 编译步骤
```bash
cd /home/sysu/Desktop/Dog_Train/Bruce/Bruce/CANET_V1.0.6/Tool
qmake MotorControlTool.pro
make
```

### 5.2 运行
```bash
./MotorControlTool
```

### 5.3 测试项
- [ ] GUI窗口正常显示
- [ ] 电机选择功能正常
- [ ] 各模式参数输入框显示/隐藏正确
- [ ] 阻抗模式CAN指令计算正确
- [ ] 速度模式CAN指令计算正确
- [ ] 位置模式CAN指令计算正确
- [ ] 参数读写指令计算正确
- [ ] 特殊指令（使能/失能/置零/清错误）计算正确
- [ ] CAN帧显示格式正确（16进制）

---

## 关键实现细节

### 调用ele_motor_drive的接口

#### 参数验证
```cpp
#include "ele_motor_drive.h"

// 验证参数
float validPos = MotorParamValidator::ValidatePosition(pos);
float validVel = MotorParamValidator::ValidateVelocity(vel);
float validKp = MotorParamValidator::ValidateKp(kp);
float validKd = MotorParamValidator::ValidateKd(kd);
float validTorque = MotorParamValidator::ValidateTorque(torque);
```

#### 参数编解码
```cpp
// 量化float为整数
unsigned int posInt = MotorParamCodec::FloatToUint(validPos, -12.5f, 12.5f, 15);
unsigned int velInt = MotorParamCodec::FloatToUint(validVel, -14.0f, 14.0f, 12);

// 反量化整数为float
float decodedPos = MotorParamCodec::UintToFloat(posInt, -12.5f, 12.5f, 15);

// 参数打包
uint8_t data[8];
MotorParamCodec::Float2Bag(paramValue, RW, paramType, motorId, data);
```

### CAN帧显示格式
显示为16进制字符串，例如：
```
CAN ID: 0x001
Data: 00 01 02 03 04 05 06 07
```

### 参数范围
- 位置 (pos): ±12.5 rad
- 速度 (vel): ±14 rad/s
- Kp: 0~500
- Kd: 0~100
- Ki: 0~10000
- 扭矩 (torque): ±200 Nm

---

## 文件结构

```
Tool/
├── MotorControlTool.pro          # Qt项目配置
├── motor_control_gui.h           # GUI头文件
├── motor_control_gui.cpp         # GUI实现
├── main.cpp                      # 主程序
└── TASK_LIST.md                  # 本文件
```

---

## 依赖关系

- Qt 5.x 或更高版本
- C++14编译器
- libmotor_framework.a（已编译）
- ele_motor_drive.h/cpp（已实现）

---

## 注意事项

1. **不需要实际CAN通信**: 只需计算和显示CAN指令
2. **参数自动截断**: 使用MotorParamValidator自动将参数截断到允许范围
3. **防御性编程**: 所有参数输入都应该验证
4. **清晰的UI布局**: 使用QGroupBox分组不同功能
5. **实时反馈**: 参数改变时立即显示计算结果

---

## 完成标准

- [ ] 所有文件创建完成
- [ ] 代码编译无错误
- [ ] GUI界面美观易用
- [ ] 所有功能正常工作
- [ ] CAN指令计算正确
- [ ] 参数验证有效
