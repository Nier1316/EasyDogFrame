# motor_control_gui.cpp 中使用的函数清单

## 📌 来自 ele_motor_drive 的函数调用

### MotorParamValidator 类（参数验证）

用于验证和截断参数到允许范围内的函数：

| 函数名 | 功能 | 调用位置 | 参数范围 |
|--------|------|---------|---------|
| `ValidatePosition()` | 验证位置参数 | 阻抗模式、位置模式 | ±12.5 rad |
| `ValidateVelocity()` | 验证速度参数 | 阻抗模式、速度模式 | ±14.0 rad/s |
| `ValidateKp()` | 验证 Kp 增益 | 所有模式 | 0~500 |
| `ValidateKd()` | 验证 Kd 阻尼 | 阻抗模式、位置模式 | 0~100 |
| `ValidateKi()` | 验证 Ki 积分增益 | 速度模式、位置模式 | 0~10000 |
| `ValidateTorque()` | 验证扭矩参数 | 阻抗模式 | ±10.0 Nm |

**调用统计**：
- `ValidatePosition()` - 2 次（阻抗、位置模式）
- `ValidateVelocity()` - 2 次（阻抗、速度模式）
- `ValidateKp()` - 4 次（阻抗、速度、位置模式各1-2次）
- `ValidateKd()` - 2 次（阻抗、位置模式）
- `ValidateKi()` - 2 次（速度、位置模式）
- `ValidateTorque()` - 1 次（阻抗模式）

---

### MotorParamCodec 类（参数编解码）

用于参数量化和 CAN 帧打包的函数：

| 函数名 | 功能 | 调用位置 | 说明 |
|--------|------|---------|------|
| `FloatToUint()` | Float 转无符号整数 | 所有控制模式 | 量化浮点数为整数 |
| `Float2Bag()` | 参数打包成 CAN 帧 | 参数读写 | 生成参数指令帧 |

**调用统计**：
- `FloatToUint()` - 7 次
  - 阻抗模式：5 次（pos、vel、kp、kd、torque）
  - 速度模式：3 次（vel、kp、ki）
  - 位置模式：5 次（pos、kvp、kp、kd、kvi）
- `Float2Bag()` - 2 次
  - 读参数：1 次
  - 写参数：1 次

---

## 📊 函数调用详细分析

### 1. 阻抗模式计算 (`onCalculateImpedanceCommand()`)

```cpp
// 参数验证
float pos = MotorParamValidator::ValidatePosition(impedancePosSpinBox->value());
float vel = MotorParamValidator::ValidateVelocity(impedanceVelSpinBox->value());
float kp = MotorParamValidator::ValidateKp(impedanceKpSpinBox->value());
float kd = MotorParamValidator::ValidateKd(impedanceKdSpinBox->value());
float torque = MotorParamValidator::ValidateTorque(impedanceTorqueSpinBox->value());

// 参数量化
unsigned int posInt = MotorParamCodec::FloatToUint(pos, P_MIN, P_MAX, 15);
unsigned int velInt = MotorParamCodec::FloatToUint(vel, V_MIN, V_MAX, 12);
unsigned int kpInt = MotorParamCodec::FloatToUint(kp, KP_MIN, KP_MAX, 9);
unsigned int kdInt = MotorParamCodec::FloatToUint(kd, KD_MIN, KD_MAX, 8);
unsigned int torqueInt = MotorParamCodec::FloatToUint(torque, T_MIN, T_MAX, 12);
```

**调用函数**：
- ✅ `MotorParamValidator::ValidatePosition()`
- ✅ `MotorParamValidator::ValidateVelocity()`
- ✅ `MotorParamValidator::ValidateKp()`
- ✅ `MotorParamValidator::ValidateKd()`
- ✅ `MotorParamValidator::ValidateTorque()`
- ✅ `MotorParamCodec::FloatToUint()` × 5

---

### 2. 速度模式计算 (`onCalculateSpeedCommand()`)

```cpp
// 参数验证
float vel = MotorParamValidator::ValidateVelocity(speedVelSpinBox->value());
float kp = MotorParamValidator::ValidateKp(speedKpSpinBox->value());
float ki = MotorParamValidator::ValidateKi(speedKiSpinBox->value());

// 参数量化
unsigned int velInt = MotorParamCodec::FloatToUint(vel, V_MIN, V_MAX, 12);
unsigned int kpInt = MotorParamCodec::FloatToUint(kp, KP_MIN, KP_MAX, 9);
unsigned int kiInt = MotorParamCodec::FloatToUint(ki, KI_MIN, KI_MAX, 14);
```

**调用函数**：
- ✅ `MotorParamValidator::ValidateVelocity()`
- ✅ `MotorParamValidator::ValidateKp()`
- ✅ `MotorParamValidator::ValidateKi()`
- ✅ `MotorParamCodec::FloatToUint()` × 3

---

### 3. 位置模式计算 (`onCalculatePositionCommand()`)

```cpp
// 参数验证
float pos = MotorParamValidator::ValidatePosition(positionPosSpinBox->value());
float kvp = MotorParamValidator::ValidateKp(positionKvpSpinBox->value());
float kp = MotorParamValidator::ValidateKp(positionKpSpinBox->value());
float kd = MotorParamValidator::ValidateKd(positionKdSpinBox->value());
float kvi = MotorParamValidator::ValidateKi(positionKviSpinBox->value());

// 参数量化
unsigned int posInt = MotorParamCodec::FloatToUint(pos, P_MIN, P_MAX, 15);
unsigned int kvpInt = MotorParamCodec::FloatToUint(kvp, KP_MIN, KP_MAX, 9);
unsigned int kpInt = MotorParamCodec::FloatToUint(kp, KP_MIN, KP_MAX, 9);
unsigned int kdInt = MotorParamCodec::FloatToUint(kd, KD_MIN, KD_MAX, 8);
unsigned int kviInt = MotorParamCodec::FloatToUint(kvi, KI_MIN, KI_MAX, 14);
```

**调用函数**：
- ✅ `MotorParamValidator::ValidatePosition()`
- ✅ `MotorParamValidator::ValidateKp()` × 2
- ✅ `MotorParamValidator::ValidateKd()`
- ✅ `MotorParamValidator::ValidateKi()`
- ✅ `MotorParamCodec::FloatToUint()` × 5

---

### 4. 参数读取 (`onReadParameter()`)

```cpp
uint8_t paramType = paramTypeCombo->currentData().toUInt();
uint8_t data[8];
MotorParamCodec::Float2Bag(0.0f, 0, paramType, currentMotorId, data);
```

**调用函数**：
- ✅ `MotorParamCodec::Float2Bag()` - 读参数（RW=0）

---

### 5. 参数写入 (`onWriteParameter()`)

```cpp
uint8_t paramType = paramTypeCombo->currentData().toUInt();
float value = paramValueSpinBox->value();
uint8_t data[8];
MotorParamCodec::Float2Bag(value, 1, paramType, currentMotorId, data);
```

**调用函数**：
- ✅ `MotorParamCodec::Float2Bag()` - 写参数（RW=1）

---

## 📈 函数调用统计总表

| 函数 | 调用次数 | 模式 |
|------|---------|------|
| `MotorParamValidator::ValidatePosition()` | 2 | 阻抗、位置 |
| `MotorParamValidator::ValidateVelocity()` | 2 | 阻抗、速度 |
| `MotorParamValidator::ValidateKp()` | 4 | 所有模式 |
| `MotorParamValidator::ValidateKd()` | 2 | 阻抗、位置 |
| `MotorParamValidator::ValidateKi()` | 2 | 速度、位置 |
| `MotorParamValidator::ValidateTorque()` | 1 | 阻抗 |
| `MotorParamCodec::FloatToUint()` | 13 | 所有模式 |
| `MotorParamCodec::Float2Bag()` | 2 | 参数读写 |
| **总计** | **28** | - |

---

## ✅ 证明上位机使用了 ele_motor_drive 接口

### 1. **源代码包含**
```cpp
#include "ele_motor_drive.h"  // ✅ 直接包含头文件
```

### 2. **编译配置链接**
```makefile
LIBS += -L../lib -lmotor_framework  # ✅ 链接静态库
INCLUDEPATH += ../include            # ✅ 包含路径
```

### 3. **函数调用证据**
- 共调用 **8 个不同的函数**
- 共调用 **28 次**
- 涵盖所有三种控制模式
- 涵盖参数读写功能

### 4. **参数验证流程**
```
用户输入 → MotorParamValidator验证 → MotorParamCodec量化 → CAN帧生成
```

### 5. **没有重复实现**
- ❌ 源代码中**没有**自己实现的参数验证逻辑
- ❌ 源代码中**没有**自己实现的量化算法
- ✅ 所有计算都通过调用 ele_motor_drive 的接口完成

---

## 🎯 结论

**motor_control_gui.cpp 完全依赖 ele_motor_drive 中的接口进行计算**，具体表现为：

1. ✅ 直接包含 `ele_motor_drive.h` 头文件
2. ✅ 链接 `libmotor_framework.a` 静态库
3. ✅ 调用 `MotorParamValidator` 进行参数验证
4. ✅ 调用 `MotorParamCodec` 进行参数编解码
5. ✅ 没有任何自己实现的计算逻辑

**这证明了上位机是通过调用 ele_motor_drive 中的接口进行计算，而不是另外实现计算逻辑。**
