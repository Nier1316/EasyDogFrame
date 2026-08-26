#pragma once
// 诊断/只读示例 24, 26~29, 33, 34（固件参数 / 键盘 / CANET 探针 / 频率 / IMU / 轮方向）
void Example24_ReadMotorParams();
void Example26_KeyboardInputTest();
void Example27_CANetFrequencyProbe();
void Example28_CANetBatchProbe();
void Example29_MainLoopCadenceTest();
void Example33_IMUCheck();
void Example34_WheelDirectionCheck();
// USB2CAN 传输链路验证（走 CanTransport 接口，不依赖 CANET）
void Example39_Usb2CanProbe();
// USB2CAN 读 CAN1 电机姿态（走完整 MotorManager 链路，使能+保持+读状态）
void Example40_Usb2CanReadStatus();
// CAN1 三关节 500Hz 插值回站立姿态（测 USB2CAN 高频控制）
void Example41_CAN1_500HzStand();
// USB2CAN 控制频率测试（单帧耗时 + 目标频率可达性）
void Example42_Usb2CanRateTest();
// 4 路 USB2CAN CAN 顺序标定（摆腿检测腿→CAN 映射）
void Example43_CANOrderCalibrate();
// Xbox 手柄控制（USB2CAN 4 路版，复刻 Example21）
void Example44_USB2CanXboxControl();
// 选择电机移动到物理零位（USB2CAN 4 路版，复刻 Example20）
void Example45_USB2CanMoveToZero();
