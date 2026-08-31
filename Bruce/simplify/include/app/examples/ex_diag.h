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
// USB2CAN 单电机阶跃响应测试（区分批量发送丢帧 vs 单发链路问题）
void Example46_USB2CanSingleMotorStep();
void Example47_ChirpSysId();
void Example48_WheelDirectionVerify();
void Example49_StandAndWheelSpeedLoopTest();
// 趴下姿态实际角度记录（起立→趴下→记录稳态角，用于标定 LIE_DOWN_*_DEG）
void Example50_LieDownAngleRecord();
// 吊装摩擦辨识（重力标定 + 前馈恒速 + 双向配对 → 离线回归出 LEG_FF_FC/FV）
void Example54_FrictionSysId();
