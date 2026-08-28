#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "runtime/robot_app.h"
#include "app/examples.h"

static RobotApp g_app;
static volatile bool g_running = true;

static void signal_handler(int) {
    g_running = false;
}

// =====================================================================
//  示例切换说明
//  = 只运行一个示例：目标示例的 3 行取消注释、其余保持注释即可。
//  = 当前启用：Example44（USB2CAN 手柄控制）。
//  =====================================================================
int main() {
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    // 运行示例9 - 基础电机控制
    // printf("[INFO] Running Example9_BasicMotorCtr...\n");
    // Example9_BasicMotorCtr();
    // printf("[INFO] Example9 completed.\n");

    // 运行示例10 - 多电机扭矩控制
    // printf("[INFO] Running Example10_MultiMotorTorqueControl...\n");
    // Example10_MultiMotorTorqueControl();
    // printf("[INFO] Example10 completed.\n");

    // 运行示例11 - 全电机扭矩控制（CAN0~3）
    // printf("[INFO] Running Example11_MoveAll...\n");
    // Example11_MoveAll();
    // printf("[INFO] Example11 completed.\n");

    // 运行示例12 - 正弦周期运动控制
    // printf("[INFO] Running Example12_SinusoidalMotion...\n");
    // Example12_SinusoidalMotion();
    // printf("[INFO] Example12 completed.\n");

    // 运行示例13 - 多电机正弦运动
    // printf("[INFO] Running Example13_MultiMotorSinusoidalMotion...\n");
    // Example13_MultiMotorSinusoidalMotion();
    // printf("[INFO] Example13 completed.\n");

    // 运行示例14 - 标定检测
    // printf("[INFO] Running Example14_CalibrationDetect...\n");
    // Example14_CalibrationDetect();
    // printf("[INFO] Example14 completed.\n");

    // 运行示例15 - 标定验证
    // printf("[INFO] Running Example15_CalibrationVerify...\n");
    // Example15_CalibrationVerify();
    // printf("[INFO] Example15 completed.\n");

    // 运行示例16 - 电机测试
    // printf("[INFO] Running Example16_MotorTest...\n");
    // Example16_MotorTest();
    // printf("[INFO] Example16 completed.\n");

    // 运行示例17 - SimSync 仿真集成
    // printf("[INFO] Running Example17_SimSyncIntegration...\n");
    // Example17_SimSyncIntegration();
    // printf("[INFO] Example17 completed.\n");

    // 运行示例18 - LEG IK Control
    // printf("[INFO] Running Example18_LegIKControl...\n");
    // Example18_LegIKControl();
    // printf("[INFO] Example18 completed.\n");

    // 运行示例19 - 读取当前姿态并缓慢移动到站立
    // printf("[INFO] Running Example19_ReadAndStand...\n");
    // Example19_ReadAndStand();
    // printf("[INFO] Example19 completed.\n");

    // 运行示例20 - 选择电机移动到物理零位
    // printf("[INFO] Running Example20_MoveToPhysicalZero...\n");
    // Example20_MoveToPhysicalZero();
    // printf("[INFO] Example20 completed.\n");

    // 运行示例21 - Xbox 手柄控制（普通 PD 站立，对照底层扭矩）
    // printf("[INFO] Running Example21_XboxControllerControl...\n");
    // Example21_XboxControllerControl();
    // printf("[INFO] Example21 completed.\n");

    // 运行示例22 - 起立 + 轮子阻抗模式测试
    // printf("[INFO] Running Example22_StandAndWheelTest...\n");
    // Example22_StandAndWheelTest();
    // printf("[INFO] Example22 completed.\n");

    // 运行示例23 - 单路 CAN 键盘控制（选路 + 站立 + ↑↓高度 + ←→轮子）
    // printf("[INFO] Running Example23_SingleCanKeyboardControl...\n");
    // Example23_SingleCanKeyboardControl();
    // printf("[INFO] Example23 completed.\n");

    // 运行示例24 - 只读固件参数诊断（不使能电机，安全）
    // printf("[INFO] Running Example24_ReadMotorParams...\n");
    // Example24_ReadMotorParams();
    // printf("[INFO] Example24 completed.\n");

    // 运行示例25 - RL 策略控制（dogurdf sim2real 部署）
    // printf("[INFO] Running Example25_RLPolicyControl...\n");
    // Example25_RLPolicyControl();
    // printf("[INFO] Example25 completed.\n");

    // 运行示例26 - 键盘输入接收测试（纯诊断，不碰电机）
    // printf("[INFO] Running Example26_KeyboardInputTest...\n");
    // Example26_KeyboardInputTest();
    // printf("[INFO] Example26 completed.\n");

    // 运行示例27 - CANET 接收频率探针（纯读取，不使能电机）
    // printf("[INFO] Running Example27_CANetFrequencyProbe...\n");
    // Example27_CANetFrequencyProbe();
    // printf("[INFO] Example27 completed.\n");

    // 运行示例28 - CANET 批量发送探针（电机下电，不使能电机）
    // printf("[INFO] Running Example28_CANetBatchProbe...\n");
    // Example28_CANetBatchProbe();
    // printf("[INFO] Example28 completed.\n");

    // 运行示例29 - 控制环频率测试（拆锁后，电机可下电）
    // printf("[INFO] Running Example29_MainLoopCadenceTest...\n");
    // Example29_MainLoopCadenceTest();
    // printf("[INFO] Example29 completed.\n");

    // 运行示例30 - RL 策略链路离线验证（不碰 CAN）——换权重后的回归门
    // printf("[INFO] Running Example30_RLPolicyLinkTest...\n");
    // Example30_RLPolicyLinkTest();
    // printf("[INFO] Example30 completed.\n");

    // 运行示例31 - RL 零位对齐（摆腿读角，不使能电机）
    // printf("[INFO] Running Example31_RLZeroAlign...\n");
    // Example31_RLZeroAlign();
    // printf("[INFO] Example31 completed.\n");

    // 运行示例32 - RL 默认姿态验证（命令到 DEFAULT_POSE，低增益）
    // printf("[INFO] Running Example32_RLPoseCheck...\n");
    // Example32_RLPoseCheck();
    // printf("[INFO] Example32 completed.\n");

    // 运行示例33 - IMU 链路验证（只读，不碰电机）
    // printf("[INFO] Running Example33_IMUCheck...\n");
    // Example33_IMUCheck();
    // printf("[INFO] Example33 completed.\n");

    // 运行示例34 - 轮子方向诊断（扭矩测向，起立悬空 ±1.0 Nm）
    // printf("[INFO] Running Example34_WheelDirectionCheck...\n");
    // Example34_WheelDirectionCheck();
    // printf("[INFO] Example34 completed.\n");

    // 运行示例35 - 轮电机前馈标定（起立悬空 + 键盘调扭矩）
    // printf("[INFO] Running Example35_WheelFFCalibrate...\n");
    // Example35_WheelFFCalibrate();
    // printf("[INFO] Example35 completed.\n");

    // 运行示例36 - RL 站立循环（USB2CAN 4 路，无手柄）
    // printf("[INFO] Running Example36_RLStandLoop...\n");
    // Example36_RLStandLoop();
    // printf("[INFO] Example36 completed.\n");

    // 运行示例47 - 悬空 chirp 扫频 + 最小二乘参数辨识（辨识 J/B/f_c/b → tau_ff + KP/KD）
    // printf("[INFO] Running Example47_ChirpSysId...\n");
    // Example47_ChirpSysId();
    // printf("[INFO] Example47 completed.\n");

    // 运行示例48 - 轮子扭矩方向安全验证（单轮开环测向，无 RL 失控风险）
    // printf("[INFO] Running Example48_WheelDirectionVerify...\n");
    // Example48_WheelDirectionVerify();
    // printf("[INFO] Example48 completed.\n");

    // 运行示例37 - RL 遥操作（手柄前进/后退 + 转向，USB2CAN 4 路）
    printf("[INFO] Running Example37_RLTeleopControl...\n");
    Example37_RLTeleopControl();
    // printf("[INFO] Example37 completed.\n");

    // 运行示例38 - 执行器延迟辨识（扫频测 Action Delay，定 action_delay_steps）

    // printf("[INFO] Running Example38_ActionDelayMeasure...\n");
    // Example38_ActionDelayMeasure();
    // printf("[INFO] Example38 completed.\n");

    // 运行示例39 - USB2CAN 传输链路验证（走 CanTransport 接口测达妙模块）
    // printf("[INFO] Running Example39_Usb2CanProbe...\n");
    // Example39_Usb2CanProbe();
    // printf("[INFO] Example39 completed.\n");

    // 运行示例40 - USB2CAN 读 CAN1 电机姿态（走完整 MotorManager 链路）
    // printf("[INFO] Running Example40_Usb2CanReadStatus...\n");
    // Example40_Usb2CanReadStatus();
    // printf("[INFO] Example40 completed.\n");

    // 运行示例41 - CAN1 三关节 500Hz 插值回站立
    // printf("[INFO] Running Example41_CAN1_500HzStand...\n");
    // Example41_CAN1_500HzStand();
    // printf("[INFO] Example41 completed.\n");

    // 运行示例42 - USB2CAN 控制频率测试
    // printf("[INFO] Running Example42_Usb2CanRateTest...\n");
    // Example42_Usb2CanRateTest();
    // printf("[INFO] Example42 completed.\n");

    // 运行示例43 - 4 路 USB2CAN CAN 顺序标定（摆腿检测）
    // printf("[INFO] Running Example43_CANOrderCalibrate...\n");
    // Example43_CANOrderCalibrate();
    // printf("[INFO] Example43 completed.\n");

    // 运行示例44 - Xbox 手柄控制（USB2CAN 4 路）
    // printf("[INFO] Running Example44_USB2CanXboxControl...\n");
    // Example44_USB2CanXboxControl();
    // printf("[INFO] Example44 completed.\n");

    // 运行示例45 - 选择电机移动到物理零位（USB2CAN）
    // printf("[INFO] Running Example45_USB2CanMoveToZero...\n");
    // Example45_USB2CanMoveToZero();
    // printf("[INFO] Example45 completed.\n");

    // 运行示例46 - USB2CAN 单电机阶跃响应测试（A/B 区分批量发送 vs 链路问题）
    // printf("[INFO] Running Example46_USB2CanSingleMotorStep...\n");
    // Example46_USB2CanSingleMotorStep();
    // printf("[INFO] Example46 completed.\n");

    return 0;
}
