#include <stdio.h>
#include <unistd.h>       // sleep()
#include <signal.h>       // signal() / SIGINT / SIGTERM
#include "bsp/bsp_can.h"   // 示例 1 直接使用 BSP 层
#include "motor_manager.h"
#include "thread/thread_manager.h"


void Example1_RawCanFrameTest();
void Example2_OnceMode();
void Example3_LoopMode();
void Example4_SharedData();
void Example5_DuplicateRegister();
void Example6_ThreadRestart();
void Example7_AutoCleanup();
void Example8_StateQuery();
void Example9_BasicMotorCtr();
void Example10_MultiMotorTorqueControl();
void Example11_MoveAll();
void Example12_SinusoidalMotion();
