#include <stdio.h>
#include <unistd.h>       // sleep()
#include <signal.h>       // signal() / SIGINT / SIGTERM
#include "bsp/bsp_can.h"   // 示例 1 直接使用 BSP 层



void Example1_RawCanFrameTest();
void Example2_OnceMode();
void Example3_LoopMode();
void Example4_SharedData();