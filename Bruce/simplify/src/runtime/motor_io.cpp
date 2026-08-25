/**
 * @file    motor_io.cpp
 * @brief   电机收发线程注册实现（见 motor_io.h）
 */
#include "runtime/motor_io.h"
#include "motor/motor_manager.h"

void RegisterMotorIoThreads(ThreadManager& thread_mgr, MotorManager& mm) {
    thread_mgr.register_thread(
        "motor_receive",
        [&mm]() { mm.ReceiveOnce(); },
        ThreadMode::LOOP, 10, 80   // 10ms 间隔：对齐 SDK 实际接收节拍（VCI_Receive 内部 ~10ms 轮询）
    );
    thread_mgr.register_thread(
        "motor_send",
        [&mm]() { mm.SendOnce(); },
        ThreadMode::LOOP, 1, 80    // 1ms 间隔：发送路径实测 ~0.01ms，可真正达到 1ms
    );
}
