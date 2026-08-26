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
        // 2ms 间隔：匹配 CONTROL_HZ=500 的控制循环。USB2CAN 回调队列 2ms 内取到最新反馈；
        // CANET 下 ReceiveOnce 内 recv(timeout=10) 会阻塞到 ~10ms（VCI_Receive 节拍），无副作用。
        ThreadMode::LOOP, 2, 80
    );
    thread_mgr.register_thread(
        "motor_send",
        [&mm]() { mm.SendOnce(); },
        ThreadMode::LOOP, 1, 80    // 1ms 间隔：发送路径实测 ~0.01ms，可真正达到 1ms
    );
}
