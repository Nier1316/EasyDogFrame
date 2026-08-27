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
        // 2ms 间隔：对齐 CONTROL_HZ=500。原 1ms 是冗余（每 2ms 重复发相同帧），
        // 且 1kHz×16 电机=16k 帧/s 是高负载，是 USB2CAN 发送压力/接收不稳的来源之一。
        // 轮子上位机闭环也在 SendOnce（500Hz），2ms 周期正合适。
        ThreadMode::LOOP, 2, 80
    );
}
