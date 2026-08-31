/**
 * @file    motor_io.h
 * @brief   电机收发线程注册（L6 运行时）
 * @details MotorManager 只提供单次轮询（ReceiveOnce/SendOnce），本模块负责把
 *          它们注册到 ThreadManager 并定义节拍/优先级。这样线程编排属于运行时层，
 *          MotorManager 不拥有线程生命周期（分层清晰）。
 */
#ifndef MOTOR_IO_H_
#define MOTOR_IO_H_

#include "runtime/thread_manager.h"

class MotorManager;

/**
 * @brief 注册 motor_receive / motor_send 两个 LOOP 线程
 * @param thread_mgr 统一线程管理器（示例持有的 ThreadManager）
 * @param mm         MotorManager（调用其 ReceiveOnce/SendOnce）
 *
 * 线程名/节拍/优先级（当前实现）：
 *   motor_receive  LOOP 2ms  优先级 80  （对齐 CONTROL_HZ=500；USB2CAN 回调队列 2ms 取最新反馈）
 *   motor_send     LOOP 2ms  优先级 80  （500Hz；原 1ms 冗余，每 2ms 重复发相同帧）
 */
void RegisterMotorIoThreads(ThreadManager& thread_mgr, MotorManager& mm);

#endif // MOTOR_IO_H_
