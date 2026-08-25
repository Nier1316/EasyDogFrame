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
 * 线程名/节拍/优先级保持与原 MotorManager::Initialize 一致：
 *   motor_receive  LOOP 10ms  优先级 80  （对齐 SDK VCI_Receive ~10ms 节拍）
 *   motor_send     LOOP 1ms   优先级 80
 */
void RegisterMotorIoThreads(ThreadManager& thread_mgr, MotorManager& mm);

#endif // MOTOR_IO_H_
