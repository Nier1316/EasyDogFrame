/**
 * @file  robot_app.h
 * @brief 顶层应用类，统一管理所有线程的生命周期
 *
 * 所有线程在此处注册和启动，各模块（MotorManager 等）只提供任务函数，
 * 不自己持有线程。线程注册表：
 *
 *   motor_receive  LOOP 1ms   优先级80  CAN 数据接收
 *   motor_send     LOOP 1ms   优先级80  控制指令发送（预留）
 *   state_calc     LOOP 5ms   优先级50  数据解算（预留）
 *   monitor        LOOP 100ms 优先级0   状态监控（预留）
 */
#ifndef ROBOT_APP_H_
#define ROBOT_APP_H_

#include "thread/thread_manager.h"

class RobotApp {
public:
    RobotApp() = default;
    ~RobotApp();

    /**
     * @brief 初始化所有模块，向 thread_mgr_ 注册所有线程（不启动）
     * @return false 表示初始化失败（如 CAN 设备连接失败）
     */
    bool init();

    /**
     * @brief 按优先级顺序启动所有已注册线程
     */
    void start();

    /**
     * @brief 停止所有线程，关闭设备
     */
    void stop();

    /**
     * @brief 获取共享数据区，供外部读写跨线程数据
     */
    SharedData& shared_data();

private:
    ThreadManager thread_mgr_;  // 唯一 ThreadManager 实例，管理所有线程
};

#endif // ROBOT_APP_H_
