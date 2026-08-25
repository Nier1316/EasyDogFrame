/**
 * @file  robot_app.cpp
 * @brief 顶层应用类实现
 */
#include "runtime/robot_app.h"
#include "motor/motor_manager.h"
#include <cstdio>

RobotApp::~RobotApp() {
    printf("[DEBUG] RobotApp destructor called\n");
    fflush(stdout);
    try {
        stop();
    } catch (const std::exception& e) {
        printf("[ERROR] Exception in RobotApp destructor: %s\n", e.what());
        fflush(stdout);
    }
    printf("[DEBUG] RobotApp destructor finished\n");
    fflush(stdout);
}

bool RobotApp::init() {
    // 各模块向 thread_mgr_ 注册任务函数（不启动线程）
    if (!MotorManager::GetInstance().Initialize(thread_mgr_)) {
        printf("[ERROR] MotorManager initialization failed\n");
        return false;
    }

    // 预留：其他模块注册
    // thread_mgr_.register_thread("state_calc", [this]() { ... }, ThreadMode::LOOP, 5, 50);
    // thread_mgr_.register_thread("monitor",    [this]() { ... }, ThreadMode::LOOP, 100, 0);

    printf("[INFO] RobotApp initialized\n");
    return true;
}

void RobotApp::start() {
    // 按优先级从高到低启动，确保高优先级线程先就绪
    thread_mgr_.start_thread("motor_receive");
    thread_mgr_.start_thread("motor_send");
    // thread_mgr_.start_thread("state_calc");
    // thread_mgr_.start_thread("monitor");

    printf("[INFO] RobotApp started\n");
}

void RobotApp::stop() {
    // 示例大多用独立的局部 ThreadManager，RobotApp 的 thread_mgr_ 里通常
    // 没有注册 motor_receive/motor_send。直接 stop_thread 会对未注册线程
    // 抛 std::runtime_error，在析构里只剩一行难看的异常输出。
    // 先查注册状态再停，未注册的线程直接跳过，析构阶段不再抛异常。
    if (thread_mgr_.get_thread_state("motor_receive") != ThreadState::UNREGISTERED)
        thread_mgr_.stop_thread("motor_receive");
    if (thread_mgr_.get_thread_state("motor_send") != ThreadState::UNREGISTERED)
        thread_mgr_.stop_thread("motor_send");
    // thread_mgr_.stop_thread("state_calc");
    // thread_mgr_.stop_thread("monitor");

    MotorManager::GetInstance().Stop();
    printf("[INFO] RobotApp stopped\n");
}

SharedData& RobotApp::shared_data() {
    return thread_mgr_.get_shared_data();
}
