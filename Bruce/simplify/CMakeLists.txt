# =====================================================================
#   Simplify CAN 电机控制框架 —— 独立编译配置
# =====================================================================
#   编译步骤：
#     mkdir build && cd build
#     cmake -DCMAKE_BUILD_TYPE=Release ..
#     make -j$(nproc)
#   产物：
#     bin/can_motor_app  —— 可执行文件
# =====================================================================

cmake_minimum_required(VERSION 3.8)
project(SimplifyCAN)

# C++ 标准：使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 导出 compile_commands.json，供 clangd / cpptools 解析头文件路径
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============ 输出目录 ============
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ============ 头文件搜索路径 ============
# 分层目录结构：include/<layer>/xxx.h，以 include/ 为根，用 "layer/xxx.h" 引用
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/lib
)

# ============ 查找依赖 ============
find_package(Threads REQUIRED)

# SDL2 — 手柄控制 (Example21)
find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(SDL2 sdl2)
    endif()
endif()

# ============ 源文件列表（按分层目录） ============
set(SOURCES
    # L7 应用
    src/app/main.cpp
    # 示例（按功能拆分，公共 helper）
    src/app/examples/examples_common.cpp
    src/app/examples/ex_basic.cpp
    src/app/examples/ex_diag.cpp
    src/app/examples/ex_rl.cpp

    # L6 运行时（线程/顶层应用/电机IO线程）
    src/runtime/thread_manager.cpp
    src/runtime/robot_app.cpp
    src/runtime/motor_io.cpp

    # L5 策略/感知（RL/IMU/sim2real 标定）
    src/strategy/rl_controller.cpp
    src/strategy/sim2real_conv.cpp
    src/strategy/imu_device.cpp

    # L4 运控
    src/motion/motion_controller.cpp

    # L3 电机管理
    src/motor/motor_manager.cpp

    # L2 电机驱动
    src/motor/ele_motor.cpp

    # L1 传输层（CANET / USB2CAN）
    src/transport/canet_transport.cpp
    src/transport/can_device.cpp
    src/transport/usb2can_transport.cpp
)

# ============ 编译可执行文件 ============
add_executable(can_motor_app ${SOURCES})

# ============ 链接库 ============
target_link_libraries(can_motor_app PRIVATE Threads::Threads)

# 条件链接 SDL2
if(SDL2_FOUND OR SDL2_LIBRARIES)
    target_include_directories(can_motor_app PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(can_motor_app PRIVATE ${SDL2_LIBRARIES})
    message(STATUS "SDL2 found: ${SDL2_LIBRARIES}")
else()
    message(STATUS "SDL2 not found — Example21 will not be available")
endif()

# 链接 CANET 库（根据编译类型选择 Debug 或 Release）
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(can_motor_app PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/linux_x64/Debug/libCANET_TCP.a
    )
else()
    target_link_libraries(can_motor_app PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/linux_x64/Release/libCANET_TCP.a
    )
endif()

# ============ 达妙 USB2CAN 官方 SDK ============
# libdm_device.so 依赖新版 libusb(≥1.0.26) + libstdc++(≥3.4.32)，系统 gcc11 太旧，
# 经 conda miniforge 库绕开。运行同样需要这些库，用 DT_RPATH（--disable-new-dtags，
# 递归生效）指向 damiao_sdk 与 miniforge3/lib。
set(DAMIAO_SDK_DIR ${PROJECT_SOURCE_DIR}/lib/damiao_sdk/linux/x86_64)
# 自动探测本机 conda lib（libstdc++ >= GLIBCXX_3.4.32 + libusb >= 1.0.26，达妙依赖）。
# 可用 -DCONDA_LIB_DIR=... 覆盖；bruce 机 /home/bruce/miniforge3，sysu 机 /home/sysu/miniconda3。
set(CONDA_LIB_DIR "" CACHE STRING "conda lib dir for damiao deps (libstdc++>=3.4.32 + libusb>=1.0.26)")
if(NOT CONDA_LIB_DIR)
  if(EXISTS "/home/bruce/miniforge3/lib/libusb-1.0.so")
    set(CONDA_LIB_DIR "/home/bruce/miniforge3/lib")
  elseif(EXISTS "/home/sysu/miniconda3/lib/libusb-1.0.so")
    set(CONDA_LIB_DIR "/home/sysu/miniconda3/lib")
  endif()
endif()
if(NOT CONDA_LIB_DIR)
  message(FATAL_ERROR "找不到含 libusb-1.0.so 的 conda lib 目录，请用 -DCONDA_LIB_DIR=<path> 指定")
endif()
message(STATUS "CONDA_LIB_DIR = ${CONDA_LIB_DIR}")
target_link_libraries(can_motor_app PRIVATE
    ${DAMIAO_SDK_DIR}/libdm_device.so
    # libusb 用绝对路径，避免 -L 顺序解析到系统老 libusb(1.0.25, 无 libusb_init_context)
    ${CONDA_LIB_DIR}/libusb-1.0.so
    # -L 保留给 libstdc++（系统 gcc11 只到 GLIBCXX_3.4.30，conda 有 3.4.32）
    -L${CONDA_LIB_DIR}
)
target_link_options(can_motor_app PRIVATE
    -Wl,--disable-new-dtags
    -Wl,-rpath,${DAMIAO_SDK_DIR}
    -Wl,-rpath,${CONDA_LIB_DIR}
)

# ============ 编译选项 ============
if(UNIX)
    target_compile_options(can_motor_app PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ============ AddressSanitizer（诊断用，默认关闭）============
# 开启方式: cmake -B build -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
option(ENABLE_ASAN "Enable AddressSanitizer for heap corruption diagnosis" OFF)
if(ENABLE_ASAN)
    message(STATUS "AddressSanitizer: ENABLED")
    target_compile_options(can_motor_app PRIVATE -fsanitize=address -fno-omit-frame-pointer -g)
    target_link_options(can_motor_app PRIVATE -fsanitize=address)
endif()

# ============ 编译信息 ============
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
