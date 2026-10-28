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
    # L0.5 公共工具（RL 遥测记录器）
    src/common/s2r_recorder.cpp
    src/common/s2r_dataset.cpp

    # L7 应用
    src/app/main.cpp
    # 示例（按功能拆分，公共 helper）
    src/app/examples/examples_common.cpp
    src/app/examples/ex_basic.cpp
    src/app/examples/ex_diag.cpp
    src/app/examples/ex_rl.cpp
    src/app/examples/ex_sysid.cpp
    src/app/examples/ex_probe.cpp

    # L6 运行时（线程/顶层应用/电机IO线程）
    src/runtime/thread_manager.cpp
    src/runtime/robot_app.cpp
    src/runtime/motor_io.cpp

    # L5 策略/感知（RL/IMU/sim2real 标定）
    src/strategy/rl_controller.cpp
    src/strategy/sim2real_conv.cpp
    src/strategy/imu_device.cpp
    src/strategy/policy_set.cpp      # 两套策略权重同时编译 + 运行时选择

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
target_link_libraries(can_motor_app PRIVATE Threads::Threads ${CMAKE_DL_LIBS})   # libdl: 自检垫片

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

# ============ 达妙 SDK 的 libusb 事件收割超时修正（LD_PRELOAD 垫片）============
# 背景：libdm_device.so 的 can_bus_box::usb_handle_thread_func() 把
#   libusb_handle_events_timeout_completed() 的等待超时写死成 100 ms
#   （.rodata@0x151d8 = timeval{0,100000}，**全库仅此一处 100000**）⇒ CAN 回帧被攒成
#   ~100 ms 一批，上位机看到的位置反馈只有 ~10 Hz。详见 tool/libusb_fastpoll.c 的文件头。
# 本目标产出 ./bin/libusb_fastpoll.so，由 run.sh 用 LD_PRELOAD 加载；不加载 = 原行为。
add_library(usb_fastpoll SHARED tool/libusb_fastpoll.c)
set_target_properties(usb_fastpoll PROPERTIES
    # 顶层设了 CMAKE_LIBRARY_OUTPUT_DIRECTORY=lib/（那是入库的 SDK 目录），
    # 垫片是构建产物，改放 bin/（已在 .gitignore 里）。
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
)
target_link_libraries(usb_fastpoll PRIVATE ${CMAKE_DL_LIBS})
if(UNIX)
    target_compile_options(usb_fastpoll PRIVATE -Wall -Wextra -Wpedantic)
endif()
message(STATUS "libusb 垫片: ${PROJECT_SOURCE_DIR}/bin/libusb_fastpoll.so")

# ---------------------------------------------------------------------------
# libusb 传输探针垫片：打印 SDK 请求的 bulk IN transfer 长度 + 完成间隔直方图，
# 用于定位 "CAN 回帧被攒成 ~100 ms 一批" 到底在传输层还是设备固件。
# 详见 tool/libusb_xferprobe.c 的文件头。可选 DMCAN_XFER_BYTES=<n> 夹小请求长度（验证性修复）。
# ---------------------------------------------------------------------------
add_library(usb_xferprobe SHARED tool/libusb_xferprobe.c)
set_target_properties(usb_xferprobe PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
)
# libusb 头从 conda 前缀推（与链接同一个 libusb，避免拿到系统 1.0.25 的头）
get_filename_component(CONDA_PREFIX_DIR ${CONDA_LIB_DIR} DIRECTORY)
target_include_directories(usb_xferprobe PRIVATE ${CONDA_PREFIX_DIR}/include/libusb-1.0)
target_link_libraries(usb_xferprobe PRIVATE ${CMAKE_DL_LIBS})
if(UNIX)
    target_compile_options(usb_xferprobe PRIVATE -Wall -Wextra -Wpedantic)
endif()
message(STATUS "libusb 传输探针: ${PROJECT_SOURCE_DIR}/bin/libusb_xferprobe.so")

# ---------------------------------------------------------------------------
# 策略权重离线数值回归（**不碰 CAN/USB**）：对全部变体跑 mlp_forward(REF_OBS) 并与
# REF_ACTION 比对。单独一个可执行文件，避免为了验证权重去启动真机程序。
# 用法：./bin/policy_selftest   （退出码 0=通过）
# ---------------------------------------------------------------------------
add_executable(policy_selftest tool/policy_selftest.cc src/strategy/policy_set.cpp
                                 src/strategy/rl_controller.cpp
                                 src/strategy/sim2real_conv.cpp)   # 观测通道自检需要 build_observation + DEFAULT_POSE（纯 CPU）
target_include_directories(policy_selftest PRIVATE ${PROJECT_SOURCE_DIR}/include)
if(UNIX)
    target_compile_options(policy_selftest PRIVATE -Wall -Wextra)
endif()
message(STATUS "策略离线回归: ${PROJECT_SOURCE_DIR}/bin/policy_selftest")

# ---------------------------------------------------------------------------
# 达妙 SDK 入站派发提速垫片：notification_thread_func 里硬编码 nanosleep(100 ms)
#（.rodata@0x151e0 = timespec{0,100000000}）⇒ 上位机 ~10 Hz 攒批。
# 本垫片只夹"libdm_device.so 内、请求 >20 ms"的 nanosleep（心跳的 10 ms 与我们自己的
# sleep 都不受影响）。详见 tool/dmcan_fastdispatch.c。
# ---------------------------------------------------------------------------
add_library(dmcan_fastdispatch SHARED tool/dmcan_fastdispatch.c)
set_target_properties(dmcan_fastdispatch PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
    PREFIX ""                      # 产物名要正好是 dmcan_fastdispatch.so（不带 lib 前缀）
)
target_link_libraries(dmcan_fastdispatch PRIVATE ${CMAKE_DL_LIBS})
if(UNIX)
    target_compile_options(dmcan_fastdispatch PRIVATE -Wall -Wextra)
endif()
message(STATUS "派发提速垫片: ${PROJECT_SOURCE_DIR}/bin/dmcan_fastdispatch.so")

# ============ 编译信息 ============
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
