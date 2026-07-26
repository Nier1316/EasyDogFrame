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

# C++ 标准：使用 C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============ 输出目录 ============
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ============ 头文件搜索路径 ============
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/motor_drive
    ${PROJECT_SOURCE_DIR}/include/math
    ${PROJECT_SOURCE_DIR}/include/bsp
    ${PROJECT_SOURCE_DIR}/lib
    ${PROJECT_SOURCE_DIR}/src/base
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

# ============ 源文件列表 ============
set(SOURCES
    # 主程序
    src/main.cpp

    # 示例程序
    src/example.cpp

    # CAN 设备层
    src/can_device.cpp

    # BSP 层
    src/bsp/bsp_can.cpp

    # 线程管理层
    src/thread/thread_manager.cpp

    # 顶层应用
    src/RoboTasks/robot_app.cpp

    # 电机管理层
    src/motor_manager.cpp

    # 电机驱动层
    src/motor_drive/ele_motor.cpp

    # 基础库
    src/base/common.cpp
    src/base/crc16.cpp
    src/base/log.cpp
    src/base/jsoncpp/jsoncpp.cpp
    src/base/md5/aw_md5.cpp
    src/base/platform/linux/network.cpp
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
