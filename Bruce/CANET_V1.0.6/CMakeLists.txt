# =====================================================================
#   CAN 电机控制框架 —— 顶层 CMake 构建脚本
# ---------------------------------------------------------------------
#   编译步骤：
#     mkdir build && cd build
#     cmake -DCMAKE_BUILD_TYPE=Release ..
#     make -j$(nproc)
#   产物：
#     bin/motor_app             —— 示例可执行文件
#     lib/libmotor_framework.a  —— 框架静态库（供外部工程复用）
# =====================================================================
cmake_minimum_required(VERSION 3.8)

project(MotorControlFramework)

# 采用 C++11：框架用到的 std::thread / unique_ptr / lambda 等均需 ≥11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------- 头文件搜索路径 ----------------
# include/        框架自身的头文件
# src/CANET       CANET 库的 ControlCAN.h 等公开头
# src/base        CANET 所需的基础头（log、common 等）
# lib             CANET.h 所在位置
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/CANET
    ${PROJECT_SOURCE_DIR}/src/base
    ${PROJECT_SOURCE_DIR}/lib
)

# 线程库：Linux 下对应 -lpthread
find_package(Threads REQUIRED)

# ---------------- 框架源文件 ----------------
# 按分层顺序排列：底层驱动 → 管理层 → 应用门面
set(FRAMEWORK_SOURCES
    src/motor.cpp              # 电机驱动层
    src/can_device.cpp         # CAN 设备层
    src/motor_manager.cpp      # 设备/电机统一管理
    src/motor_controller.cpp   # 应用层门面 API
)

# ---------------- 编译产物 ----------------
# 1) motor_framework：框架静态库（可被其它项目链接复用）
add_library(motor_framework STATIC ${FRAMEWORK_SOURCES})
target_link_libraries(motor_framework PUBLIC Threads::Threads)

# 2) motor_app：示例可执行文件
add_executable(motor_app src/main.cpp)
target_link_libraries(motor_app PRIVATE motor_framework)

# ---------------- 链接 CANET 库 ----------------
# 按构建类型选择对应版本的预编译静态库（Debug / Release 二选一）
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(motor_app PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/linux_x64/Debug/libCANET_TCP.a
    )
else()
    target_link_libraries(motor_app PRIVATE
        ${PROJECT_SOURCE_DIR}/lib/linux_x64/Release/libCANET_TCP.a
    )
endif()

# pthread：CANET 内部和本框架都使用了 std::thread
target_link_libraries(motor_app PRIVATE Threads::Threads)

# ---------------- 输出目录 ----------------
# 可执行文件 → bin/，静态库/归档 → lib/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ---------------- 编译警告（仅 UNIX） ----------------
# -Wall -Wextra -Wpedantic：打开常见的静态检查，尽早发现隐患
if(UNIX)
    target_compile_options(motor_framework PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(motor_app       PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------- 编译信息打印 ----------------
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Framework sources: ${FRAMEWORK_SOURCES}")
