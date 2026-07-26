# 独立的电机 CAN 帧编解码 GUI 工具 —— 与主工程无关，单独构建
cmake_minimum_required(VERSION 3.16)
project(can_frame_tool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)   # 处理 Q_OBJECT / main.moc

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(can_frame_tool main.cpp)
target_link_libraries(can_frame_tool PRIVATE Qt5::Widgets)
