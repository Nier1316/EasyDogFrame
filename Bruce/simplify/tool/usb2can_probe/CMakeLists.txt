# 达妙 USB2CAN 探针 —— 独立小工具，与主工程无关，单独构建
cmake_minimum_required(VERSION 3.16)
project(usb2can_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(usb2can_probe main.cpp)
target_link_libraries(usb2can_probe PRIVATE)
