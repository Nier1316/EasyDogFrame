cmake_minimum_required(VERSION 3.8)

project(CANET)

include_directories(../base/)

set(CMAKE_CXX_STANDARD 11)

add_definitions(-D _CRT_SECURE_NO_WARNINGS)

set(PLATFORM_TAG windows)
IF(UNIX) 
    set(PLATFORM_TAG linux)
    set(OUT_PLATFORM_DIR ${PLATFORM_TAG}_x64)
    find_package(Threads)
ELSE()
    IF(CMAKE_CL_64) 
    set(OUT_PLATFORM_DIR ${PLATFORM_TAG}_x64)
    ELSE(CMAKE_CL_64)
    set(OUT_PLATFORM_DIR ${PLATFORM_TAG}_x86)
    ENDIF(CMAKE_CL_64)
ENDIF()

set(SRC
    ${PROJECT_SOURCE_DIR}/CANET.h
    ${PROJECT_SOURCE_DIR}/CANET.cpp 
    ${PROJECT_SOURCE_DIR}/Device.h
    ${PROJECT_SOURCE_DIR}/Device.cpp 
    ${PROJECT_SOURCE_DIR}/DeviceMgr.h
    ${PROJECT_SOURCE_DIR}/DeviceMgr.cpp   
    ${PROJECT_SOURCE_DIR}/CANET.def
    ${PROJECT_SOURCE_DIR}/service/TcpClient.h
    ${PROJECT_SOURCE_DIR}/service/TcpClient.cpp
    ${PROJECT_SOURCE_DIR}/service/TcpServer.h
    ${PROJECT_SOURCE_DIR}/service/TcpServer.cpp
    ${PROJECT_SOURCE_DIR}/../base/log.h
    ${PROJECT_SOURCE_DIR}/../base/log.cpp
    ${PROJECT_SOURCE_DIR}/../base/common.h
    ${PROJECT_SOURCE_DIR}/../base/common.cpp
    ${PROJECT_SOURCE_DIR}/../base/network.h
    ${PROJECT_SOURCE_DIR}/../base/platform/${PLATFORM_TAG}/network.cpp  
)

set(LIB_NAME "CANET_TCP")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin/${LIB_NAME}/${OUT_PLATFORM_DIR}/${CMAKE_BUILD_TYPE})
add_library(${LIB_NAME} SHARED ${SRC})

IF(WIN32)
target_link_libraries(${LIB_NAME} Winmm.lib)
ENDIF()

IF(UNIX)
    add_library(CANET_STATIC STATIC ${SRC}) 

    # static and shared lib have the same name
    set_target_properties(CANET_STATIC PROPERTIES OUTPUT_NAME ${LIB_NAME})
    set_target_properties(CANET_STATIC PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    set_target_properties(${LIB_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    
    target_link_libraries(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT})
ENDIF()

# test lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin/${LIB_NAME}/${OUT_PLATFORM_DIR}/${CMAKE_BUILD_TYPE})
add_executable(canet_test ${PROJECT_SOURCE_DIR}/test.cpp)
target_link_libraries(canet_test ${LIB_NAME})
