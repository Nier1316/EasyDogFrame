QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14

TARGET = MotorControlTool
TEMPLATE = app

# 源文件
SOURCES += \
    main.cpp \
    motor_control_gui.cpp

# 头文件
HEADERS += \
    motor_control_gui.h

# 包含路径 - 指向主项目的include目录
INCLUDEPATH += ../include

# 链接库 - 链接编译好的静态库
LIBS += -L../lib -lmotor_framework

# 链接pthread（因为框架使用了std::thread）
LIBS += -lpthread

# 编译选项
QMAKE_CXXFLAGS += -Wall -Wextra -Wpedantic

# 输出目录配置
DESTDIR = ./bin                 # 可执行文件输出目录
OBJECTS_DIR = ./build/obj       # 对象文件输出目录
MOC_DIR = ./build/moc           # MOC生成文件输出目录
RCC_DIR = ./build/rcc           # 资源文件输出目录
UI_DIR = ./build/ui             # UI生成文件输出目录
