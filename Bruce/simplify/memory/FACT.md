# 项目事实

## 工程位置与构建
- 主工程：`/home/sysu/Desktop/Project/Bruce/EasyDogFrame/Bruce/simplify`（四足机器狗 CAN 电机控制框架，CMake + C++17）。
- 构建：`cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j$(nproc)`，产物 `bin/can_motor_app`。
- CANET SDK 在 `lib/` 下：`lib/CANET.h`、`lib/ControlCAN.h`、`lib/linux_x64/{Debug,Release}/libCANET_TCP.{a,so}`。工程可完整编译（SDL2 用于手柄示例）。

## 硬件拓扑
- 4 路 CANET TCP（CAN0~3，IP 192.168.0.178，端口 4001~4004）。
- 每路 4 个电机 = 共 16 电机：motor_id 1=髋、2=大腿、3=小腿、4=轮；tx_id=motor_id，rx_id=50+motor_id。
- 标定矩阵 `MOTOR_CALIBRATION[4][4]` 位于 `include/motor_calibration.h`。

## 注意
- `PLAN.md`（12 电机）与 `TODO.md` 滞后于代码（实际 16 电机）；`TODO.md` 列的 Bug 1~5 多已在代码中修复。
- 零位偏移唯一真值来源是 `MOTOR_CALIBRATION[].pos_offset`；`robot_calibration.h` 直接引用它，勿再手抄字面量。
