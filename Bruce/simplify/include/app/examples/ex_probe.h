#pragma once
// 真机激励探针（系统辨识专用，2026-09-29 新增）
//
// Example60_SysIdProbe：悬空状态下的**通用激励 + 数据集录制**，一次覆盖三类辨识需求：
//   1. 单关节阶跃/脉冲   → 端到端延迟 T_d（指令→反馈）
//   2. 单关节位置 chirp  → 执行器频响 / 一阶+延迟模型
//   3. 轮速阶跃 + 扫频   → 轮子固件速度环伺服模型（本项目最大的结构性 sim2real gap）
//   4. 全部 12 关节 chirp → 批量执行器辨识
//
// 全程用 S2RDataset 录制（500 Hz，指令与反馈同一行、共时间基准，含 IMU/电压/温度），
// 产物 log/dataset_*.csv 直接喂给：
//   tool/dataset_health.py（体检）、tool/delay_fit.py（延迟）、tool/wheel_servo_fit.py（轮速环）
//
// 与 Example38 的分工：Ex38 用相位法在线估延迟（只打印）；本示例把原始数据落盘，
// 延迟/时间常数/伺服模型都离线拟合，可复算、可对比、可回放。
void Example60_SysIdProbe();
