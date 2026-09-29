#pragma once
// 系统辨识 / 通道校验示例（为「触地检测 · 接触力估计」做前提件）
//
// 分工（勿与已有示例重复）：
//   Example47（chirp 扫频）已辨识 J / B / f_c / K_g / b —— 本文件的示例不重复这些。
//   Example54（吊装摩擦辨识）已辨识腿库仑摩擦 f_c。
//   本文件补 Ex47/54 给不出的两件前提：
//     ① Example58 —— 16 路**力矩通道**是否可信（零偏/增益/符号/线性度）。
//        任何"用 τ 反解外力"的做法都以这一步为前提。
//     ② Example59 —— **重力矩系数 G_j = m·g·d** 与质量-质心，用于填
//        robot_calibration.h 里仍是 TODO 的 LINK_DYNAMICS / BODY_MASS。
//        没有重力项就无法把自重从 τ 里扣掉，f = (Jᵀ)⁻¹τ 的误差会被 J 的条件数
//        放大成几十牛（见 include/motion/leg_kinematics.h 的 leg_cond_proxy 标定）。
//
// 建议顺序：Example58（通道可信）→ Example59（重力/质量）→ Example47（J/B/f_c）
//           → 之后才谈接触检测。
void Example58_TorqueChannelCheck();
void Example59_GravityMassIdentify();
