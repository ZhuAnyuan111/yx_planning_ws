#pragma once

/// @file angle_utils.h
/// @brief 角度换算、归一化与最短角差工具（纯 C++，无 ROS 依赖）。
///
/// 重点处理回转关节 0°/360° 跨界问题（归一化、有符号最短角差、无符号最短角距）。

namespace truck_smooth_planner {

constexpr double kPi = 3.14159265358979323846;

inline double Deg2Rad(double d) { return d * kPi / 180.0; }
inline double Rad2Deg(double r) { return r * 180.0 / kPi; }

/// 将角度归一化到 [0, 360)
double NormalizeAngle(double angle);

/// b 相对 a 的有符号最短角差 (deg)，返回值 ∈ (-180, 180]
/// 用于 swing 沿最短路径展开，避免 0°/360° 跨界时方向错误
double SignedShortestDiffDeg(double a_deg, double b_deg);

/// 两角之间的无符号最短角距 (deg)，返回值 ∈ [0, 180]
double ShortestAngularDistanceDeg(double a_deg, double b_deg);

}  // namespace truck_smooth_planner
