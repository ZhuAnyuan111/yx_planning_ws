#pragma once

/// @file angle_utils.h
/// @brief 角度换算、归一化与 swing 展开工具函数（纯 C++，无 ROS 依赖）。
///
/// 统一承载节点层与航路点生成层共用的角度工具，重点处理回转关节
/// 0°/360° 跨界问题（展开、最短角距、回包）。

#include <vector>

#include <kinematics/kinematics.hpp>

namespace dump_trajectory_planner {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double Deg2Rad(double d) { return d * kPi / 180.0; }
inline double Rad2Deg(double r) { return r * 180.0 / kPi; }

/// 将弧度归一化到 (-π, π]
double NormalizeRadToPi(double rad);

/// 最短角距离 (deg)，返回值 ∈ [0, 180]
double ShortestAngularDistanceDeg(double a_deg, double b_deg);

/// b 相对 a 的有符号最短角差 (deg)，返回值 ∈ (-180, 180]
/// 用于将 swing 角度沿最短路径展开，避免 0°/360° 跨界时方向错误
double SignedShortestDiffDeg(double a_deg, double b_deg);

/// 角度回包到 [0°, 360°)
double WrapTo360Deg(double deg);

/// 航路点序列 swing 角度展开：保证相邻点差值 ≤ π（处理 0°/360° 跨越）
void UnwrapSwingSequence(std::vector<kinematics::JointState>& waypoints);

/// 两关节状态的最大关节角误差 (deg)，swing 使用最短角距离
double MaxJointErrorDeg(const kinematics::JointState& a,
                        const kinematics::JointState& b);

}  // namespace dump_trajectory_planner
