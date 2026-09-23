#pragma once

/// @file cubic_hermite_interpolator.h
/// @brief 关节空间 PCHIP（保形分段三次 Hermite 插值）工具（纯 C++，无 ROS 依赖）。
///
/// PCHIP 在标准三次 Hermite 基上增加单调性约束，保证插值不产生超调：
///   - 2 点段：起终切线由调用方给定，经 Fritsch-Carlson 单调性修正后使用
///
/// 基函数与标准 Hermite 一致：
///   s = (t - t_i) / T,  s ∈ [0, 1]
///   h00 = 2s³ - 3s² + 1,  h10 = s³ - 2s² + s
///   h01 = -2s³ + 3s²,      h11 = s³ - s²
///   q(s) = h00·q_i + h10·T·m_i + h01·q_{i+1} + h11·T·m_{i+1}
///
/// 其中 m_i 为经 PCHIP 单调性修正后的切线。

#include <vector>

#include <kinematics/kinematics.hpp>

namespace dump_trajectory_planner {

/// 带显式起终切线的单段 PCHIP 插值（2 点，含单调性修正）。
/// 用于在线规划模式，每段可指定非零起终速度以实现平滑过渡。
/// @param q_start   起点关节角
/// @param q_end     终点关节角
/// @param v_start   起点速度 (rad/s)
/// @param v_end     终点速度 (rad/s)
/// @param duration  段时长 (s)
/// @param dt        采样步长 (s)
/// @return 采样关节序列（含起点与终点）
std::vector<kinematics::JointState> InterpolateSegmentWithTangents(
    const kinematics::JointState& q_start,
    const kinematics::JointState& q_end,
    const kinematics::JointState& v_start,
    const kinematics::JointState& v_end,
    double duration,
    double dt);

}  // namespace dump_trajectory_planner
