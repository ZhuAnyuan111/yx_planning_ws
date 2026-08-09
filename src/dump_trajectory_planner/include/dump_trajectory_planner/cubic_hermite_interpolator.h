#pragma once

/// @file cubic_hermite_interpolator.h
/// @brief 关节空间三次 Hermite 插值工具（纯 C++，无 ROS 依赖）。
///
/// 每段 [t_i, t_{i+1}] 采用三次 Hermite 基：
///   s = (t - t_i) / T,  T = t_{i+1} - t_i,  s ∈ [0, 1]
///   h00(s) = 2s³ - 3s² + 1
///   h10(s) = s³ - 2s² + s
///   h01(s) = -2s³ + 3s²
///   h11(s) = s³ - s²
///   q(s)   = h00·q_i + h10·T·v_i + h01·q_{i+1} + h11·T·v_{i+1}
///
/// 切线由调用方显式给定（在线规划模式：起点取反馈速度估计，终点取
/// 预计算的 Catmull-Rom 目标切线，门控点/端点为 0）。
///
/// 特性：C¹ 连续；段时长为 0 时直接取终点。

#include <vector>

#include <kinematics/kinematics.hpp>

namespace dump_trajectory_planner {

/// 带显式起终点切线的单段三次 Hermite 插值。
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
