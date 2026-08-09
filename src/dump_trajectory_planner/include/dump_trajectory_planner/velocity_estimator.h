#pragma once

/// @file velocity_estimator.h
/// @brief 双路关节速度估计器（纯 C++，无 ROS 依赖，便于单元测试）。
///
/// swing 与 boom/arm/bucket 来自两个异步话题（发布频率可能不同），
/// 两路历史独立维护、各自用自身时间戳做最近 3 帧首末差分，
/// 避免跨话题打补丁导致的相位偏差；估计结果限幅 ±30°/s 防噪。

#include <deque>
#include <utility>

#include <kinematics/kinematics.hpp>

namespace dump_trajectory_planner {

class VelocityEstimator {
 public:
  static constexpr size_t kHistorySize = 5;  // 每路保留最近 N 帧

  /// 写入一帧 swing 反馈（rad），stamp_sec 为该帧到达时刻（秒）
  void PushSwing(double stamp_sec, double swing_rad);

  /// 写入一帧臂架反馈（仅 boom/arm/bucket 分量有效，swing 不使用）
  void PushJoints(double stamp_sec, const kinematics::JointState& q);

  /// 估计当前关节速度 (rad/s)；历史不足 3 帧的分量返回 0
  kinematics::JointState Estimate() const;

  /// 清空两路历史（如门控停车等待后速度归零语义）
  void Clear();

 private:
  std::deque<std::pair<double, double>> swing_hist_;                 // (t, swing)
  std::deque<std::pair<double, kinematics::JointState>> joints_hist_;  // (t, q)
};

}  // namespace dump_trajectory_planner
