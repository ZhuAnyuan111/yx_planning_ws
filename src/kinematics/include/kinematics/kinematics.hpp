#pragma once

/// @file kinematics.hpp
/// @brief 挖掘机运动学求解库（ROS1 Noetic 版）。
///
/// 关节链：swing → boom → arm → bucket
///   - 关节角以 rad 存储；IK 输入/输出时的姿态角单位为 deg；
///   - 支持两种构造方式：
///       1) 从 ros::NodeHandle 加载参数服务器；
///       2) 直接传入几何 / 限位结构体（无 ROS 依赖）。

#include <optional>
#include <string>

#include <ros/node_handle.h>

namespace kinematics {

struct JointState {
  double swing = 0.0;
  double boom = 0.0;
  double arm = 0.0;
  double bucket = 0.0;
};

struct Pose2D {
  double x = 0.0;
  double z = 0.0;
  double alpha = 0.0;
};

struct Pose3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double alpha = 0.0;
};

struct LinkGeometry {
  double boom_pivot_x = 0.09;
  double boom_pivot_y = 0.03;
  double boom_pivot_z = 0.0;
  double boom_length = 6.987;
  double arm_length = 2.797;
  double bucket_tooth_length = 2.359;
};

struct JointLimitsDeg {
  double boom_lower = -20.0;
  double boom_upper = 45.0;
  double arm_lower = -153.0;
  double arm_upper = -30.0;
  double bucket_lower = -135.0;
  double bucket_upper = 30.0;
};

struct IkResult {
  bool success = false;
  JointState q;
  std::string message;
};

struct SwingBaseIkRequest {
  Pose3D target_pose;
  std::optional<double> bucket_angle_deg;
  std::optional<double> bucket_attitude_deg;
  std::optional<bool> elbow_up;
  /// 铲斗长度覆盖（m）：设置后 IK 使用此值代替 geometry_.bucket_tooth_length
  /// 设为 0 表示目标点为斗杆-铲斗铰接点（而非齿尖）
  std::optional<double> bucket_tooth_length_override;
};

struct BoomPivotIkRequest {
  Pose2D target_pose;
  std::optional<double> bucket_attitude_deg;
  std::optional<bool> elbow_up;
};

class KinematicsSolver {
 public:
  /// 从 ROS 参数服务器加载机构参数
  /// @param nh           NodeHandle（一般传入 pnh_）
  /// @param param_prefix 参数命名空间前缀（默认 "kinematics"）
  explicit KinematicsSolver(ros::NodeHandle& nh,
                            const std::string& param_prefix = "kinematics");

  /// 直接注入几何 / 关节限位（无 ROS 依赖，便于单元测试或跨包复用）
  KinematicsSolver(const LinkGeometry& geometry,
                   const JointLimitsDeg& limits);

  // ---------- Forward Kinematics ----------
  Pose3D swing_center_forward(const JointState& q) const;
  Pose2D boom_pivot_forward(const JointState& q) const;

  // ---------- Inverse Kinematics ----------
  IkResult swing_center_inverse_by_bucket_angle(
      const SwingBaseIkRequest& request) const;
  IkResult swing_center_inverse_by_bucket_attitude(
      const SwingBaseIkRequest& request) const;
  IkResult boom_pivot_inverse_by_bucket_attitude(
      const BoomPivotIkRequest& request) const;

  // ---------- Getters ----------
  const LinkGeometry& geometry() const { return geometry_; }
  const JointLimitsDeg& limits() const { return limits_; }

 private:
  LinkGeometry geometry_;
  JointLimitsDeg limits_;
};

}  // namespace kinematics
