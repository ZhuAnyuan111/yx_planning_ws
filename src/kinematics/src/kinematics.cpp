#include "kinematics/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace kinematics {

namespace {
constexpr double kPi = 3.14159265358979323846;

inline double deg_to_rad(double degrees) {
  return degrees * kPi / 180.0;
}

inline double rad_to_deg(double radians) {
  return radians * 180.0 / kPi;
}

inline double clamp_unit(double value) {
  return std::clamp(value, -1.0, 1.0);
}

inline double get_param_double(ros::NodeHandle& nh, const std::string& name) {
  double value = 0.0;
  nh.param<double>(name, value, 0.0);
  return value;
}
}  // namespace

// ==================== 构造函数 ====================

KinematicsSolver::KinematicsSolver(ros::NodeHandle& nh,
                                   const std::string& param_prefix) {
  const std::string p = param_prefix;
  geometry_.boom_length = get_param_double(nh, p + "/boom_length");
  geometry_.arm_length = get_param_double(nh, p + "/arm_length");
  geometry_.bucket_tooth_length = get_param_double(nh, p + "/bucket_tooth_length");
  geometry_.boom_pivot_x = get_param_double(nh, p + "/boom_pivot_x");
  geometry_.boom_pivot_y = get_param_double(nh, p + "/boom_pivot_y");
  geometry_.boom_pivot_z = get_param_double(nh, p + "/boom_pivot_z");

  limits_.boom_lower = get_param_double(nh, p + "/boom_joint_lower");
  limits_.boom_upper = get_param_double(nh, p + "/boom_joint_upper");
  limits_.arm_lower = get_param_double(nh, p + "/arm_joint_lower");
  limits_.arm_upper = get_param_double(nh, p + "/arm_joint_upper");
  limits_.bucket_lower = get_param_double(nh, p + "/bucket_joint_lower");
  limits_.bucket_upper = get_param_double(nh, p + "/bucket_joint_upper");
}

KinematicsSolver::KinematicsSolver(const LinkGeometry& geometry,
                                   const JointLimitsDeg& limits)
    : geometry_(geometry), limits_(limits) {}

// ==================== Forward Kinematics ====================

Pose3D KinematicsSolver::swing_center_forward(const JointState& q) const {
  const double boom_angle = q.boom;
  const double arm_angle = q.boom + q.arm;
  const double bucket_angle = q.boom + q.arm + q.bucket;

  Pose3D pose;
  const double radial_distance = geometry_.boom_pivot_x +
      geometry_.boom_length * std::cos(boom_angle) +
      geometry_.arm_length * std::cos(arm_angle) +
      geometry_.bucket_tooth_length * std::cos(bucket_angle);

  pose.x = std::cos(q.swing) * radial_distance;
  pose.y = std::sin(q.swing) * radial_distance + geometry_.boom_pivot_y;
  pose.z = geometry_.boom_pivot_z +
      geometry_.boom_length * std::sin(boom_angle) +
      geometry_.arm_length * std::sin(arm_angle) +
      geometry_.bucket_tooth_length * std::sin(bucket_angle);
  pose.alpha = bucket_angle;
  return pose;
}

Pose2D KinematicsSolver::boom_pivot_forward(const JointState& q) const {
  const double boom_angle = q.boom;
  const double arm_angle = q.boom + q.arm;
  const double bucket_angle = q.boom + q.arm + q.bucket;

  Pose2D pose;
  pose.x = geometry_.boom_length * std::cos(boom_angle) +
           geometry_.arm_length * std::cos(arm_angle) +
           geometry_.bucket_tooth_length * std::cos(bucket_angle);
  pose.z = geometry_.boom_length * std::sin(boom_angle) +
           geometry_.arm_length * std::sin(arm_angle) +
           geometry_.bucket_tooth_length * std::sin(bucket_angle);
  pose.alpha = bucket_angle;
  return pose;
}

// ==================== Inverse Kinematics ====================

IkResult KinematicsSolver::swing_center_inverse_by_bucket_angle(
    const SwingBaseIkRequest& request) const {
  IkResult result;
  const Pose3D& target = request.target_pose;
  const double bucket_angle_deg =
      request.bucket_angle_deg.value_or(rad_to_deg(target.alpha));
  // 铲斗长度：优先使用覆盖值（0 表示目标点为斗杆-铲斗铰接点）
  const double bucket_len = request.bucket_tooth_length_override.value_or(
      geometry_.bucket_tooth_length);

  // swing 角计算（照 MATLAB calculate_swing）：
  // swing = (atan2d(y, x) + 180) mod 360
  const double swing_rad = std::atan2(target.y - geometry_.boom_pivot_y, target.x);
  double swing_deg = rad_to_deg(swing_rad) + 180.0;
  if (swing_deg > 360.0) swing_deg -= 360.0;
  if (swing_deg < 0.0) swing_deg += 360.0;

  const double plane_distance = std::abs(std::cos(swing_rad)) < 1e-9 ?
      (target.y - geometry_.boom_pivot_y) / std::sin(swing_rad) :
      target.x / std::cos(swing_rad);
  const double x_local = plane_distance - geometry_.boom_pivot_x;
  const double z_local = target.z - geometry_.boom_pivot_z;

  const double tip_rad = deg_to_rad(bucket_angle_deg);
  const double x3 = x_local - bucket_len * std::cos(tip_rad);
  const double z3 = z_local - bucket_len * std::sin(tip_rad);
  const double l5 = std::sqrt(x_local * x_local + z_local * z_local);
  const double l6 = std::sqrt(x3 * x3 + z3 * z3);
  if (l5 < 1e-9 || l6 < 1e-9) {
    result.message = "degenerate geometry";
    return result;
  }

  const double cos_beta = (l5 * l5 + l6 * l6 - bucket_len * bucket_len) /
      (2.0 * l5 * l6);
  const double cos_alpha = (geometry_.boom_length * geometry_.boom_length +
      l6 * l6 - geometry_.arm_length * geometry_.arm_length) /
      (2.0 * geometry_.boom_length * l6);
  if (cos_beta < -1.0 || cos_beta > 1.0 ||
      cos_alpha < -1.0 || cos_alpha > 1.0) {
    result.message = "triangle cosine out of range";
    return result;
  }

  const double r = rad_to_deg(std::atan2(z_local, x_local));
  const double b = rad_to_deg(std::acos(clamp_unit(cos_beta)));
  const double a = rad_to_deg(std::acos(clamp_unit(cos_alpha)));

  const double boom_deg = r + b + a;
  const double arm_deg = rad_to_deg(std::acos(clamp_unit(
      (geometry_.boom_length * geometry_.boom_length +
       geometry_.arm_length * geometry_.arm_length - l6 * l6) /
      (2.0 * geometry_.boom_length * geometry_.arm_length)))) - 180.0;
  // bucket_angle 模式：直接使用输入的铲斗关节角（非姿态角）
  const double bucket_deg = bucket_angle_deg;

  if (!(boom_deg >= limits_.boom_lower && boom_deg <= limits_.boom_upper &&
        arm_deg >= limits_.arm_lower && arm_deg <= limits_.arm_upper &&
        bucket_deg >= limits_.bucket_lower &&
        bucket_deg <= limits_.bucket_upper)) {
    result.message = "joint limits violated: boom=" +
        std::to_string(boom_deg) + " arm=" + std::to_string(arm_deg) +
        " bucket=" + std::to_string(bucket_deg);
    return result;
  }

  result.success = true;
  result.q.swing = deg_to_rad(swing_deg);
  result.q.boom = deg_to_rad(boom_deg);
  result.q.arm = deg_to_rad(arm_deg);
  result.q.bucket = deg_to_rad(bucket_deg);
  result.message = "swing-center IK solved";
  return result;
}

IkResult KinematicsSolver::swing_center_inverse_by_bucket_attitude(
    const SwingBaseIkRequest& request) const {
  IkResult result;
  const Pose3D& target = request.target_pose;
  const double bucket_attitude_deg =
      request.bucket_attitude_deg.value_or(rad_to_deg(target.alpha));
  // 铲斗长度：优先使用覆盖值（0 表示目标点为斗杆-铲斗铰接点）
  const double bucket_len = request.bucket_tooth_length_override.value_or(
      geometry_.bucket_tooth_length);

  // swing 角计算（照 MATLAB calculate_swing）：
  // swing = (atan2d(y, x) + 180) mod 360
  const double swing_rad = std::atan2(target.y - geometry_.boom_pivot_y, target.x);
  double swing_deg = rad_to_deg(swing_rad) + 180.0;
  if (swing_deg > 360.0) swing_deg -= 360.0;
  if (swing_deg < 0.0) swing_deg += 360.0;

  const double plane_distance = std::abs(std::cos(swing_rad)) < 1e-9 ?
      (target.y - geometry_.boom_pivot_y) / std::sin(swing_rad) :
      target.x / std::cos(swing_rad);
  const double x_local = plane_distance - geometry_.boom_pivot_x;
  const double z_local = target.z - geometry_.boom_pivot_z;

  const double tip_rad = deg_to_rad(bucket_attitude_deg);
  const double x3 = x_local - bucket_len * std::cos(tip_rad);
  const double z3 = z_local - bucket_len * std::sin(tip_rad);
  const double l5 = std::sqrt(x_local * x_local + z_local * z_local);
  const double l6 = std::sqrt(x3 * x3 + z3 * z3);
  if (l5 < 1e-9 || l6 < 1e-9) {
    result.message = "degenerate geometry";
    return result;
  }

  const double cos_beta = (l5 * l5 + l6 * l6 - bucket_len * bucket_len) /
      (2.0 * l5 * l6);
  const double cos_alpha = (geometry_.boom_length * geometry_.boom_length +
      l6 * l6 - geometry_.arm_length * geometry_.arm_length) /
      (2.0 * geometry_.boom_length * l6);
  if (cos_beta < -1.0 || cos_beta > 1.0 ||
      cos_alpha < -1.0 || cos_alpha > 1.0) {
    result.message = "triangle cosine out of range";
    return result;
  }

  const double r = rad_to_deg(std::atan2(z_local, x_local));
  const double b = rad_to_deg(std::acos(clamp_unit(cos_beta)));
  const double a = rad_to_deg(std::acos(clamp_unit(cos_alpha)));

  const double boom_deg = r + b + a;
  const double arm_deg = rad_to_deg(std::acos(clamp_unit(
      (geometry_.boom_length * geometry_.boom_length +
       geometry_.arm_length * geometry_.arm_length - l6 * l6) /
      (2.0 * geometry_.boom_length * geometry_.arm_length)))) - 180.0;
  const double bucket_deg = bucket_attitude_deg - boom_deg - arm_deg;

  if (!(boom_deg >= limits_.boom_lower && boom_deg <= limits_.boom_upper &&
        arm_deg >= limits_.arm_lower && arm_deg <= limits_.arm_upper &&
        bucket_deg >= limits_.bucket_lower &&
        bucket_deg <= limits_.bucket_upper)) {
    result.message = "joint limits violated";
    return result;
  }

  result.success = true;
  result.q.swing = deg_to_rad(swing_deg);
  result.q.boom = deg_to_rad(boom_deg);
  result.q.arm = deg_to_rad(arm_deg);
  result.q.bucket = deg_to_rad(bucket_deg);
  result.message = "swing-center IK solved";
  return result;
}

IkResult KinematicsSolver::boom_pivot_inverse_by_bucket_attitude(
    const BoomPivotIkRequest& request) const {
  IkResult result;
  const Pose2D& target = request.target_pose;
  const double bucket_attitude_deg =
      request.bucket_attitude_deg.value_or(rad_to_deg(target.alpha));

  const double x = target.x;
  const double z = target.z;
  const double tip_rad = deg_to_rad(bucket_attitude_deg);
  const double x3 = x - geometry_.bucket_tooth_length * std::cos(tip_rad);
  const double z3 = z - geometry_.bucket_tooth_length * std::sin(tip_rad);
  const double l5 = std::sqrt(x * x + z * z);
  const double l6 = std::sqrt(x3 * x3 + z3 * z3);
  if (l5 < 1e-9 || l6 < 1e-9) {
    result.message = "degenerate geometry";
    return result;
  }

  const double cos_beta = (l5 * l5 + l6 * l6 -
      geometry_.bucket_tooth_length * geometry_.bucket_tooth_length) /
      (2.0 * l5 * l6);
  const double cos_alpha = (geometry_.boom_length * geometry_.boom_length +
      l6 * l6 - geometry_.arm_length * geometry_.arm_length) /
      (2.0 * geometry_.boom_length * l6);
  if (cos_beta < -1.0 || cos_beta > 1.0 ||
      cos_alpha < -1.0 || cos_alpha > 1.0) {
    result.message = "triangle cosine out of range";
    return result;
  }

  const double r = rad_to_deg(std::atan2(z, x));
  const double b = rad_to_deg(std::acos(clamp_unit(cos_beta)));
  const double a = rad_to_deg(std::acos(clamp_unit(cos_alpha)));

  const double boom_deg = r + b + a;
  const double arm_deg = rad_to_deg(std::acos(clamp_unit(
      (geometry_.boom_length * geometry_.boom_length +
       geometry_.arm_length * geometry_.arm_length - l6 * l6) /
      (2.0 * geometry_.boom_length * geometry_.arm_length)))) - 180.0;
  const double bucket_deg = bucket_attitude_deg - boom_deg - arm_deg;

  if (!(boom_deg >= limits_.boom_lower && boom_deg <= limits_.boom_upper &&
        arm_deg >= limits_.arm_lower && arm_deg <= limits_.arm_upper &&
        bucket_deg >= limits_.bucket_lower &&
        bucket_deg <= limits_.bucket_upper)) {
    result.message = "joint limits violated";
    return result;
  }

  result.success = true;
  result.q.boom = deg_to_rad(boom_deg);
  result.q.arm = deg_to_rad(arm_deg);
  result.q.bucket = deg_to_rad(bucket_deg);
  result.message = "boom-pivot IK solved";
  return result;
}

}  // namespace kinematics
