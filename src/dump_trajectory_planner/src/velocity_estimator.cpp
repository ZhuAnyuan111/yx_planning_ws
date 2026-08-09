#include "dump_trajectory_planner/velocity_estimator.h"

#include <algorithm>

#include "dump_trajectory_planner/angle_utils.h"

namespace dump_trajectory_planner {

namespace {
/// 估计速度限幅 (rad/s)，防止噪声导致估计过大
constexpr double kMaxVel = 30.0 * kPi / 180.0;  // 30 deg/s
}  // namespace

void VelocityEstimator::PushSwing(double stamp_sec, double swing_rad) {
  swing_hist_.push_back({stamp_sec, swing_rad});
  if (swing_hist_.size() > kHistorySize) {
    swing_hist_.pop_front();
  }
}

void VelocityEstimator::PushJoints(double stamp_sec,
                                   const kinematics::JointState& q) {
  joints_hist_.push_back({stamp_sec, q});
  if (joints_hist_.size() > kHistorySize) {
    joints_hist_.pop_front();
  }
}

kinematics::JointState VelocityEstimator::Estimate() const {
  kinematics::JointState v{};

  // swing：独立历史最近 3 帧首末差分
  if (swing_hist_.size() >= 3) {
    const size_t n = swing_hist_.size();
    const auto& newest = swing_hist_[n - 1];
    const auto& oldest = swing_hist_[n - 3];
    const double dt = newest.first - oldest.first;
    if (dt > 1e-6) {
      // swing 为 wrapped 角（下游 0~360°），首末直接相减在跨 0°/360° 时会得到
      // ±358° 的伪大值并被限幅成符号错误的切线 → 必须先归一化到 (-π, π]
      v.swing = NormalizeRadToPi(newest.second - oldest.second) / dt;
    }
  }

  // boom/arm/bucket：独立历史最近 3 帧首末差分
  if (joints_hist_.size() >= 3) {
    const size_t n = joints_hist_.size();
    const auto& newest = joints_hist_[n - 1];
    const auto& oldest = joints_hist_[n - 3];
    const double dt = newest.first - oldest.first;
    if (dt > 1e-6) {
      v.boom = (newest.second.boom - oldest.second.boom) / dt;
      v.arm = (newest.second.arm - oldest.second.arm) / dt;
      v.bucket = (newest.second.bucket - oldest.second.bucket) / dt;
    }
  }

  // 限幅防噪
  v.swing = std::max(-kMaxVel, std::min(kMaxVel, v.swing));
  v.boom = std::max(-kMaxVel, std::min(kMaxVel, v.boom));
  v.arm = std::max(-kMaxVel, std::min(kMaxVel, v.arm));
  v.bucket = std::max(-kMaxVel, std::min(kMaxVel, v.bucket));

  return v;
}

void VelocityEstimator::Clear() {
  swing_hist_.clear();
  joints_hist_.clear();
}

}  // namespace dump_trajectory_planner
