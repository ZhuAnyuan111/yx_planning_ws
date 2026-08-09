#include "dump_trajectory_planner/cubic_hermite_interpolator.h"

#include <algorithm>
#include <cmath>

namespace dump_trajectory_planner {

namespace {

/// 单关节标量的三次 Hermite 插值
/// s ∈ [0,1]，v0/v1 单位为 rad/s（相对时间 t），T 为段时长（s）
inline double HermiteScalar(double q0, double q1,
                             double v0, double v1,
                             double T, double s) {
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  return h00 * q0 + h10 * T * v0 + h01 * q1 + h11 * T * v1;
}

inline kinematics::JointState HermiteJoint(const kinematics::JointState& q0,
                                            const kinematics::JointState& q1,
                                            const kinematics::JointState& v0,
                                            const kinematics::JointState& v1,
                                            double T, double s) {
  kinematics::JointState q;
  q.swing = HermiteScalar(q0.swing, q1.swing, v0.swing, v1.swing, T, s);
  q.boom = HermiteScalar(q0.boom, q1.boom, v0.boom, v1.boom, T, s);
  q.arm = HermiteScalar(q0.arm, q1.arm, v0.arm, v1.arm, T, s);
  q.bucket = HermiteScalar(q0.bucket, q1.bucket, v0.bucket, v1.bucket, T, s);
  return q;
}

}  // namespace

std::vector<kinematics::JointState> InterpolateSegmentWithTangents(
    const kinematics::JointState& q_start,
    const kinematics::JointState& q_end,
    const kinematics::JointState& v_start,
    const kinematics::JointState& v_end,
    double duration,
    double dt) {
  std::vector<kinematics::JointState> traj;
  if (duration <= 0.0 || dt <= 0.0) {
    traj.push_back(q_end);
    return traj;
  }

  traj.reserve(static_cast<size_t>(duration / dt) + 2);

  double t = 0.0;
  while (t < duration - 1e-9) {
    double s = t / duration;
    s = std::max(0.0, std::min(1.0, s));
    traj.push_back(HermiteJoint(q_start, q_end, v_start, v_end, duration, s));
    t += dt;
  }
  // 末帧：确保精确到终点
  traj.push_back(q_end);
  return traj;
}

}  // namespace dump_trajectory_planner
