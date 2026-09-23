#include "dump_trajectory_planner/cubic_hermite_interpolator.h"

#include <algorithm>
#include <cmath>

namespace dump_trajectory_planner {

namespace {

/// 三次 Hermite 基函数求值（标量）
/// s ∈ [0,1]，m0/m1 为经 PCHIP 修正后的切线 (rad/s)，T 为段时长
inline double HermiteScalar(double q0, double q1,
                             double m0, double m1,
                             double T, double s) {
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  return h00 * q0 + h10 * T * m0 + h01 * q1 + h11 * T * m1;
}

inline kinematics::JointState HermiteJoint(const kinematics::JointState& q0,
                                            const kinematics::JointState& q1,
                                            const kinematics::JointState& m0,
                                            const kinematics::JointState& m1,
                                            double T, double s) {
  kinematics::JointState q;
  q.swing  = HermiteScalar(q0.swing,  q1.swing,  m0.swing,  m1.swing,  T, s);
  q.boom   = HermiteScalar(q0.boom,   q1.boom,   m0.boom,   m1.boom,   T, s);
  q.arm    = HermiteScalar(q0.arm,    q1.arm,    m0.arm,    m1.arm,    T, s);
  q.bucket = HermiteScalar(q0.bucket, q1.bucket, m0.bucket, m1.bucket, T, s);
  return q;
}

/// PCHIP 单调性修正（Fritsch-Carlson 方法，单关节标量）
/// 给定弦斜率 delta 和初始切线 m，返回修正后的切线，保证插值不产生超调。
inline double PchipClampScalar(double delta, double m) {
  if (std::abs(delta) < 1e-12) {
    return 0.0;  // 平坦区域，切线归零
  }
  // 若切线与弦斜率异号，截断为 0（保持单调性）
  if (m * delta < 0.0) {
    return 0.0;
  }
  // Fritsch-Carlson: α = m / delta，限制 α ≤ 3 使 m 不超过 3|δ|
  const double alpha = m / delta;
  const double alpha_max = 3.0;
  if (alpha > alpha_max) {
    return alpha_max * delta;
  }
  return m;
}

/// 单关节标量的弦斜率
inline double ScalarDelta(double q0, double q1, double T) {
  return (T > 1e-12) ? (q1 - q0) / T : 0.0;
}

/// 计算 2 点段的弦斜率向量
inline kinematics::JointState ComputeDelta(
    const kinematics::JointState& q0,
    const kinematics::JointState& q1,
    double T) {
  kinematics::JointState d;
  d.swing  = ScalarDelta(q0.swing,  q1.swing,  T);
  d.boom   = ScalarDelta(q0.boom,   q1.boom,   T);
  d.arm    = ScalarDelta(q0.arm,    q1.arm,    T);
  d.bucket = ScalarDelta(q0.bucket, q1.bucket, T);
  return d;
}

}  // namespace

// ==================== 单段 PCHIP 插值（2 点） ====================

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

  // 弦斜率
  const kinematics::JointState delta = ComputeDelta(q_start, q_end, duration);

  // PCHIP 单调性修正：用实际弦斜率对起终切线施加 Fritsch-Carlson 约束
  auto clamp_with_delta = [](const kinematics::JointState& v,
                              const kinematics::JointState& d) {
    kinematics::JointState m;
    m.swing  = PchipClampScalar(d.swing,  v.swing);
    m.boom   = PchipClampScalar(d.boom,   v.boom);
    m.arm    = PchipClampScalar(d.arm,    v.arm);
    m.bucket = PchipClampScalar(d.bucket, v.bucket);
    return m;
  };
  const kinematics::JointState m_start = clamp_with_delta(v_start, delta);
  const kinematics::JointState m_end   = clamp_with_delta(v_end,   delta);

  traj.reserve(static_cast<size_t>(duration / dt) + 2);

  double t = 0.0;
  while (t < duration - 1e-9) {
    double s = t / duration;
    s = std::max(0.0, std::min(1.0, s));
    traj.push_back(HermiteJoint(q_start, q_end, m_start, m_end, duration, s));
    t += dt;
  }
  // 末帧：确保精确到终点
  traj.push_back(q_end);
  return traj;
}

}  // namespace dump_trajectory_planner
