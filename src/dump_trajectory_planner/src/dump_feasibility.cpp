#include "dump_trajectory_planner/dump_feasibility.h"

#include <cmath>
#include <sstream>

namespace dump_trajectory_planner {

FeasibilityResult CheckReachable(const Point3& unload_point,
                                 const FeasibilityParams& params,
                                 const kinematics::KinematicsSolver& solver) {
  FeasibilityResult r;

  // ---- 1. 可选包络粗筛 ----
  if (params.enable_envelope_prefilter) {
    const double radius = std::sqrt(unload_point.x * unload_point.x +
                                     unload_point.y * unload_point.y);
    if (radius < params.reach_min) {
      std::ostringstream oss;
      oss << "envelope: radius=" << radius
          << " < reach_min=" << params.reach_min;
      r.reason = oss.str();
      return r;
    }
    if (radius > params.reach_max) {
      std::ostringstream oss;
      oss << "envelope: radius=" << radius
          << " > reach_max=" << params.reach_max;
      r.reason = oss.str();
      return r;
    }
    if (unload_point.z < params.dump_height_min) {
      std::ostringstream oss;
      oss << "envelope: z=" << unload_point.z
          << " < dump_height_min=" << params.dump_height_min;
      r.reason = oss.str();
      return r;
    }
    if (unload_point.z > params.dump_height_max) {
      std::ostringstream oss;
      oss << "envelope: z=" << unload_point.z
          << " > dump_height_max=" << params.dump_height_max;
      r.reason = oss.str();
      return r;
    }
  }

  // ---- 2. 真实 IK 判定 ----
  kinematics::SwingBaseIkRequest req;
  req.target_pose.x = unload_point.x;
  req.target_pose.y = unload_point.y;
  req.target_pose.z = unload_point.z;
  req.target_pose.alpha = 0.0;  // 未使用（由 bucket_attitude_deg 覆盖）
  req.bucket_attitude_deg = params.bucket_attitude_deg;

  const auto ik = solver.swing_center_inverse_by_bucket_attitude(req);
  if (!ik.success) {
    std::ostringstream oss;
    oss << "IK failed: " << ik.message;
    r.reason = oss.str();
    return r;
  }

  r.reachable = true;
  r.unload_joint = ik.q;
  r.reason = "reachable";
  return r;
}

}  // namespace dump_trajectory_planner
