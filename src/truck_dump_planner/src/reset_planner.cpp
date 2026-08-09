#include "truck_dump_planner/reset_planner.h"
#include <cmath>

namespace truck_dump_planner {

ResetTrajectory PlanResetTrajectory(const Vec3& bucket_now,
                                    const ResetPlannerParams& params) {
  ResetTrajectory traj;
  const Vec3& home = params.home_position;
  const double safe_z = params.safe_height;

  // 取当前和目标中较高的 z + safe_height 作为安全高度
  double cruise_z = std::max(bucket_now.z, home.z) + safe_z;

  // 第1段：从当前位置垂直提升到安全高度
  Vec3 lift = {bucket_now.x, bucket_now.y, cruise_z};
  traj.waypoints.push_back(bucket_now);
  traj.waypoints.push_back(lift);

  // 第2段：水平移动到待机位正上方
  Vec3 above_home = {home.x, home.y, cruise_z};
  traj.waypoints.push_back(above_home);

  // 第3段：垂直下降到待机位
  traj.waypoints.push_back(home);

  traj.success = true;
  traj.message = "reset trajectory planned (simple)";
  return traj;
}

}  // namespace truck_dump_planner
