#include "truck_dump_planner/dig_planner.h"

namespace truck_dump_planner {

DigTrajectory PlanDigTrajectory(const Vec3& bucket_now,
                                const DigPlannerParams& params) {
  DigTrajectory traj;
  const Vec3& dp = params.dig_point;

  // 第1段：从当前位置移动到挖掘点正上方（接近）
  Vec3 approach = {dp.x, dp.y, dp.z + params.approach_height};
  traj.waypoints.push_back(bucket_now);
  traj.waypoints.push_back(approach);

  // 第2段：垂直下降到挖掘点（入土）
  traj.waypoints.push_back(dp);

  // 第3段：向后铲入（沿 -x 方向铲一段）
  Vec3 scoop_end = {dp.x - params.dig_depth, dp.y, dp.z};
  traj.waypoints.push_back(scoop_end);

  // 第4段：提升（铲满后抬起）
  Vec3 lift = {scoop_end.x, scoop_end.y, scoop_end.z + params.approach_height};
  traj.waypoints.push_back(lift);

  traj.success = true;
  traj.message = "dig trajectory planned (simple)";
  return traj;
}

}  // namespace truck_dump_planner
