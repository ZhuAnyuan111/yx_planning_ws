#ifndef TRUCK_DUMP_PLANNER_RESET_PLANNER_H
#define TRUCK_DUMP_PLANNER_RESET_PLANNER_H

#include <vector>
#include "truck_dump_planner/truck_model.h"

namespace truck_dump_planner {

/// 复位轨迹：铲斗从当前位置回到待机位
struct ResetTrajectory {
  bool success = false;
  std::vector<Vec3> waypoints;  // 铲斗齿尖轨迹点（base 系）
  std::string message;
};

/// 复位规划参数
struct ResetPlannerParams {
  Vec3 home_position{2.0, 0.0, 1.5};   // 待机位（base 系）
  double safe_height = 3.0;             // 安全提升高度（避障）
};

/// 最简复位轨迹规划：提升→平移→下降
ResetTrajectory PlanResetTrajectory(const Vec3& bucket_now,
                                    const ResetPlannerParams& params);

}  // namespace truck_dump_planner

#endif  // TRUCK_DUMP_PLANNER_RESET_PLANNER_H
