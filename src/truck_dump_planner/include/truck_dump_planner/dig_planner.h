#ifndef TRUCK_DUMP_PLANNER_DIG_PLANNER_H
#define TRUCK_DUMP_PLANNER_DIG_PLANNER_H

#include <vector>
#include "truck_dump_planner/truck_model.h"

namespace truck_dump_planner {

/// 挖掘轨迹：铲斗从当前位置运动到挖掘点并完成铲取
struct DigTrajectory {
  bool success = false;
  std::vector<Vec3> waypoints;  // 铲斗齿尖轨迹点（base 系）
  std::string message;
};

/// 挖掘规划参数
struct DigPlannerParams {
  Vec3 dig_point{3.0, 0.0, -0.5};   // 挖掘目标点（base 系）
  double approach_height = 1.0;       // 接近高度（挖掘点上方）
  double dig_depth = 0.5;            // 入土深度
};

/// 最简挖掘轨迹规划：直线三段式（下降→铲入→提升）
DigTrajectory PlanDigTrajectory(const Vec3& bucket_now,
                                const DigPlannerParams& params);

}  // namespace truck_dump_planner

#endif  // TRUCK_DUMP_PLANNER_DIG_PLANNER_H
