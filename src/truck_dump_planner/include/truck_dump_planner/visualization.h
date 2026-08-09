#ifndef TRUCK_DUMP_PLANNER_VISUALIZATION_H
#define TRUCK_DUMP_PLANNER_VISUALIZATION_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "truck_dump_planner/truck_model.h"

namespace truck_dump_planner {

/// 卡车车厢包络可视化构建器（不持有 publisher，只生成 MarkerArray）
class EnvelopeVisualizer {
 public:
  explicit EnvelopeVisualizer(const std::string& frame_id);

  /// 构建车厢包络 + 朝向 + 中心 + 挖掘机回转中心的 MarkerArray
  visualization_msgs::MarkerArray BuildEnvelopeMarkers(
      const TruckEnvelope& env,
      const Vec2& excavator_xy,
      const ros::Time& stamp) const;

  /// 构建可达卸载点可视化（绿球=最优，浅绿=其他可达）
  visualization_msgs::MarkerArray BuildDumpPointMarkers(
      const Vec3& best_point,
      const std::vector<Vec3>& reachable_points,
      const ros::Time& stamp) const;

 private:
  std::string frame_id_;

  visualization_msgs::Marker MakeBase(const std::string& ns, int id,
                                      const ros::Time& stamp) const;
};

}  // namespace truck_dump_planner

#endif  // TRUCK_DUMP_PLANNER_VISUALIZATION_H
