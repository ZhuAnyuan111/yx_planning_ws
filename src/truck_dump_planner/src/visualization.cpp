#include "truck_dump_planner/visualization.h"
#include <geometry_msgs/Point.h>

namespace truck_dump_planner {

EnvelopeVisualizer::EnvelopeVisualizer(const std::string& frame_id)
    : frame_id_(frame_id) {}

visualization_msgs::Marker EnvelopeVisualizer::MakeBase(
    const std::string& ns, int id, const ros::Time& stamp) const {
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.pose.orientation.w = 1.0;
  return m;
}

visualization_msgs::MarkerArray EnvelopeVisualizer::BuildEnvelopeMarkers(
    const TruckEnvelope& env,
    const Vec2& excavator_xy,
    const ros::Time& stamp) const {

  visualization_msgs::MarkerArray arr;

  // 清空历史 marker
  {
    visualization_msgs::Marker del;
    del.header.frame_id = frame_id_;
    del.header.stamp = stamp;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
  }

  // 车厢包络（橙线）
  {
    visualization_msgs::Marker m = MakeBase("truck_box", 0, stamp);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.1;
    m.color.r = 1.0;
    m.color.g = 0.5;
    m.color.b = 0.0;
    m.color.a = 1.0;
    for (int i = 0; i < 5; ++i) {
      geometry_msgs::Point p;
      const Vec2& c = env.box_corners[i % 4];
      p.x = c.x;
      p.y = c.y;
      p.z = env.box_top_z;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
  }

  // 车厢纵轴方向箭头（黄）
  {
    visualization_msgs::Marker m = MakeBase("truck_heading", 0, stamp);
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 2.0;
    m.scale.y = 0.15;
    m.scale.z = 0.15;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    geometry_msgs::Point p0, p1;
    p0.x = env.center.x;
    p0.y = env.center.y;
    p0.z = env.z + 0.5;
    p1.x = env.center.x + env.forward_dir.x * 2.0;
    p1.y = env.center.y + env.forward_dir.y * 2.0;
    p1.z = env.z + 0.5;
    m.points.push_back(p0);
    m.points.push_back(p1);
    arr.markers.push_back(m);
  }

  // 车厢中心（红球）
  {
    visualization_msgs::Marker m = MakeBase("box_center", 0, stamp);
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.25;
    m.scale.y = 0.25;
    m.scale.z = 0.25;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.pose.position.x = env.box_center.x;
    m.pose.position.y = env.box_center.y;
    m.pose.position.z = env.box_top_z;
    arr.markers.push_back(m);
  }

  // 挖掘机回转中心（黑球）
  {
    visualization_msgs::Marker m = MakeBase("excavator", 0, stamp);
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.4;
    m.scale.y = 0.4;
    m.scale.z = 0.4;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.pose.position.x = excavator_xy.x;
    m.pose.position.y = excavator_xy.y;
    m.pose.position.z = 0.0;
    arr.markers.push_back(m);
  }

  return arr;
}

visualization_msgs::MarkerArray EnvelopeVisualizer::BuildDumpPointMarkers(
    const Vec3& best_point,
    const std::vector<Vec3>& reachable_points,
    const ros::Time& stamp) const {

  visualization_msgs::MarkerArray arr;

  // 最优卸载点（绿球）
  {
    visualization_msgs::Marker m = MakeBase("dump_best", 0, stamp);
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.35;
    m.scale.y = 0.35;
    m.scale.z = 0.35;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.pose.position.x = best_point.x;
    m.pose.position.y = best_point.y;
    m.pose.position.z = best_point.z;
    arr.markers.push_back(m);
  }

  // 其他可达点（浅绿小球）
  int rid = 0;
  for (const auto& pt : reachable_points) {
    visualization_msgs::Marker m = MakeBase("reachable", rid++, stamp);
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.scale.z = 0.15;
    m.color.r = 0.2;
    m.color.g = 0.8;
    m.color.b = 0.2;
    m.color.a = 0.6;
    m.pose.position.x = pt.x;
    m.pose.position.y = pt.y;
    m.pose.position.z = pt.z;
    arr.markers.push_back(m);
  }

  return arr;
}

}  // namespace truck_dump_planner
