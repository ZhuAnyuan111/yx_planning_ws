/// @file truck_smooth_node_main.cpp
/// @brief ROS 节点入口：初始化 ROS、构造节点类并驱动事件循环。
///        所有业务逻辑封装在 TruckSmoothNode 类中。

#include <ros/ros.h>

#include "truck_smooth_planner/truck_smooth_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "truck_smooth_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  truck_smooth_planner::TruckSmoothNode node(nh, pnh);

  ros::spin();
  return 0;
}
