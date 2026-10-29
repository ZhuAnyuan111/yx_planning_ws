/// @file smooth_trajectory_node_main.cpp
/// @brief ROS 节点入口：初始化 ROS、构造节点类并驱动事件循环。
///        所有业务逻辑封装在 SmoothTrajectoryNode 类中。

#include <ros/ros.h>

#include "smooth_trajectory_planner/smooth_trajectory_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "smooth_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  smooth_trajectory_planner::SmoothTrajectoryNode node(nh, pnh);

  ros::spin();
  return 0;
}
