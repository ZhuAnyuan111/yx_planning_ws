/// @file reset_trajectory_node_main.cpp
/// @brief ROS 节点入口：仅负责初始化 ROS、构造节点类并驱动事件循环。
///        所有业务逻辑封装在 ResetTrajectoryNode 类中。

#include <ros/ros.h>

#include "reset_trajectory_planner/reset_trajectory_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "reset_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  reset_trajectory_planner::ResetTrajectoryNode node(nh, pnh);

  ros::spin();
  return 0;
}
