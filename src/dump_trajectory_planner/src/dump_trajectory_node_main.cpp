/// @file dump_trajectory_node_main.cpp
/// @brief 卸载轨迹规划节点入口。

#include <ros/ros.h>

#include "dump_trajectory_planner/dump_trajectory_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dump_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  dump_trajectory_planner::DumpTrajectoryNode node(nh, pnh);

  ros::spin();
  return 0;
}
