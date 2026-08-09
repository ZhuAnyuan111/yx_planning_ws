#include <ros/ros.h>
#include "truck_dump_planner/planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dump_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  truck_dump_planner::PlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
