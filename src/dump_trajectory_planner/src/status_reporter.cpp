#include "dump_trajectory_planner/status_reporter.h"

namespace dump_trajectory_planner {

StatusReporter::StatusReporter(ros::NodeHandle& nh) {
  pub_swing_ = nh.advertise<geometry_msgs::Point>("/Swing_topic", 1);
  pub_bucket_pos_ = nh.advertise<geometry_msgs::Point>("/RealBktPosXYZ", 1);
  // latched：后启动的订阅者也能立即读到当前状态
  pub_finish_flag_ = nh.advertise<std_msgs::Float64>(
      "/Sys_RUn_FlagUnloadExcuteFinish", 1, true);
  pub_up_feasible_ = nh.advertise<std_msgs::Bool>("/UPFeasible", 1, true);
  pub_max_boom_ =
      nh.advertise<std_msgs::Float32>("/loadTraj_maxBoom", 1, true);
  pub_max_z_ = nh.advertise<std_msgs::Float32>("/loadTraj_maxZ", 1, true);
  pub_planning_pulse_ =
      nh.advertise<std_msgs::UInt32>("/UT_PlanningPulse_topic", 1);
}

void StatusReporter::ReportSwingForward(double swing_deg) {
  geometry_msgs::Point msg;
  msg.x = swing_deg;  // deg 原值转发，保持下游协议兼容
  msg.y = 0.0;
  msg.z = 0.0;
  pub_swing_.publish(msg);
}

void StatusReporter::ReportBucketPos(double x, double y, double z) {
  geometry_msgs::Point msg;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  pub_bucket_pos_.publish(msg);
}

void StatusReporter::ReportFinishFlag(double value) {
  std_msgs::Float64 msg;
  msg.data = value;
  pub_finish_flag_.publish(msg);
}

void StatusReporter::ReportFeasible(bool reachable) {
  std_msgs::Bool msg;
  msg.data = reachable;
  pub_up_feasible_.publish(msg);
}

void StatusReporter::ReportWaypointStats(double max_boom_deg, double max_z) {
  std_msgs::Float32 boom_msg;
  boom_msg.data = static_cast<float>(max_boom_deg);
  pub_max_boom_.publish(boom_msg);
  std_msgs::Float32 z_msg;
  z_msg.data = static_cast<float>(max_z);
  pub_max_z_.publish(z_msg);
}

void StatusReporter::ReportPulse(bool truck_ready, bool error_flag) {
  // 错误逻辑判定
  uint32_t error_code = 0;
  if (!truck_ready) {
    error_code = 1;  // 卡车位置无法卸载
  } else if (!error_flag) {
    error_code = 2;  // 轨迹规划算法失败
  }
  PublishPulse(error_code);
}

void StatusReporter::ReportExecutionAbort() {
  PublishPulse(3);  // 执行期中止：段超时未达标 / 门控 boost 耗尽 / boom 到限
}

void StatusReporter::PublishPulse(uint32_t error_code) {
  // 心跳累加（每次发布 +1，>255 回绕 0）
  ++heartbeat_cnt_;
  if (heartbeat_cnt_ > 255) heartbeat_cnt_ = 0;
  // 位拼接：heartbeat[31:16] | module_id[15:8] | error_code[7:0]
  constexpr uint32_t kModuleId = 2;  // 轨迹规划模块 ID
  std_msgs::UInt32 msg;
  msg.data = (heartbeat_cnt_ << 16) + (kModuleId << 8) + error_code;
  pub_planning_pulse_.publish(msg);
  ROS_INFO("[dump_traj] planning pulse: hb=%u module=%u err=%u (word=%u)",
           heartbeat_cnt_, kModuleId, error_code, msg.data);
}

}  // namespace dump_trajectory_planner
