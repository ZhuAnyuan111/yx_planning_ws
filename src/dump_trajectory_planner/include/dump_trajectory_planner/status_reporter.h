#pragma once

/// @file status_reporter.h
/// @brief 状态上报器：统一管理全部状态/统计/适配类话题发布。
///
/// 承载除执行指令流（/RefDeviceTraj_Dump）之外的全部发布出口：
///   /Swing_topic                    swing 转发（下游协议兼容）
///   /RealBktPosXYZ                  FK 铲斗齿尖坐标
///   /Sys_RUn_FlagUnloadExcuteFinish 完成标志（latched）
///   /UPFeasible                     卸载点可达性（latched）
///   /loadTraj_maxBoom | maxZ        航路点统计量（latched）
///   /UT_PlanningPulse_topic         功能安全状态字（心跳|模块ID|错误码）

#include <cstdint>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

namespace dump_trajectory_planner {

class StatusReporter {
 public:
  /// 构造时完成全部话题 advertise（latched 属性内置）
  explicit StatusReporter(ros::NodeHandle& nh);

  /// 转发 swing 角（deg）为 /Swing_topic（Point.x，y/z=0）
  void ReportSwingForward(double swing_deg);

  /// 发布 FK 铲斗齿尖坐标 (base 系, m)
  void ReportBucketPos(double x, double y, double z);

  /// 发布完成标志：0.0=执行中 / 1.0=结束或空闲
  void ReportFinishFlag(double value);

  /// 发布卸载点可达性校验结果
  void ReportFeasible(bool reachable);

  /// 发布航路点统计量：动臂最大角度 (deg) / 齿尖最高 z (m)
  void ReportWaypointStats(double max_boom_deg, double max_z);

  /// 发布功能安全状态字（心跳+模块ID+错误码位拼接）
  /// @param truck_ready  卸载点可达（CheckReachable 结果）
  /// @param error_flag   轨迹规划成功（WP4 可达性 / 限位校验通过）
  void ReportPulse(bool truck_ready, bool error_flag);

  /// 发布执行期中止状态字（error_code=3：段执行/门控失败等运行期异常中止，
  /// 用于与正常完成 FinishFlag=1.0 区分，避免上位机误判卸载成功）
  void ReportExecutionAbort();

 private:
  /// 位拼接并发布 /UT_PlanningPulse_topic（heartbeat|module|error_code）
  void PublishPulse(uint32_t error_code);

  ros::Publisher pub_swing_;        // /Swing_topic 转发
  ros::Publisher pub_bucket_pos_;   // /RealBktPosXYZ（FK 计算）
  ros::Publisher pub_finish_flag_;  // /Sys_RUn_FlagUnloadExcuteFinish (latched)
  ros::Publisher pub_up_feasible_;  // /UPFeasible (latched)
  ros::Publisher pub_max_boom_;     // /loadTraj_maxBoom (latched)
  ros::Publisher pub_max_z_;        // /loadTraj_maxZ (latched)
  ros::Publisher pub_planning_pulse_;  // /UT_PlanningPulse_topic

  // 心跳计数（persistent，跨激活保持，0-255 循环）
  uint32_t heartbeat_cnt_ = 0;
};

}  // namespace dump_trajectory_planner
