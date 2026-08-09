#ifndef TRUCK_DUMP_PLANNER_PLANNER_NODE_H
#define TRUCK_DUMP_PLANNER_PLANNER_NODE_H

#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include "truck_dump_planner/coords.h"
#include "truck_dump_planner/truck_model.h"
#include "truck_dump_planner/visualization.h"
#include "truck_dump_planner/dig_planner.h"
#include "truck_dump_planner/reset_planner.h"

namespace truck_dump_planner {

/// 规划阶段状态（由系统决策节点激活）
enum class PlannerState {
  kIdle,    // 空闲等待激活
  kDig,     // 挖掘规划中
  kDump,    // 卸载规划中
  kReset    // 复位规划中
};

class PlannerNode {
 public:
  PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void LoadParams();
  void PoseCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void JointsCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void SwingCallback(const geometry_msgs::Point::ConstPtr& msg);
  void ActivateCallback(const std_msgs::String::ConstPtr& msg);
  void TimerCallback(const ros::TimerEvent& event);

  void TransitionTo(PlannerState new_state);
  void PublishFeedback(const std::string& status);
  Vec3 GetBucketPosition() const;

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  // 订阅
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_joints_;
  ros::Subscriber sub_swing_;
  ros::Subscriber sub_activate_;   // 系统决策节点 → 本节点：阶段激活信号
  // 发布
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_feedback_;    // 本节点 → 系统决策节点：规划完成/失败反馈
  ros::Timer timer_;

  // 状态机
  PlannerState state_ = PlannerState::kIdle;
  std::vector<Vec3> current_trajectory_;

  // 参数
  TruckParams p_;
  DigPlannerParams dig_params_;
  ResetPlannerParams reset_params_;
  Vec2 excavator_xy_{0.0, 0.0};
  std::string frame_id_ = "base";
  AxisConvention conv_ = AxisConvention::kNorthWest;

  // 关节角（度）
  double swing_deg_ = 0.0;
  double boom_deg_ = 0.0;
  double arm_deg_ = 0.0;
  double bucket_deg_ = 0.0;

  // 卡车包络缓存
  TruckEnvelope last_env_{};
  bool env_valid_ = false;

  std::unique_ptr<EnvelopeVisualizer> viz_;
};

}  // namespace truck_dump_planner

#endif  // TRUCK_DUMP_PLANNER_PLANNER_NODE_H

