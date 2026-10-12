#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <string>

#include "excavator_system_decision/decision_context.hpp"

namespace excavator_system_decision {

struct TopicConfig {
  // Input: planning feedback
  std::string ground_plan_feedback;
  std::string unload_plan_feedback;

  // Input: execution feedback
  std::string reset_execute_feedback;
  std::string dig_execute_feedback;
  std::string unload_execute_feedback;
  std::string smooth_finish;
  std::string clean_finish;
  std::string walk_finish;

  // Input: status
  std::string truck_pose;
  std::string truck_full;
  std::string joint_angles;
  std::string real_swing_angle;
  std::string ref_swing_angle;
  std::string upper_lower_alignment_finished;
  std::string truck_center_unloadpoint;
  std::string unman_params_set;
  std::string mpc_switch2;

  // Output: planning command
  std::string ground_plan_command;
  std::string unload_plan_command;

  // Output: execution command
  std::string reset_execute_command;
  std::string dig_execute_command;
  std::string ground_level_execute_command;
  std::string unload_execute_command;

  // Output: status and coordination
  std::string loading_command;
  std::string seq_plan;
  std::string seq_action;
  std::string seq_truck;
  std::string fault_takeover;
  std::string bucket_number;
  std::string truck_load_number;
  std::string state_text;
};

class TopicInterface {
 public:
  void init(ros::NodeHandle nh, DecisionContext* context);
  void initDefaultTopics();
  void resetOutputs();
  void publishAllCommandsZero();
  void publishOutputs();

  const TopicConfig& config() const { return config_; }

 private:
  // Input: planning feedback
  void groundPlanCallback(const std_msgs::Float64::ConstPtr& msg);
  void unloadPlanCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

  // Input: execution feedback
  void resetExecuteCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void digExecuteCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void unloadExecuteCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void smoothCallback(const std_msgs::Bool::ConstPtr& msg);
  void cleanCallback(const std_msgs::Float32::ConstPtr& msg);
  void walkFinishCallback(const std_msgs::Bool::ConstPtr& msg);

  // Input: status
  void truckPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void fullCallback(const std_msgs::Bool::ConstPtr& msg);
  void jointAnglesCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void realSwingAngleCallback(const std_msgs::Float32::ConstPtr& msg);
  void refSwingAngleCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void upperLowerAlignmentFinishedCallback(const std_msgs::Bool::ConstPtr& msg);
  void truckCenterUnloadpointCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void unmanParamsSetCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void mpcSwitch2Callback(const std_msgs::Bool::ConstPtr& msg);

  ros::NodeHandle nh_;
  DecisionContext* context_{nullptr};
  TopicConfig config_;

  // Output: planning command
  ros::Publisher ground_plan_pub_;
  ros::Publisher unload_plan_pub_;

  // Output: execution command
  ros::Publisher reset_execute_pub_;
  ros::Publisher dig_execute_pub_;
  ros::Publisher ground_level_execute_pub_;
  ros::Publisher unload_execute_pub_;

  // Output: status and coordination
  ros::Publisher loading_command_pub_;
  ros::Publisher seq_plan_pub_;
  ros::Publisher seq_action_pub_;
  ros::Publisher seq_truck_pub_;
  ros::Publisher fault_takeover_pub_;
  ros::Publisher bucket_number_pub_;
  ros::Publisher truck_load_number_pub_;
  ros::Publisher state_text_pub_;

  // Input: planning feedback
  ros::Subscriber ground_plan_sub_;
  ros::Subscriber unload_plan_sub_;

  // Input: execution feedback
  ros::Subscriber reset_execute_sub_;
  ros::Subscriber dig_execute_sub_;
  ros::Subscriber unload_execute_sub_;
  ros::Subscriber smooth_finish_sub_;
  ros::Subscriber clean_finish_sub_;
  ros::Subscriber walk_finish_sub_;

  // Input: status
  ros::Subscriber truck_pose_sub_;
  ros::Subscriber truck_full_sub_;
  ros::Subscriber joint_angles_sub_;
  ros::Subscriber real_swing_angle_sub_;
  ros::Subscriber ref_swing_angle_sub_;
  ros::Subscriber upper_lower_alignment_finished_sub_;
  ros::Subscriber truck_center_unloadpoint_sub_;
  ros::Subscriber unman_params_set_sub_;
  ros::Subscriber mpc_switch2_sub_;
};

}  // namespace excavator_system_decision
