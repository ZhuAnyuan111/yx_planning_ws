#include "excavator_system_decision/topic_interface.hpp"

#include <cmath>
#include <cstddef>

namespace excavator_system_decision {

namespace {

// Input: planning feedback
constexpr const char* kGroundPlanFeedbackTopic = "/Sys_RGp_FlagGpPlanFinish";
constexpr const char* kUnloadPlanFeedbackTopic = "/Sys_RUn_UnloadPointSequence";

// Input: execution feedback
constexpr const char* kResetExecuteFeedbackTopic = "/RefDeviceTraj_Reset";
constexpr const char* kDigExecuteFeedbackTopic = "/RefDeviceTraj_Dig";
constexpr const char* kUnloadExecuteFeedbackTopic = "/RefDeviceTraj_Unload";
constexpr const char* kSmoothFinishTopic = "/FlagSmoothFinish";
constexpr const char* kCleanFinishTopic = "/FlagCleanFinish";
constexpr const char* kUpperLowerAlignmentFinishedTopic = "/FlagUpperLowerAlignmentFinish";
constexpr const char* kTruckCenterUnloadpointTopic = "/truck_center_unloadpoint";
constexpr const char* kWalkFinishTopic = "/FlagWalkFinish";

// Input: status
constexpr const char* kTruckPoseTopic = "/truck_pose_topic";
constexpr const char* kTruckFullTopic = "/FlagTruckFull";
constexpr const char* kJointAnglesTopic = "/joints_angle";
constexpr const char* kRealSwingAngleTopic = "/heading2swing_topic";
constexpr const char* kRefSwingAngleTopic = "/Sys_SCo_Traj";
constexpr const char* kUnmanParamsSetTopic = "/unman_params_set";
constexpr const char* kMpcSwitch2Topic = "/Mpc_Switch2";

// Output: planning command
constexpr const char* kGroundPlanCommandTopic = "/Sys_SGp_FlagGpPlan";
constexpr const char* kUnloadPlanCommandTopic = "/Sys_SUn_FlagUnloadPlan";

// Output: execution command
constexpr const char* kResetExecuteCommandTopic = "/Sys_SRe_FlagResetExcute";
constexpr const char* kDigExecuteCommandTopic = "/Sys_SDi_FlagDigExcute";
constexpr const char* kGroundLevelExecuteCommandTopic = "/Sys_SLe_FlagGroundLevelExcute";
constexpr const char* kUnloadExecuteCommandTopic = "/Sys_SUn_FlagUnloadExcute";

// Output: status and coordination
constexpr const char* kLoadingCommandTopic = "/digger_loading_topic";
constexpr const char* kSeqPlanTopic = "/Sys_SeqPlan";
constexpr const char* kSeqActionTopic = "/Sys_SeqAction";
constexpr const char* kSeqTruckTopic = "/Sys_SeqTruck";
constexpr const char* kFaultTakeoverTopic = "/fault_require_takeover";
constexpr const char* kBucketNumberTopic = "/Sys_SUn_BucketNumber";
constexpr const char* kTruckLoadNumberTopic = "/Sys_TruckLoadNumber";
constexpr const char* kStateTextTopic = "/system_decision/state_text";

constexpr std::size_t kUnloadTrajRows = 2000;
constexpr std::size_t kUnloadTrajCols = 5;
constexpr std::size_t kUnloadTrajSize = kUnloadTrajRows * kUnloadTrajCols;

}  // namespace

void TopicInterface::initDefaultTopics() {
  // Input: planning feedback
  config_.ground_plan_feedback = kGroundPlanFeedbackTopic;
  config_.unload_plan_feedback = kUnloadPlanFeedbackTopic;

  // Input: execution feedback
  config_.reset_execute_feedback = kResetExecuteFeedbackTopic;
  config_.dig_execute_feedback = kDigExecuteFeedbackTopic;
  config_.unload_execute_feedback = kUnloadExecuteFeedbackTopic;
  config_.smooth_finish = kSmoothFinishTopic;
  config_.clean_finish = kCleanFinishTopic;
  config_.walk_finish = kWalkFinishTopic;

  // Input: status
  config_.truck_pose = kTruckPoseTopic;
  config_.truck_full = kTruckFullTopic;
  config_.joint_angles = kJointAnglesTopic;
  config_.real_swing_angle = kRealSwingAngleTopic;
  config_.ref_swing_angle = kRefSwingAngleTopic;
  config_.upper_lower_alignment_finished = kUpperLowerAlignmentFinishedTopic;
  config_.truck_center_unloadpoint = kTruckCenterUnloadpointTopic;
  config_.unman_params_set = kUnmanParamsSetTopic;
  config_.mpc_switch2 = kMpcSwitch2Topic;

  // Output: planning command
  config_.ground_plan_command = kGroundPlanCommandTopic;
  config_.unload_plan_command = kUnloadPlanCommandTopic;

  // Output: execution command
  config_.reset_execute_command = kResetExecuteCommandTopic;
  config_.dig_execute_command = kDigExecuteCommandTopic;
  config_.ground_level_execute_command = kGroundLevelExecuteCommandTopic;
  config_.unload_execute_command = kUnloadExecuteCommandTopic;

  // Output: status and coordination
  config_.loading_command = kLoadingCommandTopic;
  config_.seq_plan = kSeqPlanTopic;
  config_.seq_action = kSeqActionTopic;
  config_.seq_truck = kSeqTruckTopic;
  config_.fault_takeover = kFaultTakeoverTopic;
  config_.bucket_number = kBucketNumberTopic;
  config_.truck_load_number = kTruckLoadNumberTopic;
  config_.state_text = kStateTextTopic;
}

void TopicInterface::init(ros::NodeHandle nh, DecisionContext* context) {
  nh_ = nh;
  context_ = context;
  initDefaultTopics();

  // Output: planning command
  ground_plan_pub_ = nh_.advertise<std_msgs::Float32>(config_.ground_plan_command, 1);
  unload_plan_pub_ = nh_.advertise<std_msgs::Float64>(config_.unload_plan_command, 1);

  // Output: execution command
  reset_execute_pub_ = nh_.advertise<std_msgs::Float64>(config_.reset_execute_command, 1);
  dig_execute_pub_ = nh_.advertise<std_msgs::Float64>(config_.dig_execute_command, 1);
  ground_level_execute_pub_ = nh_.advertise<std_msgs::Float64>(config_.ground_level_execute_command, 1);
  unload_execute_pub_ = nh_.advertise<std_msgs::Float64>(config_.unload_execute_command, 1);

  // Output: status and coordination
  loading_command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.loading_command, 1);
  seq_plan_pub_ = nh_.advertise<std_msgs::Float64>(config_.seq_plan, 1, true);
  seq_action_pub_ = nh_.advertise<std_msgs::Float64>(config_.seq_action, 1, true);
  seq_truck_pub_ = nh_.advertise<std_msgs::Float64>(config_.seq_truck, 1, true);
  fault_takeover_pub_ = nh_.advertise<std_msgs::Bool>(config_.fault_takeover, 1, true);
  bucket_number_pub_ = nh_.advertise<std_msgs::Float64>(config_.bucket_number, 1, true);
  truck_load_number_pub_ = nh_.advertise<std_msgs::Float32>(config_.truck_load_number, 1, true);
  state_text_pub_ = nh_.advertise<std_msgs::String>(config_.state_text, 1, true);

  // Input: planning feedback
  ground_plan_sub_ = nh_.subscribe(config_.ground_plan_feedback, 10, &TopicInterface::groundPlanCallback, this);
  unload_plan_sub_ = nh_.subscribe(config_.unload_plan_feedback, 10, &TopicInterface::unloadPlanCallback, this);

  // Input: execution feedback
  reset_execute_sub_ = nh_.subscribe(config_.reset_execute_feedback, 10, &TopicInterface::resetExecuteCallback, this);
  dig_execute_sub_ = nh_.subscribe(config_.dig_execute_feedback, 10, &TopicInterface::digExecuteCallback, this);
  unload_execute_sub_ = nh_.subscribe(config_.unload_execute_feedback, 10, &TopicInterface::unloadExecuteCallback, this);
  smooth_finish_sub_ = nh_.subscribe(config_.smooth_finish, 10, &TopicInterface::smoothCallback, this);
  clean_finish_sub_ = nh_.subscribe(config_.clean_finish, 10, &TopicInterface::cleanCallback, this);
  walk_finish_sub_ = nh_.subscribe(config_.walk_finish, 10, &TopicInterface::walkFinishCallback, this);

  // Input: status
  truck_pose_sub_ = nh_.subscribe(config_.truck_pose, 10, &TopicInterface::truckPoseCallback, this);
  truck_full_sub_ = nh_.subscribe(config_.truck_full, 10, &TopicInterface::fullCallback, this);
  joint_angles_sub_ = nh_.subscribe(config_.joint_angles, 10, &TopicInterface::jointAnglesCallback, this);
  real_swing_angle_sub_ = nh_.subscribe(config_.real_swing_angle, 10, &TopicInterface::realSwingAngleCallback, this);
  ref_swing_angle_sub_ = nh_.subscribe(config_.ref_swing_angle, 10, &TopicInterface::refSwingAngleCallback, this);
  upper_lower_alignment_finished_sub_ = nh_.subscribe(config_.upper_lower_alignment_finished, 10,
                                                       &TopicInterface::upperLowerAlignmentFinishedCallback, this);
  truck_center_unloadpoint_sub_ = nh_.subscribe(config_.truck_center_unloadpoint, 10,
                                                 &TopicInterface::truckCenterUnloadpointCallback, this);
  unman_params_set_sub_ = nh_.subscribe(config_.unman_params_set, 10, &TopicInterface::unmanParamsSetCallback, this);
  mpc_switch2_sub_ = nh_.subscribe(config_.mpc_switch2, 10,
                                    &TopicInterface::mpcSwitch2Callback, this);

  resetOutputs();
}

void TopicInterface::resetOutputs() {
  if (context_) {
    context_->output.resetAll();
  }
}

void TopicInterface::publishAllCommandsZero() {
  if (!context_) return;

  context_->output.resetCommands();
  context_->output.seq_plan = 0.0;
  context_->output.seq_action = 1.0;
  publishOutputs();
}

void TopicInterface::publishOutputs() {
  if (!context_) return;

  // Output: planning command
  std_msgs::Float32 ground_plan; ground_plan.data = static_cast<float>(context_->output.flag_ground_plan); ground_plan_pub_.publish(ground_plan);
  std_msgs::Float64 msg64;
  msg64.data = context_->output.flag_unload_plan; unload_plan_pub_.publish(msg64);

  // Output: execution command
  msg64.data = context_->output.flag_reset_execute; reset_execute_pub_.publish(msg64);
  msg64.data = context_->output.flag_dig_execute; dig_execute_pub_.publish(msg64);
  msg64.data = context_->output.flag_ground_level_execute; ground_level_execute_pub_.publish(msg64);
  msg64.data = context_->output.flag_unload_execute; unload_execute_pub_.publish(msg64);

  // Output: status and coordination
  context_->output.digger_loading.header.stamp = ros::Time::now();
  loading_command_pub_.publish(context_->output.digger_loading);

  msg64.data = context_->output.seq_plan; seq_plan_pub_.publish(msg64);
  msg64.data = context_->output.seq_action; seq_action_pub_.publish(msg64);
  msg64.data = context_->output.seq_truck; seq_truck_pub_.publish(msg64);
  std_msgs::Bool fault_takeover;
  fault_takeover.data = context_->output.fault_require_takeover;
  fault_takeover_pub_.publish(fault_takeover);

  msg64.data = context_->bucket_number; bucket_number_pub_.publish(msg64);
  std_msgs::Float32 load; load.data = static_cast<float>(context_->truck_number); truck_load_number_pub_.publish(load);
  std_msgs::String text; text.data = context_->output.state_text; state_text_pub_.publish(text);
}

// Input: planning feedback
void TopicInterface::groundPlanCallback(const std_msgs::Float64::ConstPtr& msg) {
  if (msg->data == 0.0) return;
  context_->ground_plan_feedback_time = ros::Time::now();
}

void TopicInterface::unloadPlanCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (!context_ || msg->data.size() < kUnloadTrajSize) return;
  context_->unload_plan_finish_flag = static_cast<double>(msg->data[0]);

  if (context_->unload_plan_finish_flag != 0.0) {
    context_->unload_plan_feedback_time = ros::Time::now();
  }
}

// Input: execution feedback
void TopicInterface::resetExecuteCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  if (!context_) return;
  if (msg->position.x != 0.0) return;

  context_->reset_execute_finished = true;
}

void TopicInterface::digExecuteCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  if (!context_) return;
  if (msg->position.x != 0.0) return;

  context_->dig_execute_finished = true;
}

void TopicInterface::unloadExecuteCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  if (!context_) return;
  if (msg->position.x != 0.0) return;

  context_->unload_execute_finished = true;
}

void TopicInterface::smoothCallback(const std_msgs::Bool::ConstPtr& msg) {
  context_->smooth_finished = msg->data;
}

void TopicInterface::cleanCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (!context_) return;
  context_->clean_finish_flag = msg->data;
}

void TopicInterface::walkFinishCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!context_ || !msg->data) return;
  context_->walk_finished = true;
  context_->walk_finish_time = ros::Time::now();
}

// Input: status
void TopicInterface::truckPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  context_->truck_event_code = msg->pose.orientation.w;
  context_->truck_event_time = ros::Time::now();
}

void TopicInterface::fullCallback(const std_msgs::Bool::ConstPtr& msg) {
  context_->truck_full = msg->data;
}

void TopicInterface::jointAnglesCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
  context_->real_boom_angle = msg->x;
  context_->real_arm_angle = msg->y;
  context_->real_bucket_angle = msg->z;
  context_->real_joint_angles_time = ros::Time::now();
  context_->real_joint_angles_received = true;
}

void TopicInterface::realSwingAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
  context_->real_swing_angle = msg->data;
  context_->real_swing_angle_time = ros::Time::now();
  context_->real_swing_angle_received = true;
}

void TopicInterface::refSwingAngleCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  if (!context_) return;
  context_->ref_swing_angle = msg->orientation.x;
  context_->ref_swing_angle_time = ros::Time::now();
  context_->ref_swing_angle_received = true;
}

void TopicInterface::upperLowerAlignmentFinishedCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!context_) return;
  context_->upper_lower_alignment_finished = msg->data;
  if (msg->data) {
    context_->upper_lower_alignment_finish_time = ros::Time::now();
  }
}

void TopicInterface::truckCenterUnloadpointCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  if (!context_) return;

  // Match the specified MATLAB function: atan2d(-y, -x), normalized to 0..360.
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  double theta = std::atan2(-msg->y, -msg->x) * kRadToDeg;
  if (theta < 0.0) theta += 360.0;

  context_->truck_center_unloadpoint_angle = theta;
  context_->truck_center_unloadpoint_angle_time = ros::Time::now();
  context_->truck_center_unloadpoint_angle_received = true;
}

void TopicInterface::unmanParamsSetCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  if (!context_) return;

  const bool previous_is_one = context_->unman_params_received &&
                               context_->unman_params_orientation_w == 1.0;
  const bool current_is_one = msg->orientation.w == 1.0;
  if (!previous_is_one && current_is_one) {
    context_->resetBucketNumber();
    ROS_INFO("[TopicInterface] Bucket number reset by /unman_params_set orientation.w rising edge");
  }

  context_->unman_params_orientation_w = msg->orientation.w;
  context_->unman_params_received = true;
}

void TopicInterface::mpcSwitch2Callback(const std_msgs::Bool::ConstPtr& msg) {
  if (!context_) return;

  // /Mpc_Switch2 is a level signal: true means staging is requested and
  // false means staging is not requested. Do not latch or edge-detect it.
  context_->staging_requested = msg->data;
  ROS_INFO("[TopicInterface] /Mpc_Switch2=%s; staging_requested updated",
           msg->data ? "true" : "false");
}

}  // namespace excavator_system_decision
