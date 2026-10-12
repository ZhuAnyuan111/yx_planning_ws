#pragma once

#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>

namespace excavator_system_decision {

constexpr double kExecutionFeedbackDelaySec = 2.0;

struct DecisionOutputs {
  double seq_plan{0.0};
  double seq_action{1.0};
  double seq_truck{0.0};
  bool fault_require_takeover{false};

  double flag_ground_plan{0.0};
  double flag_reset_execute{0.0};
  double flag_dig_execute{0.0};
  double flag_ground_level_execute{0.0};
  double flag_unload_plan{0.0};
  double flag_unload_execute{0.0};

  geometry_msgs::PoseStamped digger_loading;
  std::string state_text{"IDLE:IDLE"};

  void resetCommands() {
    fault_require_takeover = false;
    flag_ground_plan = 0.0;
    flag_reset_execute = 0.0;
    flag_dig_execute = 0.0;
    flag_ground_level_execute = 0.0;
    flag_unload_plan = 0.0;
    flag_unload_execute = 0.0;
    digger_loading.pose.orientation.w = 0.0;
  }

  void resetAll() {
    seq_plan = 0.0;
    seq_action = 1.0;
    seq_truck = 0.0;
    fault_require_takeover = false;
    state_text = "IDLE:IDLE";
    resetCommands();
  }
};

struct DecisionContext {
  void resetBucketNumber() {
    bucket_number = 0;
  }

  void initializeForProgramStart() {
    // Reset counters only when the process starts.
    truck_number = 0;
    resetBucketNumber();
    output.resetAll();
  }

  double truck_event_code{0.0};
  ros::Time truck_event_time;

  bool truck_full{false};
  bool smooth_finished{false};
  double clean_finish_flag{0.0};
  bool staging_requested{false};
  bool walk_finished{false};
  ros::Time walk_finish_time;

  double real_boom_angle{0.0};
  double real_arm_angle{0.0};
  double real_bucket_angle{0.0};
  ros::Time real_joint_angles_time;
  bool real_joint_angles_received{false};
  double real_swing_angle{0.0};
  ros::Time real_swing_angle_time;
  bool real_swing_angle_received{false};

  // Current angle calculated from /truck_center_unloadpoint (degrees, 0..360).
  double truck_center_unloadpoint_angle{0.0};
  ros::Time truck_center_unloadpoint_angle_time;
  bool truck_center_unloadpoint_angle_received{false};

  double ref_swing_angle{0.0};
  ros::Time ref_swing_angle_time;
  bool ref_swing_angle_received{false};

  // External completion feedback for the upper/lower carriage alignment.
  // This is used only by StagingPreMoveFsm::SWING_RESET.
  bool upper_lower_alignment_finished{false};
  ros::Time upper_lower_alignment_finish_time;

  double unman_params_orientation_w{0.0};
  bool unman_params_received{false};

  ros::Time ground_plan_feedback_time;
  bool reset_execute_finished{false};
  bool dig_execute_finished{false};
  ros::Time unload_plan_feedback_time;
  double unload_plan_finish_flag{0.0};
  bool unload_execute_finished{false};

  // Keep truck_number for the whole process lifetime.
  int truck_number{0};
  // Reset only at program start, new truck entry, leveling completion, staging
  // scenario exit, or the /unman_params_set rising edge.
  int bucket_number{0};
  DecisionOutputs output;
};

}  // namespace excavator_system_decision
