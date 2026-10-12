#include "excavator_system_decision/scenarios/truck_interaction_scenario/staging_cycle_fsm/staging_cycle_fsm.hpp"

#include <cmath>

namespace excavator_system_decision {

void StagingCycleFsm::init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context) {
  nh_ = nh;
  topics_ = topics;
  context_ = context;
  nh_.param("operation/boom_up_angle_threshold", boom_up_angle_threshold_, boom_up_angle_threshold_);
  nh_.param("operation/swing_angle_error_threshold",
            swing_angle_error_threshold_,
            swing_angle_error_threshold_);
  if (swing_angle_error_threshold_ < 0.0) {
    ROS_WARN("[StagingCycleFsm] swing_angle_error_threshold must be non-negative, using 20.0");
    swing_angle_error_threshold_ = 20.0;
  }
}

void StagingCycleFsm::start() {
  if (context_) context_->output.resetCommands();
  initial_unload_execute_done_ = false;
  reset_execute_done_ = false;
  dig_execute_done_ = false;
  unload_plan_done_ = false;
  unload_execute_done_ = false;
  final_dig_execute_done_ = false;
  transitionTo(StagingState::INITIAL_DELAY);
}

void StagingCycleFsm::cancel() {
  if (context_) context_->output.resetCommands();
  initial_unload_execute_done_ = false;
  reset_execute_done_ = false;
  dig_execute_done_ = false;
  unload_plan_done_ = false;
  unload_execute_done_ = false;
  final_dig_execute_done_ = false;
  state_ = StagingState::IDLE;
}

void StagingCycleFsm::transitionTo(StagingState next) {
  state_ = next;
  state_enter_time_ = ros::Time::now();

  if (next == StagingState::INITIAL_UNLOAD_EXECUTE) {
    initial_unload_execute_done_ = false;
    if (context_) context_->unload_execute_finished = false;
  } else if (next == StagingState::GROUND_LEVEL_EXECUTE) {
    // Only feedback received while phase 7 is active may select its exit path.
    if (context_) context_->clean_finish_flag = 0.0;
  } else if (next == StagingState::RESET_EXECUTE) {
    reset_execute_done_ = false;
    if (context_) context_->reset_execute_finished = false;
  } else if (next == StagingState::DIG_EXECUTE) {
    dig_execute_done_ = false;
    if (context_) context_->dig_execute_finished = false;
    unload_plan_done_ = false;
  } else if (next == StagingState::UNLOAD_EXECUTE) {
    unload_execute_done_ = false;
    if (context_) context_->unload_execute_finished = false;
  } else if (next == StagingState::FINAL_DIG_EXECUTE) {
    final_dig_execute_done_ = false;
    if (context_) context_->dig_execute_finished = false;
  }

  ROS_INFO("[StagingCycleFsm] -> %s", toString(state_));
}

bool StagingCycleFsm::feedbackAfter(const ros::Time& feedback_time) const {
  return !feedback_time.isZero() && feedback_time > state_enter_time_;
}

bool StagingCycleFsm::executionFeedbackReady() const {
  return (ros::Time::now() - state_enter_time_) >= ros::Duration(kExecutionFeedbackDelaySec);
}

bool StagingCycleFsm::swingAngleWithinThreshold() const {
  if (!context_ || !context_->real_swing_angle_received ||
      !context_->ref_swing_angle_received) {
    return false;
  }
  const double error = std::fabs(
      std::remainder(context_->ref_swing_angle - context_->real_swing_angle, 360.0));
  return error <= swing_angle_error_threshold_;
}

void StagingCycleFsm::update() {
  if (!context_ || state_ == StagingState::IDLE) return;

  switch (state_) {
    case StagingState::INITIAL_DELAY:
      if (executionFeedbackReady()) {
        transitionTo(StagingState::INITIAL_UNLOAD_PLAN);
      }
      break;

    case StagingState::INITIAL_UNLOAD_PLAN:
      context_->output.seq_plan = 4.0;
      context_->output.flag_unload_plan = 1.0;
      if (feedbackAfter(context_->unload_plan_feedback_time)) {
        context_->output.flag_unload_plan = 0.0;
        context_->output.seq_plan = 4.5;
        transitionTo(StagingState::INITIAL_UNLOAD_EXECUTE);
      }
      break;

    case StagingState::INITIAL_UNLOAD_EXECUTE:
      context_->output.seq_action = 4.0;
      context_->output.flag_unload_execute = 1.0;
      if (executionFeedbackReady() && context_->unload_execute_finished) {
        context_->output.flag_unload_execute = 0.0;
        context_->output.seq_action = 4.5;
        context_->unload_execute_finished = false;
        initial_unload_execute_done_ = true;
        ++context_->bucket_number;
      }
      if (initial_unload_execute_done_) {
        transitionTo(StagingState::RESET_EXECUTE);
      }
      break;

    case StagingState::RESET_EXECUTE:
      context_->output.seq_action = 2.0;
      context_->output.flag_reset_execute = 1.0;
      if (executionFeedbackReady() && context_->reset_execute_finished) {
        context_->output.flag_reset_execute = 0.0;
        context_->output.seq_action = 2.5;
        context_->reset_execute_finished = false;
        reset_execute_done_ = true;
      }
      if (reset_execute_done_) transitionTo(StagingState::GROUND_LEVEL_EXECUTE);
      break;

    case StagingState::GROUND_LEVEL_EXECUTE:
      context_->output.seq_action = 7.0;
      context_->output.flag_ground_level_execute = 1.0;
      if (context_->clean_finish_flag == 1.0) {
        context_->output.flag_ground_level_execute = 0.0;
        context_->output.seq_action = 7.5;
        context_->clean_finish_flag = 0.0;
        transitionTo(StagingState::DIG_EXECUTE);
      } else if (context_->clean_finish_flag == 2.0) {
        context_->output.flag_ground_level_execute = 0.0;
        context_->output.seq_action = 7.5;
        context_->clean_finish_flag = 0.0;
        transitionTo(StagingState::FINAL_DIG_EXECUTE);
      }
      break;

    case StagingState::DIG_EXECUTE:
      if (!dig_execute_done_) {
        context_->output.seq_action = 3.0;
        context_->output.flag_dig_execute = 1.0;
      }
      if (!unload_plan_done_) {
        context_->output.seq_plan = 4.0;
        context_->output.flag_unload_plan = 1.0;
      }
      if (!dig_execute_done_ && executionFeedbackReady() && context_->dig_execute_finished) {
        context_->output.flag_dig_execute = 0.0;
        context_->output.seq_action = 3.5;
        context_->dig_execute_finished = false;
        dig_execute_done_ = true;
      }
      if (!unload_plan_done_ && feedbackAfter(context_->unload_plan_feedback_time)) {
        context_->output.flag_unload_plan = 0.0;
        context_->output.seq_plan = 4.5;
        unload_plan_done_ = true;
      }
      if (dig_execute_done_ && unload_plan_done_) transitionTo(StagingState::UNLOAD_EXECUTE);
      break;

    case StagingState::UNLOAD_EXECUTE:
      if (!unload_execute_done_) {
        context_->output.seq_action = 4.0;
        context_->output.flag_unload_execute = 1.0;
        if (executionFeedbackReady() && context_->unload_execute_finished) {
          context_->output.flag_unload_execute = 0.0;
          context_->output.seq_action = 4.5;
          context_->unload_execute_finished = false;
          unload_execute_done_ = true;
          ++context_->bucket_number;
        }
      }
      if (unload_execute_done_) {
        transitionTo(StagingState::RESET_EXECUTE);
      }
      break;

    case StagingState::FINAL_DIG_EXECUTE:
      context_->output.seq_action = 3.0;
      context_->output.flag_dig_execute = 1.0;
      if (executionFeedbackReady() && context_->dig_execute_finished) {
        context_->output.flag_dig_execute = 0.0;
        context_->output.seq_action = 3.5;
        context_->dig_execute_finished = false;
        final_dig_execute_done_ = true;
      }
      if (final_dig_execute_done_) {
        transitionTo(StagingState::FINAL_BOOM_UP_WAIT);
      }
      break;

    case StagingState::FINAL_BOOM_UP_WAIT:
      context_->output.seq_action = 5.0;
      if (context_->real_joint_angles_received &&
          context_->real_boom_angle >= boom_up_angle_threshold_ &&
          swingAngleWithinThreshold()) {
        context_->output.seq_action = 5.5;
        transitionTo(StagingState::SUCCESS);
      }
      break;

    case StagingState::SUCCESS:
    case StagingState::IDLE:
      break;
  }
}

}  // namespace excavator_system_decision
