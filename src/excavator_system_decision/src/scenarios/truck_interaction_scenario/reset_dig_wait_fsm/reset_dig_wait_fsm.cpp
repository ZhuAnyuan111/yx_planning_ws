#include "excavator_system_decision/scenarios/truck_interaction_scenario/reset_dig_wait_fsm/reset_dig_wait_fsm.hpp"

#include <cmath>

namespace excavator_system_decision {

void ResetDigWaitFsm::init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context) {
  nh_ = nh;
  topics_ = topics;
  context_ = context;
  nh_.param("operation/boom_up_angle_threshold", config_.boom_up_angle_threshold, config_.boom_up_angle_threshold);
  nh_.param("operation/swing_angle_error_threshold",
            config_.swing_angle_error_threshold,
            config_.swing_angle_error_threshold);
  if (config_.swing_angle_error_threshold < 0.0) {
    ROS_WARN("[ResetDigWaitFsm] swing_angle_error_threshold must be non-negative, using 20.0");
    config_.swing_angle_error_threshold = 20.0;
  }
}

void ResetDigWaitFsm::start() {
  if (context_) {
    context_->output.resetCommands();
    context_->output.seq_plan = 0.0;
  }
  transitionTo(ResetDigWaitFsmState::DIG_POINT_SELECTION);
}

void ResetDigWaitFsm::cancel() {
  if (context_) {
    context_->output.resetCommands();
    context_->output.seq_plan = 0.0;
  }
  state_ = ResetDigWaitFsmState::IDLE;
}

bool ResetDigWaitFsm::feedbackAfter(const ros::Time& feedback_time) const {
  return !feedback_time.isZero() && feedback_time > state_enter_time_;
}

bool ResetDigWaitFsm::executionFeedbackReady() const {
  return (ros::Time::now() - state_enter_time_) >= ros::Duration(kExecutionFeedbackDelaySec);
}

bool ResetDigWaitFsm::swingAngleWithinThreshold() const {
  if (!context_ || !context_->real_swing_angle_received ||
      !context_->ref_swing_angle_received) {
    return false;
  }
  const double error = std::fabs(
      std::remainder(context_->ref_swing_angle - context_->real_swing_angle, 360.0));
  return error <= config_.swing_angle_error_threshold;
}

void ResetDigWaitFsm::transitionTo(ResetDigWaitFsmState next) {
  if (context_) {
    if (next == ResetDigWaitFsmState::RESET_EXECUTE) {
      context_->reset_execute_finished = false;
    } else if (next == ResetDigWaitFsmState::DIG_EXECUTE) {
      context_->dig_execute_finished = false;
    }
  }
  state_ = next;
  state_enter_time_ = ros::Time::now();
  ROS_INFO("[ResetDigWaitFsm] -> %s", toString(state_));
}

void ResetDigWaitFsm::update() {
  if (!context_ || state_ == ResetDigWaitFsmState::IDLE || state_ == ResetDigWaitFsmState::DONE) {
    return;
  }

  switch (state_) {
    case ResetDigWaitFsmState::DIG_POINT_SELECTION:
      context_->output.seq_plan = 1.0;
      context_->output.flag_ground_plan = 1.0;
      if (feedbackAfter(context_->ground_plan_feedback_time)) {
        context_->output.flag_ground_plan = 0.0;
        context_->output.seq_plan = 1.5;
        transitionTo(ResetDigWaitFsmState::RESET_EXECUTE);
      }
      break;

    case ResetDigWaitFsmState::RESET_EXECUTE:
      context_->output.seq_action = 2.0;
      context_->output.flag_reset_execute = 1.0;
      if (executionFeedbackReady() && context_->reset_execute_finished) {
        context_->output.flag_reset_execute = 0.0;
        context_->output.seq_action = 2.5;
        context_->reset_execute_finished = false;
        transitionTo(ResetDigWaitFsmState::DIG_EXECUTE);
      }
      break;

    case ResetDigWaitFsmState::DIG_EXECUTE:
      context_->output.seq_action = 3.0;
      context_->output.flag_dig_execute = 1.0;
      if (executionFeedbackReady() && context_->dig_execute_finished) {
        context_->output.flag_dig_execute = 0.0;
        context_->output.seq_action = 3.5;
        context_->dig_execute_finished = false;
        transitionTo(ResetDigWaitFsmState::BOOM_UP_EXECUTE);
      }
      break;

    case ResetDigWaitFsmState::BOOM_UP_EXECUTE:
      context_->output.seq_action = 5.0;
      if (context_->real_joint_angles_received &&
          context_->real_boom_angle >= config_.boom_up_angle_threshold &&
          swingAngleWithinThreshold()) {
        context_->output.seq_action = 5.5;
        transitionTo(ResetDigWaitFsmState::DONE);
      }
      break;

    case ResetDigWaitFsmState::DONE:
    case ResetDigWaitFsmState::IDLE:
      break;
  }
}

}  // namespace excavator_system_decision
