#include "excavator_system_decision/scenarios/truck_interaction_scenario/dig_wait_fsm/dig_wait_fsm.hpp"

#include <cmath>

namespace excavator_system_decision {

void DigWaitFsm::init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context) {
  nh_ = nh;
  topics_ = topics;
  context_ = context;
  nh_.param("operation/boom_up_angle_threshold",
            config_.boom_up_angle_threshold,
            config_.boom_up_angle_threshold);
  nh_.param("operation/swing_angle_error_threshold",
            config_.swing_angle_error_threshold,
            config_.swing_angle_error_threshold);
  if (config_.swing_angle_error_threshold < 0.0) {
    ROS_WARN("[DigWaitFsm] swing_angle_error_threshold must be non-negative, using 20.0");
    config_.swing_angle_error_threshold = 20.0;
  }
}

void DigWaitFsm::start() {
  transitionTo(DigWaitFsmState::DIG_EXECUTE);
}

void DigWaitFsm::cancel() {
  if (context_) {
    context_->output.flag_dig_execute = 0.0;
  }
  state_ = DigWaitFsmState::IDLE;
}

bool DigWaitFsm::feedbackAfter(const ros::Time& feedback_time) const {
  return !feedback_time.isZero() && feedback_time > state_enter_time_;
}

bool DigWaitFsm::executionFeedbackReady() const {
  return (ros::Time::now() - state_enter_time_) >= ros::Duration(kExecutionFeedbackDelaySec);
}

bool DigWaitFsm::swingAngleWithinThreshold() const {
  if (!context_ || !context_->real_swing_angle_received || !context_->ref_swing_angle_received) {
    return false;
  }
  const double error = std::fabs(
      std::remainder(context_->ref_swing_angle - context_->real_swing_angle, 360.0));
  return error <= config_.swing_angle_error_threshold;
}

void DigWaitFsm::transitionTo(DigWaitFsmState next) {
  if (next == DigWaitFsmState::DIG_EXECUTE && context_) {
    context_->dig_execute_finished = false;
  }
  state_ = next;
  state_enter_time_ = ros::Time::now();
  ROS_INFO("[DigWaitFsm] -> %s", toString(state_));
}

void DigWaitFsm::update() {
  if (!context_ || state_ == DigWaitFsmState::IDLE || state_ == DigWaitFsmState::DONE) {
    return;
  }

  switch (state_) {
    case DigWaitFsmState::DIG_EXECUTE:
      context_->output.seq_action = 3.0;
      context_->output.flag_dig_execute = 1.0;
      if (executionFeedbackReady() && context_->dig_execute_finished) {
        context_->output.flag_dig_execute = 0.0;
        context_->output.seq_action = 3.5;
        context_->dig_execute_finished = false;
        transitionTo(DigWaitFsmState::BOOM_UP_PLAN);
      }
      break;

    case DigWaitFsmState::BOOM_UP_PLAN:
      context_->output.seq_action = 5.0;
      if (context_->real_joint_angles_received &&
          context_->real_boom_angle >= config_.boom_up_angle_threshold &&
          swingAngleWithinThreshold()) {
        context_->output.seq_action = 5.5;
        transitionTo(DigWaitFsmState::DONE);
      }
      break;

    case DigWaitFsmState::DONE:
    case DigWaitFsmState::IDLE:
      break;
  }
}

}  // namespace excavator_system_decision
