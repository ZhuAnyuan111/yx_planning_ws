#include "excavator_system_decision/scenarios/truck_interaction_scenario/digging_cycle_fsm/digging_cycle_fsm.hpp"

namespace excavator_system_decision {

void DiggingCycleFsm::init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context) {
  nh_ = nh;
  topics_ = topics;
  context_ = context;
  nh_.param("thresholds/bucket_numbers_threshold_low", config_.min_bucket_count, config_.min_bucket_count);
  nh_.param("thresholds/bucket_numbers_threshold_high", config_.max_bucket_count, config_.max_bucket_count);
  nh_.param("operation/boom_up_angle_threshold", config_.boom_up_angle_threshold, config_.boom_up_angle_threshold);
  nh_.param("operation/level_truck_timeout_sec",
            config_.level_truck_timeout_sec,
            config_.level_truck_timeout_sec);

  if (config_.min_bucket_count < 1) {
    ROS_WARN("[DiggingCycleFsm] bucket_numbers_threshold_low must be >= 1, using 1");
    config_.min_bucket_count = 1;
  }
  if (config_.max_bucket_count < config_.min_bucket_count) {
    ROS_WARN("[DiggingCycleFsm] bucket_numbers_threshold_high must be >= "
             "bucket_numbers_threshold_low, using %d",
             config_.min_bucket_count);
    config_.max_bucket_count = config_.min_bucket_count;
  }
  if (config_.level_truck_timeout_sec <= 0.0) {
    ROS_WARN("[DiggingCycleFsm] level_truck_timeout_sec must be > 0, using 15.0");
    config_.level_truck_timeout_sec = 15.0;
  }
}

void DiggingCycleFsm::start() {
  context_->truck_full = false;
  context_->smooth_finished = false;
  unload_done_ = false;
  dig_execute_done_ = false;
  unload_plan_done_ = false;
  plan_phase_ = PlanPhase::DIG_POINT_SELECTION;
  transitionTo(OperationState::UNLOAD_PLAN);
}

void DiggingCycleFsm::cancel() {
  if (context_) context_->output.resetCommands();
  unload_done_ = false;
  dig_execute_done_ = false;
  unload_plan_done_ = false;
  plan_phase_ = PlanPhase::DIG_POINT_SELECTION;
  state_ = OperationState::IDLE;
}

bool DiggingCycleFsm::finished() const {
  return state_ == OperationState::SUCCESS;
}

bool DiggingCycleFsm::feedbackAfter(const ros::Time& feedback_time) const {
  return !feedback_time.isZero() && feedback_time > state_enter_time_;
}

bool DiggingCycleFsm::executionFeedbackReady() const {
  return (ros::Time::now() - state_enter_time_) >= ros::Duration(kExecutionFeedbackDelaySec);
}

void DiggingCycleFsm::transitionTo(OperationState next) {
  if (next == OperationState::UNLOAD_EXECUTE) {
    unload_done_ = false;
    if (context_) context_->unload_execute_finished = false;
    plan_phase_ = PlanPhase::DIG_POINT_SELECTION;
  }
  if (next == OperationState::DIG_EXECUTE) {
    dig_execute_done_ = false;
    if (context_) context_->dig_execute_finished = false;
    unload_plan_done_ = false;
  }
  if (next == OperationState::RESET_EXECUTE && context_) {
    context_->reset_execute_finished = false;
  }
  state_ = next;
  state_enter_time_ = ros::Time::now();
  ROS_INFO("[DiggingCycleFsm] -> %s", toString(state_));
}

void DiggingCycleFsm::update() {
  if (!context_ || state_ == OperationState::IDLE) return;

  switch (state_) {
    case OperationState::UNLOAD_PLAN:
      context_->output.seq_plan = 4.0;
      context_->output.flag_unload_plan = 1.0;
      if (feedbackAfter(context_->unload_plan_feedback_time)) {
        context_->output.flag_unload_plan = 0.0;
        context_->output.seq_plan = 4.5;
        transitionTo(OperationState::UNLOAD_EXECUTE);
      }
      break;

    case OperationState::UNLOAD_EXECUTE:
      // 左线：卸载执行
      if (!unload_done_) {
        context_->output.seq_action = 4.0;
        context_->output.flag_unload_execute = 1.0;

        if (executionFeedbackReady() && context_->unload_execute_finished) {
          context_->output.flag_unload_execute = 0.0;
          context_->unload_execute_finished = false;
          unload_done_ = true;
          ++context_->bucket_number;
          context_->output.seq_action = 4.5;
        }
      }

      // 右线：三个阶段串行
      if (plan_phase_ == PlanPhase::DIG_POINT_SELECTION) {
        context_->output.seq_plan = 1.0;
        context_->output.flag_ground_plan = 1.0;

        if (feedbackAfter(context_->ground_plan_feedback_time)) {
          context_->output.flag_ground_plan = 0.0;
          plan_phase_ = PlanPhase::DONE;
          context_->output.seq_plan = 1.5;
        }
      }

      if (unload_done_ &&
          ((context_->truck_full &&
            context_->bucket_number >= config_.min_bucket_count) ||
           context_->bucket_number >= config_.max_bucket_count)) {
        context_->output.seq_plan = 0.0;
        context_->output.flag_ground_plan = 0.0;
        transitionTo(OperationState::LEVEL_TRUCK);
      } else if (unload_done_ &&
                 plan_phase_ == PlanPhase::DONE) {
        transitionTo(OperationState::RESET_EXECUTE);
      }
      break;

    case OperationState::RESET_EXECUTE:
      context_->output.seq_action = 2.0;
      context_->output.flag_reset_execute = 1.0;

      if (executionFeedbackReady() && context_->reset_execute_finished) {
        context_->output.flag_reset_execute = 0.0;
        context_->reset_execute_finished = false;
        context_->output.seq_action = 2.5;
        transitionTo(OperationState::DIG_EXECUTE);
      }
      break;

    case OperationState::DIG_EXECUTE:
      if (!dig_execute_done_) {
        context_->output.seq_action = 3.0;
        context_->output.flag_dig_execute = 1.0;
      }

      // 与 DIG_EXECUTE 并行执行 unload_plan
      if (!unload_plan_done_) {
        context_->output.seq_plan = 4.0;
        context_->output.flag_unload_plan = 1.0;
      }

      if (!dig_execute_done_ && executionFeedbackReady() &&
          context_->dig_execute_finished) {
        context_->output.flag_dig_execute = 0.0;
        context_->dig_execute_finished = false;
        dig_execute_done_ = true;
        context_->output.seq_action = 3.5;
      }

      if (!unload_plan_done_ &&
          feedbackAfter(context_->unload_plan_feedback_time)) {
        context_->output.flag_unload_plan = 0.0;
        unload_plan_done_ = true;
        context_->output.seq_plan = 4.5;
      }

      if (dig_execute_done_ && unload_plan_done_) {
        transitionTo(OperationState::UNLOAD_EXECUTE);
      }

      break;


    case OperationState::LEVEL_TRUCK:
      context_->output.seq_action = 6.0;
      if (context_->smooth_finished) {
        context_->resetBucketNumber();
        context_->output.seq_action = 6.5;
        transitionTo(OperationState::SUCCESS);
      } else if ((ros::Time::now() - state_enter_time_).toSec() >=
                 config_.level_truck_timeout_sec) {
        ROS_WARN("[DiggingCycleFsm] LEVEL_TRUCK timed out after %.2f sec; skipping",
                 config_.level_truck_timeout_sec);
        context_->resetBucketNumber();
        context_->output.seq_action = 6.5;
        transitionTo(OperationState::SUCCESS);
      }
      break;

    case OperationState::SUCCESS:
    case OperationState::IDLE:
      break;
  }
}

}  // namespace excavator_system_decision
