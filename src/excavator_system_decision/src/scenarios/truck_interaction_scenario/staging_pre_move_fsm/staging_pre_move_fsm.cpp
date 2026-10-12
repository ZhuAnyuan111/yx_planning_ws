#include "excavator_system_decision/scenarios/truck_interaction_scenario/staging_pre_move_fsm/staging_pre_move_fsm.hpp"

namespace excavator_system_decision {

void StagingPreMoveFsm::init(ros::NodeHandle nh,
                             TopicInterface* topics,
                             DecisionContext* context) {
  nh_ = nh;
  topics_ = topics;
  context_ = context;
}

void StagingPreMoveFsm::start() {
  if (context_) {
    context_->walk_finished = false;
    // Each swing-reset action must receive a new external completion event.
    context_->upper_lower_alignment_finished = false;
    context_->upper_lower_alignment_finish_time = ros::Time();
  }
 //transitionTo(StagingPreMoveState::SWING_RESET);
  transitionTo(StagingPreMoveState::SUCCESS);
}

void StagingPreMoveFsm::cancel() {
  if (context_) {
    context_->walk_finished = false;
    context_->upper_lower_alignment_finished = false;
  }
  state_ = StagingPreMoveState::IDLE;
}

void StagingPreMoveFsm::transitionTo(StagingPreMoveState next) {
  if (next == StagingPreMoveState::WALK_EXECUTE && context_) {
    context_->walk_finished = false;
  }
  state_ = next;
  state_enter_time_ = ros::Time::now();
  ROS_INFO("[StagingPreMoveFsm] -> %s", toString(state_));
}

void StagingPreMoveFsm::update() {
  if (!context_ || state_ == StagingPreMoveState::IDLE ||
      state_ == StagingPreMoveState::SUCCESS) {
    return;
  }

  switch (state_) {
    case StagingPreMoveState::SWING_RESET:
      context_->output.seq_action = 9.0;
      if (context_->upper_lower_alignment_finished &&
          !context_->upper_lower_alignment_finish_time.isZero() &&
          context_->upper_lower_alignment_finish_time > state_enter_time_) {
        transitionTo(StagingPreMoveState::WALK_EXECUTE);
      }
      break;

    case StagingPreMoveState::WALK_EXECUTE:
      context_->output.seq_action = 8.0;
      if (context_->walk_finished &&
          !context_->walk_finish_time.isZero() &&
          context_->walk_finish_time > state_enter_time_) {
        context_->walk_finished = false;
        transitionTo(StagingPreMoveState::SUCCESS);
      }
      break;

    case StagingPreMoveState::SUCCESS:
    case StagingPreMoveState::IDLE:
      break;
  }
}

}  // namespace excavator_system_decision
