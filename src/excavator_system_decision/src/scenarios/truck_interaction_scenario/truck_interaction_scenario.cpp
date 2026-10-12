#include "excavator_system_decision/scenarios/truck_interaction_scenario/truck_interaction_scenario.hpp"

#include <cmath>
#include <string>

namespace excavator_system_decision {

void TruckInteractionScenario::init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context) {
  nh_ = nh;
  topics_ = topics;
  context_ = context;
  nh_.param("thresholds/bucket_numbers_takeover_threshold", config_.bucket_numbers_takeover_threshold, config_.bucket_numbers_takeover_threshold);
  nh_.param("thresholds/bucket_numbers_threshold_low", config_.bucket_numbers_threshold_low, config_.bucket_numbers_threshold_low);
  nh_.param("thresholds/bucket_numbers_threshold_high", config_.bucket_numbers_threshold_high, config_.bucket_numbers_threshold_high);
  nh_.param("truck_events/request_entry", config_.request_entry_code, config_.request_entry_code);
  nh_.param("truck_events/entering", config_.entering_code, config_.entering_code);
  nh_.param("truck_events/loading", config_.loading_code, config_.loading_code);
  nh_.param("truck_events/leave_confirmed", config_.leave_confirmed_code, config_.leave_confirmed_code);
  nh_.param("truck_events/working_command", config_.working_command, config_.working_command);
  nh_.param("truck_events/permit_entry_command", config_.permit_entry_command, config_.permit_entry_command);
  nh_.param("truck_events/request_loading_command", config_.request_loading_command, config_.request_loading_command);
  nh_.param("truck_events/request_leave_command", config_.request_leave_command, config_.request_leave_command);
  nh_.param("timing/truck_event_stale_sec", config_.truck_event_stale_sec, config_.truck_event_stale_sec);
  nh_.param("operation/truck_center_angle_error_threshold",
            config_.truck_center_angle_error_threshold,
            config_.truck_center_angle_error_threshold);
  if (config_.truck_center_angle_error_threshold < 0.0) {
    ROS_WARN("[TruckInteractionScenario] truck_center_angle_error_threshold must be non-negative, using 30.0");
    config_.truck_center_angle_error_threshold = 30.0;
  }
  digging_cycle_fsm_.init(nh_, topics_, context_);
  dig_wait_fsm_.init(nh_, topics_, context_);
  reset_dig_wait_fsm_.init(nh_, topics_, context_);
  staging_pre_move_fsm_.init(nh_, topics_, context_);
  staging_cycle_fsm_.init(nh_, topics_, context_);
}

void TruckInteractionScenario::start() {
  if (running_) return;
  running_ = true;
  entry_requested_ = false;
  truck_entered_ = false;
  initial_dig_wait_selected_ = false;
  initial_use_reset_dig_wait_ = false;
  truck_leave_confirmed_ = false;
  dig_one_bucket_wait_finished_ = false;
  transitionTo(TruckStage::INITIAL_WAIT_TRUCK_ENTRY);
  ROS_INFO("[TruckInteractionScenario] Started by /Mpc_Switch");
}

void TruckInteractionScenario::stop() {
  if (!running_) return;

  digging_cycle_fsm_.cancel();
  dig_wait_fsm_.cancel();
  reset_dig_wait_fsm_.cancel();
  staging_pre_move_fsm_.cancel();
  staging_cycle_fsm_.cancel();
  // Stopping a scenario must not clear truck or bucket counters.
  if (context_) context_->output.resetAll();
  running_ = false;
  entry_requested_ = false;
  truck_entered_ = false;
  initial_dig_wait_selected_ = false;
  initial_use_reset_dig_wait_ = false;
  truck_leave_confirmed_ = false;
  dig_one_bucket_wait_finished_ = false;
  stage_ = TruckStage::IDLE;
  ROS_INFO("[TruckInteractionScenario] Stopped");
}

double TruckInteractionScenario::truckEventCode() const {
  return context_ ? context_->truck_event_code : 0.0;
}

bool TruckInteractionScenario::truckEventFresh() const {
  if (!context_ || context_->truck_event_time.isZero()) return false;

  const double age_sec = (ros::Time::now() - context_->truck_event_time).toSec();
  return age_sec >= 0.0 && age_sec <= config_.truck_event_stale_sec;
}

void TruckInteractionScenario::transitionTo(TruckStage next) {
  stage_ = next;
  ROS_INFO("[TruckInteractionScenario] -> %s", toString(stage_));
  if (next == TruckStage::TRUCK_POSITION_CHECK) {
    context_->output.seq_action = 1.0;
  }
  if (next == TruckStage::INITIAL_WAIT_TRUCK_ENTRY) {
    entry_requested_ = false;
    truck_entered_ = false;
    initial_dig_wait_selected_ = false;
    initial_use_reset_dig_wait_ = false;
    dig_wait_fsm_.cancel();
    reset_dig_wait_fsm_.cancel();
    context_->output.digger_loading.pose.orientation.w = 0.0;
  }
  if (next == TruckStage::DISPATCH_TRUCK_LEAVE) {
    truck_leave_confirmed_ = false;
    dig_one_bucket_wait_finished_ = false;
    dig_wait_fsm_.cancel();
    reset_dig_wait_fsm_.start();
  }
  if (next == TruckStage::STAGING_PRE_MOVE) {
    entry_requested_ = false;
    truck_entered_ = false;
    reset_dig_wait_fsm_.cancel();
    staging_cycle_fsm_.cancel();
    staging_pre_move_fsm_.start();
    context_->output.digger_loading.pose.orientation.w = 0.0;
  }
  if (next == TruckStage::STAGING_CYCLE) {
    staging_pre_move_fsm_.cancel();
    staging_cycle_fsm_.start();
    context_->output.digger_loading.pose.orientation.w = 0.0;
  }
  if (next == TruckStage::WAIT_TRUCK_ENTRY) {
    entry_requested_ = false;
    truck_entered_ = false;
    dig_wait_fsm_.cancel();
    reset_dig_wait_fsm_.cancel();
    staging_pre_move_fsm_.cancel();
    staging_cycle_fsm_.cancel();
    context_->output.digger_loading.pose.orientation.w = 0.0;
  }
  if (next == TruckStage::DIGGING_CYCLE) digging_cycle_fsm_.start();
}

void TruckInteractionScenario::process() {
  if (!running_) return;

  switch (stage_) {
    case TruckStage::INITIAL_WAIT_TRUCK_ENTRY: {
      if (truckEventFresh() &&
          std::abs(truckEventCode() - config_.request_entry_code) < 1e-6) {
        if (!entry_requested_) {
          entry_requested_ = true;
          context_->output.digger_loading.pose.orientation.w = config_.permit_entry_command;
        }
      }

      // Do not start either FSM until both angles have been received.  The
      // selected FSM then runs without a timeout, as required.
      if (!initial_dig_wait_selected_ && context_->real_swing_angle_received &&
          context_->truck_center_unloadpoint_angle_received) {
        const double angle_error = std::fabs(std::remainder(
            context_->real_swing_angle - context_->truck_center_unloadpoint_angle, 360.0));
        initial_use_reset_dig_wait_ =
            angle_error <= config_.truck_center_angle_error_threshold;
        initial_dig_wait_selected_ = true;
        if (initial_use_reset_dig_wait_) {
          reset_dig_wait_fsm_.start();
          ROS_INFO("[TruckInteractionScenario] INITIAL_WAIT_TRUCK_ENTRY: "
                   "using ResetDigWaitFsm (angle error %.2f deg)", angle_error);
        } else {
          dig_wait_fsm_.start();
          ROS_INFO("[TruckInteractionScenario] INITIAL_WAIT_TRUCK_ENTRY: "
                   "using DigWaitFsm (angle error %.2f deg)", angle_error);
        }
      }

      if (initial_dig_wait_selected_) {
        if (initial_use_reset_dig_wait_) {
          reset_dig_wait_fsm_.update();
        } else {
          dig_wait_fsm_.update();
        }
      }

      // ResetDigWaitFsm::start() clears command outputs. Keep the entry
      // permit asserted while waiting for the truck, regardless of FSM choice.
      if (entry_requested_) {
        context_->output.digger_loading.pose.orientation.w = config_.permit_entry_command;
      }

    const bool dig_wait_finished = initial_dig_wait_selected_ &&
        (initial_use_reset_dig_wait_ ? reset_dig_wait_fsm_.finished()
                                     : dig_wait_fsm_.finished());
    if (entry_requested_ && dig_wait_finished &&
        truckEventFresh() &&
        std::abs(truckEventCode() - config_.loading_code) < 1e-6) {
      truck_entered_ = true;
      context_->resetBucketNumber();
      transitionTo(TruckStage::TRUCK_POSITION_CHECK);
    }
      break;
    }

    case TruckStage::WAIT_TRUCK_ENTRY:
    if (truckEventFresh() &&
        std::abs(truckEventCode() - config_.request_entry_code) < 1e-6) {
      if (!entry_requested_) {
        entry_requested_ = true;
        context_->output.digger_loading.pose.orientation.w = config_.permit_entry_command;
      }
    }

    if (entry_requested_ &&
        truckEventFresh() &&
        std::abs(truckEventCode() - config_.loading_code) < 1e-6) {
      truck_entered_ = true;
      context_->resetBucketNumber();
      transitionTo(TruckStage::TRUCK_POSITION_CHECK);
    }
    break;

    case TruckStage::TRUCK_POSITION_CHECK:
        context_->truck_number += 1;
        transitionTo(TruckStage::DIGGING_CYCLE);
      break;

    case TruckStage::DIGGING_CYCLE:
      context_->output.digger_loading.pose.orientation.w = config_.request_loading_command;
      digging_cycle_fsm_.update();
      if (digging_cycle_fsm_.successful()) transitionTo(TruckStage::DISPATCH_TRUCK_LEAVE);
      break;

    case TruckStage::DISPATCH_TRUCK_LEAVE:
      context_->output.digger_loading.pose.orientation.w = config_.request_leave_command;
      // Run the post-leave dig-wait FSM in parallel with waiting for the
      // truck leave confirmation.
      reset_dig_wait_fsm_.update();
      if (reset_dig_wait_fsm_.finished()) {
        dig_one_bucket_wait_finished_ = true;
      }
      if (truckEventFresh() && std::abs(truckEventCode() - config_.leave_confirmed_code) < 1e-6) {
        entry_requested_ = false;
        truck_entered_ = false;
        truck_leave_confirmed_ = true;
      }
      if (truck_leave_confirmed_ && dig_one_bucket_wait_finished_) {
        if (context_->staging_requested) {
          transitionTo(TruckStage::STAGING_PRE_MOVE);
        } else {
          transitionTo(TruckStage::WAIT_TRUCK_ENTRY);
        }
      }
      break;

    case TruckStage::STAGING_PRE_MOVE:
      staging_pre_move_fsm_.update();
      if (staging_pre_move_fsm_.successful()) {
        transitionTo(TruckStage::STAGING_CYCLE);
      }
      break;

    case TruckStage::STAGING_CYCLE:
      staging_cycle_fsm_.update();
      if (staging_cycle_fsm_.successful()) {
        context_->resetBucketNumber();
        transitionTo(TruckStage::WAIT_TRUCK_ENTRY);
      }
      break;

    case TruckStage::IDLE:
      break;
  }

  context_->output.state_text = std::string(toString(stage_)) + ":" +
                                toString(digging_cycle_fsm_.state()) + ":" +
                                toString(staging_pre_move_fsm_.state()) + ":" +
                                toString(staging_cycle_fsm_.state());
}

}  // namespace excavator_system_decision
