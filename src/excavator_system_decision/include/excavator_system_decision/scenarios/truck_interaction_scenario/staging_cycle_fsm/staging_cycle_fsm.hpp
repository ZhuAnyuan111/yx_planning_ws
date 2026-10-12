#pragma once

#include <cstdint>

#include <ros/ros.h>

#include "excavator_system_decision/decision_context.hpp"
#include "excavator_system_decision/state_types.hpp"
#include "excavator_system_decision/topic_interface.hpp"

namespace excavator_system_decision {

class StagingCycleFsm {
 public:
  void init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context);
  void start();
  void cancel();
  void update();

  StagingState state() const { return state_; }
  bool running() const { return state_ != StagingState::IDLE; }
  bool successful() const { return state_ == StagingState::SUCCESS; }

 private:
  void transitionTo(StagingState next);
  bool feedbackAfter(const ros::Time& feedback_time) const;
  bool executionFeedbackReady() const;
  bool swingAngleWithinThreshold() const;

  ros::NodeHandle nh_;
  TopicInterface* topics_{nullptr};
  DecisionContext* context_{nullptr};
  StagingState state_{StagingState::IDLE};
  ros::Time state_enter_time_;
  double boom_up_angle_threshold_{33.0};
  double swing_angle_error_threshold_{20.0};
  bool initial_unload_execute_done_{false};
  bool reset_execute_done_{false};
  bool dig_execute_done_{false};
  bool unload_plan_done_{false};
  bool unload_execute_done_{false};
  bool final_dig_execute_done_{false};
};

}  // namespace excavator_system_decision
