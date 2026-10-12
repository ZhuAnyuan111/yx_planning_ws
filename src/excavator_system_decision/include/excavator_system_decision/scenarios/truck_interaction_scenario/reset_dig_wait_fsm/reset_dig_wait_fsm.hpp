#pragma once

#include <cstdint>
#include <ros/ros.h>

#include "excavator_system_decision/decision_context.hpp"
#include "excavator_system_decision/topic_interface.hpp"

namespace excavator_system_decision {

enum class ResetDigWaitFsmState : uint8_t {
  IDLE = 0,
  DIG_POINT_SELECTION,
  RESET_EXECUTE,
  DIG_EXECUTE,
  BOOM_UP_EXECUTE,
  DONE
};

inline const char* toString(ResetDigWaitFsmState state) {
  switch (state) {
    case ResetDigWaitFsmState::IDLE: return "IDLE";
    case ResetDigWaitFsmState::DIG_POINT_SELECTION: return "DIG_POINT_SELECTION";
    case ResetDigWaitFsmState::RESET_EXECUTE: return "RESET_EXECUTE";
    case ResetDigWaitFsmState::DIG_EXECUTE: return "DIG_EXECUTE";
    case ResetDigWaitFsmState::BOOM_UP_EXECUTE: return "BOOM_UP_EXECUTE";
    case ResetDigWaitFsmState::DONE: return "DONE";
  }
  return "UNKNOWN";
}

struct ResetDigWaitFsmConfig {
  double boom_up_angle_threshold{33.0};
  double swing_angle_error_threshold{20.0};
};

class ResetDigWaitFsm {
 public:
  void init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context);
  void start();
  void cancel();
  void update();

  bool running() const { return state_ != ResetDigWaitFsmState::IDLE; }
  bool finished() const { return state_ == ResetDigWaitFsmState::DONE; }
  ResetDigWaitFsmState state() const { return state_; }

 private:
  void transitionTo(ResetDigWaitFsmState next);
  bool feedbackAfter(const ros::Time& feedback_time) const;
  bool executionFeedbackReady() const;
  bool swingAngleWithinThreshold() const;

  ros::NodeHandle nh_;
  TopicInterface* topics_{nullptr};
  DecisionContext* context_{nullptr};
  ResetDigWaitFsmConfig config_;
  ResetDigWaitFsmState state_{ResetDigWaitFsmState::IDLE};
  ros::Time state_enter_time_;
};

}  // namespace excavator_system_decision
