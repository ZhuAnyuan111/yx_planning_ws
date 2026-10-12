#pragma once

#include <cstdint>
#include <ros/ros.h>

#include "excavator_system_decision/decision_context.hpp"
#include "excavator_system_decision/topic_interface.hpp"

namespace excavator_system_decision {

enum class DigWaitFsmState : uint8_t {
  IDLE = 0,
  DIG_EXECUTE,
  BOOM_UP_PLAN,
  DONE
};

inline const char* toString(DigWaitFsmState state) {
  switch (state) {
    case DigWaitFsmState::IDLE: return "IDLE";
    case DigWaitFsmState::DIG_EXECUTE: return "DIG_EXECUTE";
    case DigWaitFsmState::BOOM_UP_PLAN: return "BOOM_UP_PLAN";
    case DigWaitFsmState::DONE: return "DONE";
  }
  return "UNKNOWN";
}

struct DigWaitFsmConfig {
  double boom_up_angle_threshold{33.0};
  double swing_angle_error_threshold{20.0};
};

class DigWaitFsm {
 public:
  void init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context);
  void start();
  void cancel();
  void update();

  bool running() const { return state_ != DigWaitFsmState::IDLE; }
  bool finished() const { return state_ == DigWaitFsmState::DONE; }
  DigWaitFsmState state() const { return state_; }

 private:
  void transitionTo(DigWaitFsmState next);
  bool feedbackAfter(const ros::Time& feedback_time) const;
  bool executionFeedbackReady() const;
  bool swingAngleWithinThreshold() const;

  ros::NodeHandle nh_;
  TopicInterface* topics_{nullptr};
  DecisionContext* context_{nullptr};
  DigWaitFsmConfig config_;
  DigWaitFsmState state_{DigWaitFsmState::IDLE};
  ros::Time state_enter_time_;
};

}  // namespace excavator_system_decision
