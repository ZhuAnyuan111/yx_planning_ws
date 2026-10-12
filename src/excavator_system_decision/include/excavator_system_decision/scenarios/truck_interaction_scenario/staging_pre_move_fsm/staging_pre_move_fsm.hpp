#pragma once

#include <cstdint>

#include <ros/ros.h>

#include "excavator_system_decision/decision_context.hpp"
#include "excavator_system_decision/topic_interface.hpp"

namespace excavator_system_decision {

enum class StagingPreMoveState : uint8_t {
  IDLE = 0,
  SWING_RESET,
  WALK_EXECUTE,
  SUCCESS
};

inline const char* toString(StagingPreMoveState state) {
  switch (state) {
    case StagingPreMoveState::IDLE: return "IDLE";
    case StagingPreMoveState::SWING_RESET: return "SWING_RESET";
    case StagingPreMoveState::WALK_EXECUTE: return "WALK_EXECUTE";
    case StagingPreMoveState::SUCCESS: return "SUCCESS";
  }
  return "UNKNOWN";
}

// The downstream motion controller executes the pre-move from Sys_SeqAction:
// 9.0 requests swing reset; 8.0 requests reverse walking.
class StagingPreMoveFsm {
 public:
  void init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context);
  void start();
  void cancel();
  void update();

  StagingPreMoveState state() const { return state_; }
  bool successful() const { return state_ == StagingPreMoveState::SUCCESS; }

 private:
  void transitionTo(StagingPreMoveState next);

  ros::NodeHandle nh_;
  TopicInterface* topics_{nullptr};
  DecisionContext* context_{nullptr};
  StagingPreMoveState state_{StagingPreMoveState::IDLE};
  ros::Time state_enter_time_;
};

}  // namespace excavator_system_decision
