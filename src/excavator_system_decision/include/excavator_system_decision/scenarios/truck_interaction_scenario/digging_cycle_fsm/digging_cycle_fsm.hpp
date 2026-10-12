#pragma once

#include <cstdint>

#include <ros/ros.h>

#include "excavator_system_decision/decision_context.hpp"
#include "excavator_system_decision/state_types.hpp"
#include "excavator_system_decision/topic_interface.hpp"

namespace excavator_system_decision {
enum class PlanPhase : uint8_t {
  DIG_POINT_SELECTION = 0,
  DONE
};

struct DiggingCycleFsmConfig {
  int min_bucket_count{3};
  int max_bucket_count{5};
  double boom_up_angle_threshold{33.0};
  double level_truck_timeout_sec{15.0};
};

class DiggingCycleFsm {
 public:
  void init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context);
  void start();
  void cancel();
  void update();

  OperationState state() const { return state_; }
  bool finished() const;
  bool successful() const { return state_ == OperationState::SUCCESS; }

 private:
  void transitionTo(OperationState next);
  bool feedbackAfter(const ros::Time& feedback_time) const;
  bool executionFeedbackReady() const;

  ros::NodeHandle nh_;
  TopicInterface* topics_{nullptr};
  DecisionContext* context_{nullptr};
  DiggingCycleFsmConfig config_;
  OperationState state_{OperationState::IDLE};
  ros::Time state_enter_time_;
  bool unload_done_{false};
  bool dig_execute_done_{false};
  bool unload_plan_done_{false};
  PlanPhase plan_phase_{PlanPhase::DIG_POINT_SELECTION};
};

}  // namespace excavator_system_decision
