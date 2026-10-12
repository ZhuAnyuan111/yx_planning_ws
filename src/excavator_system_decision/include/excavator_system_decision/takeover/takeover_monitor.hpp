#pragma once

#include <vector>

#include <ros/ros.h>

#include "excavator_system_decision/decision_context.hpp"

namespace excavator_system_decision {

struct TakeoverRule {
  TakeoverRule(const char* rule_name, double monitored_seq_action, double max_duration_sec)
      : name(rule_name), seq_action(monitored_seq_action), timeout_sec(max_duration_sec) {}

  const char* name{""};
  double seq_action{0.0};
  double timeout_sec{120.0};
  bool current{false};
  bool triggered{false};
  ros::Time enter_time;
};

class TakeoverMonitor {
 public:
  void init(ros::NodeHandle private_nh, DecisionContext* context);
  void update();
  void reset();

 private:
  static bool nearEqual(double lhs, double rhs, double tolerance);
  void setTakeover(bool value);
  void initRules();

  ros::NodeHandle private_nh_;
  DecisionContext* context_{nullptr};
  std::vector<TakeoverRule> rules_;
  double seq_action_tolerance_{1e-6};
  double reset_execute_timeout_sec_{120.0};
  double hold_sec_{10.0};

  bool initialized_{false};
  bool takeover_active_{false};
  ros::Time takeover_start_time_;
};

}  // namespace excavator_system_decision
