#include "excavator_system_decision/takeover/takeover_monitor.hpp"

#include <cmath>

namespace excavator_system_decision {

bool TakeoverMonitor::nearEqual(double lhs, double rhs, double tolerance) {
  return std::fabs(lhs - rhs) <= tolerance;
}

void TakeoverMonitor::init(ros::NodeHandle private_nh,
                           DecisionContext* context) {
  private_nh_ = private_nh;
  context_ = context;

  private_nh_.param("takeover/seq_action_tolerance",
                    seq_action_tolerance_,
                    seq_action_tolerance_);
  private_nh_.param("takeover/reset_execute_timeout_sec",
                    reset_execute_timeout_sec_,
                    reset_execute_timeout_sec_);
  private_nh_.param("takeover/hold_sec", hold_sec_, hold_sec_);

  if (seq_action_tolerance_ < 0.0) seq_action_tolerance_ = 0.0;
  if (reset_execute_timeout_sec_ <= 0.0) reset_execute_timeout_sec_ = 120.0;
  if (hold_sec_ <= 0.0) hold_sec_ = 10.0;

  initRules();
  initialized_ = true;
  reset();

  ROS_INFO("[TakeoverMonitor] Integrated monitor enabled: %zu rule(s), takeover holds for %.3fs",
           rules_.size(),
           hold_sec_);
}

void TakeoverMonitor::initRules() {
  rules_.clear();
  rules_.emplace_back("reset_execute", 2.0, reset_execute_timeout_sec_);
}

void TakeoverMonitor::update() {
  if (!initialized_ || !context_) return;

  const ros::Time now = ros::Time::now();

  if (takeover_active_) {
    if ((now - takeover_start_time_).toSec() >= hold_sec_) {
      takeover_active_ = false;
      setTakeover(false);
      ROS_WARN("[TakeoverMonitor] Fault takeover request cleared");
    } else {
      setTakeover(true);
    }
    return;
  }

  for (auto& rule : rules_) {
    const bool matched = nearEqual(context_->output.seq_action,
                                   rule.seq_action,
                                   seq_action_tolerance_);

    if (matched && !rule.current) {
      rule.enter_time = now;
      rule.triggered = false;
      ROS_INFO("[TakeoverMonitor] %s entered seq_action %.3f", rule.name, rule.seq_action);
    } else if (!matched && rule.current) {
      rule.enter_time = ros::Time();
      rule.triggered = false;
    }
    rule.current = matched;

    if (!rule.current || rule.enter_time.isZero() || rule.triggered) {
      continue;
    }

    const double duration_sec = (now - rule.enter_time).toSec();
    if (duration_sec >= rule.timeout_sec) {
      takeover_active_ = true;
      takeover_start_time_ = now;
      rule.triggered = true;
      setTakeover(true);
      ROS_ERROR("[TakeoverMonitor] %s seq_action %.3f lasted %.3fs, requesting takeover",
                rule.name,
                rule.seq_action,
                duration_sec);
      return;
    }
  }
}

void TakeoverMonitor::reset() {
  for (auto& rule : rules_) {
    rule.current = false;
    rule.triggered = false;
    rule.enter_time = ros::Time();
  }
  takeover_active_ = false;
  takeover_start_time_ = ros::Time();
  setTakeover(false);
}

void TakeoverMonitor::setTakeover(bool value) {
  if (!context_) return;
  context_->output.fault_require_takeover = value;
}

}  // namespace excavator_system_decision
