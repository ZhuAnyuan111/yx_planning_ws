#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "excavator_system_decision/decision_context.hpp"
#include "excavator_system_decision/scenario_manager.hpp"
#include "excavator_system_decision/topic_interface.hpp"
#include "excavator_system_decision/scenarios/truck_interaction_scenario/truck_interaction_scenario.hpp"
#include "excavator_system_decision/takeover/takeover_monitor.hpp"

namespace excavator_system_decision {

class DecisionNode {
 public:
  DecisionNode();

 private:
  void mpcSwitchCallback(const std_msgs::Bool::ConstPtr& msg);
  void processCallback(const ros::TimerEvent& event);
  void clearCommandsIfStopped();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber mpc_switch_sub_;
  ros::Timer process_timer_;

  DecisionContext context_;
  TopicInterface topics_;
  ScenarioManager scenario_manager_;
  std::shared_ptr<TruckInteractionScenario> truck_interaction_;
  TakeoverMonitor takeover_monitor_;
  bool stopped_commands_published_{false};
};

}  // namespace excavator_system_decision


