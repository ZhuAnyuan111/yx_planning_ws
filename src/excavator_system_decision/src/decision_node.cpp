#include "excavator_system_decision/decision_node.hpp"

namespace excavator_system_decision {

DecisionNode::DecisionNode()
    : private_nh_("~") {

  context_.initializeForProgramStart();
  topics_.init(private_nh_, &context_);
  takeover_monitor_.init(private_nh_, &context_);

  truck_interaction_ = std::make_shared<TruckInteractionScenario>();
  truck_interaction_->init(private_nh_, &topics_, &context_);
  scenario_manager_.registerScenario(truck_interaction_);

  mpc_switch_sub_ = nh_.subscribe("/Mpc_Switch", 10, &DecisionNode::mpcSwitchCallback, this);

  double rate_hz = 10.0;
  private_nh_.param("update_rate_hz", rate_hz, rate_hz);
  if (rate_hz <= 0.0) rate_hz = 10.0;
  process_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz), &DecisionNode::processCallback, this);

  topics_.publishAllCommandsZero();
  ROS_INFO("[DecisionNode] Ready; TruckInteractionScenario trigger: /Mpc_Switch");
}

void DecisionNode::mpcSwitchCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    stopped_commands_published_ = false;
    scenario_manager_.start(ScenarioType::TRUCK_INTERACTION);
  } else {
    scenario_manager_.stop(ScenarioType::TRUCK_INTERACTION);
    clearCommandsIfStopped();
  }
}

void DecisionNode::clearCommandsIfStopped() {
  if (!stopped_commands_published_ && !scenario_manager_.running(ScenarioType::TRUCK_INTERACTION)) {
    topics_.publishAllCommandsZero();
    takeover_monitor_.reset();
    stopped_commands_published_ = true;
  }
}

void DecisionNode::processCallback(const ros::TimerEvent&) {
  scenario_manager_.process();
  takeover_monitor_.update();
  clearCommandsIfStopped();
  topics_.publishOutputs();
}

}  // namespace excavator_system_decision

int main(int argc, char** argv) {
  ros::init(argc, argv, "excavator_system_decision");
  excavator_system_decision::DecisionNode node;
  ros::spin();
  return 0;
}
