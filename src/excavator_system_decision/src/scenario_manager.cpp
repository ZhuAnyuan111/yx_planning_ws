#include "excavator_system_decision/scenario_manager.hpp"

#include <ros/ros.h>

namespace excavator_system_decision {

void ScenarioManager::registerScenario(const std::shared_ptr<ScenarioBase>& scenario) {
  scenarios_[scenario->type()] = scenario;
}

bool ScenarioManager::start(ScenarioType type) {
  const auto it = scenarios_.find(type);
  if (it == scenarios_.end()) {
    ROS_ERROR("[ScenarioManager] Scenario is not registered");
    return false;
  }
  if (active_scenario_ && active_scenario_ != it->second) active_scenario_->stop();
  active_scenario_ = it->second;
  if (!active_scenario_->running()) active_scenario_->start();
  return true;
}

void ScenarioManager::stop(ScenarioType type) {
  const auto it = scenarios_.find(type);
  if (it == scenarios_.end()) return;
  it->second->stop();
  if (active_scenario_ == it->second) active_scenario_.reset();
}

void ScenarioManager::process() {
  if (active_scenario_ && active_scenario_->running()) active_scenario_->process();
}

bool ScenarioManager::running(ScenarioType type) const {
  const auto it = scenarios_.find(type);
  return it != scenarios_.end() && it->second->running();
}

}  // namespace excavator_system_decision
