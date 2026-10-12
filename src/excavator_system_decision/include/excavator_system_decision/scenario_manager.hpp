#pragma once

#include <map>
#include <memory>

#include "excavator_system_decision/scenario_base.hpp"

namespace excavator_system_decision {

class ScenarioManager {
 public:
  void registerScenario(const std::shared_ptr<ScenarioBase>& scenario);
  bool start(ScenarioType type);
  void stop(ScenarioType type);
  void process();
  bool running(ScenarioType type) const;

 private:
  std::map<ScenarioType, std::shared_ptr<ScenarioBase>> scenarios_;
  std::shared_ptr<ScenarioBase> active_scenario_;
};

}  // namespace excavator_system_decision
