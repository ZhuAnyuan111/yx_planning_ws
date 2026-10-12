#pragma once

#include "excavator_system_decision/state_types.hpp"

namespace excavator_system_decision {

class ScenarioBase {
 public:
  virtual ~ScenarioBase() = default;

  virtual ScenarioType type() const = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void process() = 0;
  virtual bool running() const = 0;
};

}  // namespace excavator_system_decision
