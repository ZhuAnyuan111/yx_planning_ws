#pragma once

#include <ros/ros.h>

#include "excavator_system_decision/decision_context.hpp"
#include "excavator_system_decision/scenario_base.hpp"
#include "excavator_system_decision/scenarios/truck_interaction_scenario/dig_wait_fsm/dig_wait_fsm.hpp"
#include "excavator_system_decision/scenarios/truck_interaction_scenario/reset_dig_wait_fsm/reset_dig_wait_fsm.hpp"
#include "excavator_system_decision/scenarios/truck_interaction_scenario/digging_cycle_fsm/digging_cycle_fsm.hpp"
#include "excavator_system_decision/scenarios/truck_interaction_scenario/staging_pre_move_fsm/staging_pre_move_fsm.hpp"
#include "excavator_system_decision/scenarios/truck_interaction_scenario/staging_cycle_fsm/staging_cycle_fsm.hpp"
#include "excavator_system_decision/topic_interface.hpp"

namespace excavator_system_decision {

struct TruckInteractionConfig {
  int bucket_numbers_takeover_threshold{5};
  int bucket_numbers_threshold_low{3};
  int bucket_numbers_threshold_high{5};
  double request_entry_code{1.0};
  double entering_code{2.0};
  double loading_code{3.0};
  double leave_confirmed_code{4.0};
  double working_command{1.0};
  double permit_entry_command{2.0};
  double request_loading_command{3.0};
  double request_leave_command{4.0};
  double truck_event_stale_sec{2.0};
  double truck_center_angle_error_threshold{30.0};
};

class TruckInteractionScenario : public ScenarioBase {
 public:
  void init(ros::NodeHandle nh, TopicInterface* topics, DecisionContext* context);
  ScenarioType type() const override { return ScenarioType::TRUCK_INTERACTION; }
  void start() override;
  void stop() override;
  void process() override;
  bool running() const override { return running_; }

  TruckStage stage() const { return stage_; }
  OperationState diggingCycleState() const { return digging_cycle_fsm_.state(); }
  DigWaitFsmState digWaitState() const { return dig_wait_fsm_.state(); }
  ResetDigWaitFsmState resetDigWaitState() const { return reset_dig_wait_fsm_.state(); }
  StagingPreMoveState stagingPreMoveState() const { return staging_pre_move_fsm_.state(); }
  StagingState stagingCycleState() const { return staging_cycle_fsm_.state(); }

 private:
  void transitionTo(TruckStage next);
  double truckEventCode() const;
  bool truckEventFresh() const;

  ros::NodeHandle nh_;
  TopicInterface* topics_{nullptr};
  DecisionContext* context_{nullptr};
  DiggingCycleFsm digging_cycle_fsm_;
  DigWaitFsm dig_wait_fsm_;
  ResetDigWaitFsm reset_dig_wait_fsm_;
  StagingPreMoveFsm staging_pre_move_fsm_;
  StagingCycleFsm staging_cycle_fsm_;
  TruckInteractionConfig config_;
  TruckStage stage_{TruckStage::IDLE};
  bool running_{false};
  bool entry_requested_{false};
  bool truck_entered_{false};
  bool initial_dig_wait_selected_{false};
  bool initial_use_reset_dig_wait_{false};
  bool truck_leave_confirmed_{false};
  bool dig_one_bucket_wait_finished_{false};
};

}  // namespace excavator_system_decision
