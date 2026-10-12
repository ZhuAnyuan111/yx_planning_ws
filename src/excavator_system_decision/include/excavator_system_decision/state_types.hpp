#pragma once

#include <cstdint>
#include <string>

namespace excavator_system_decision {

enum class ProcessStatus : uint8_t {
  IDLE = 0,
  RUNNING = 1,
  FINISHED = 2,
  TIMEOUT = 3
};

enum class ScenarioType : uint8_t {
  TRUCK_INTERACTION = 1
};

enum class TruckStage : uint8_t {
  IDLE = 0,
  WAIT_TRUCK_ENTRY = 1,
  TRUCK_POSITION_CHECK = 2,
  DIGGING_CYCLE = 3,
  DISPATCH_TRUCK_LEAVE = 4,
  STAGING_PRE_MOVE = 6,
  STAGING_CYCLE = 7,
  INITIAL_WAIT_TRUCK_ENTRY = 8
};

enum class OperationState : uint8_t {
  IDLE = 0,
  UNLOAD_PLAN,
  RESET_EXECUTE,
  DIG_EXECUTE,
  UNLOAD_EXECUTE,
  LEVEL_TRUCK,
  SUCCESS
};

enum class StagingState : uint8_t {
  IDLE = 0,
  INITIAL_DELAY,
  INITIAL_UNLOAD_PLAN,
  INITIAL_UNLOAD_EXECUTE,
  RESET_EXECUTE,
  GROUND_LEVEL_EXECUTE,
  DIG_EXECUTE,
  UNLOAD_EXECUTE,
  FINAL_DIG_EXECUTE,
  FINAL_BOOM_UP_WAIT,
  SUCCESS
};

inline const char* toString(TruckStage state) {
  switch (state) {
    case TruckStage::IDLE: return "IDLE";
    case TruckStage::INITIAL_WAIT_TRUCK_ENTRY: return "INITIAL_WAIT_TRUCK_ENTRY";
    case TruckStage::WAIT_TRUCK_ENTRY: return "WAIT_TRUCK_ENTRY";
    case TruckStage::TRUCK_POSITION_CHECK: return "TRUCK_POSITION_CHECK";
    case TruckStage::DIGGING_CYCLE: return "DIGGING_CYCLE";
    case TruckStage::DISPATCH_TRUCK_LEAVE: return "DISPATCH_TRUCK_LEAVE";
    case TruckStage::STAGING_PRE_MOVE: return "STAGING_PRE_MOVE";
    case TruckStage::STAGING_CYCLE: return "STAGING_CYCLE";
  }
  return "UNKNOWN";
}

inline const char* toString(OperationState state) {
  switch (state) {
    case OperationState::IDLE: return "IDLE";
    case OperationState::UNLOAD_PLAN: return "UNLOAD_PLAN";
    case OperationState::RESET_EXECUTE: return "RESET_EXECUTE";
    case OperationState::DIG_EXECUTE: return "DIG_EXECUTE";
    case OperationState::UNLOAD_EXECUTE: return "UNLOAD_EXECUTE";
    case OperationState::LEVEL_TRUCK: return "LEVEL_TRUCK";
    case OperationState::SUCCESS: return "SUCCESS";
  }
  return "UNKNOWN";
}

inline const char* toString(StagingState state) {
  switch (state) {
    case StagingState::IDLE: return "IDLE";
    case StagingState::INITIAL_DELAY: return "INITIAL_DELAY";
    case StagingState::INITIAL_UNLOAD_PLAN: return "INITIAL_UNLOAD_PLAN";
    case StagingState::INITIAL_UNLOAD_EXECUTE: return "INITIAL_UNLOAD_EXECUTE";
    case StagingState::RESET_EXECUTE: return "RESET_EXECUTE";
    case StagingState::GROUND_LEVEL_EXECUTE: return "GROUND_LEVEL_EXECUTE";
    case StagingState::DIG_EXECUTE: return "DIG_EXECUTE";
    case StagingState::UNLOAD_EXECUTE: return "UNLOAD_EXECUTE";
    case StagingState::FINAL_DIG_EXECUTE: return "FINAL_DIG_EXECUTE";
    case StagingState::FINAL_BOOM_UP_WAIT: return "FINAL_BOOM_UP_WAIT";
    case StagingState::SUCCESS: return "SUCCESS";
  }
  return "UNKNOWN";
}
}  // namespace excavator_system_decision
