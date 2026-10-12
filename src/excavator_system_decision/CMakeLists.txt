cmake_minimum_required(VERSION 3.0.2)
project(excavator_system_decision)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/decision_node.cpp
  src/scenarios/truck_interaction_scenario/digging_cycle_fsm/digging_cycle_fsm.cpp
  src/scenarios/truck_interaction_scenario/dig_wait_fsm/dig_wait_fsm.cpp
  src/scenarios/truck_interaction_scenario/reset_dig_wait_fsm/reset_dig_wait_fsm.cpp
  src/scenarios/truck_interaction_scenario/staging_pre_move_fsm/staging_pre_move_fsm.cpp
  src/scenarios/truck_interaction_scenario/staging_cycle_fsm/staging_cycle_fsm.cpp
  src/scenario_manager.cpp
  src/topic_interface.cpp
  src/scenarios/truck_interaction_scenario/truck_interaction_scenario.cpp
  src/takeover/takeover_monitor.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/decision_test_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)








