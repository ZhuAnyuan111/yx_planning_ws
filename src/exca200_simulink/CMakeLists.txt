cmake_minimum_required(VERSION 3.0.2)
project(exca200_simulink)

## 使用C++11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## 查找catkin的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

## 声明catkin的包
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs sensor_msgs
)

## 包含头文件的目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

install(DIRECTORY meshes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY launch urdf config scenarios docs
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Python 脚本（可视化闭环仿真体系：plant / scenario / adapter / trigger / sequencer / kpi）
## 新脚本随对应 Phase 落地时逐步加入本列表。
catkin_install_python(PROGRAMS
  scripts/exec_sim.py
  scripts/exca_plant_sim.py
  scripts/traj_to_urdf_adapter.py
  scripts/scenario_pub.py
  scripts/scenario_manager.py
  scripts/dump_trigger.py
  scripts/phase_trigger.py
  scripts/dig_planner_stub.py
  scripts/cycle_sequencer.py
  scripts/kpi_collector.py
  scripts/flow_hud.py
  scripts/run_batch.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Shell 脚本
install(PROGRAMS scripts/record_cycle.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## 可执行文件
add_executable(sy200_urdf_control src/sy200_urdf_control.cpp)

## 连接需要的库
target_link_libraries(sy200_urdf_control
  ${catkin_LIBRARIES}
)
