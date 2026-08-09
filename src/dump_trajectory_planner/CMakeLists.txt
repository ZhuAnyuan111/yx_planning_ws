cmake_minimum_required(VERSION 3.0.2)
project(dump_trajectory_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  visualization_msgs
  kinematics
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs visualization_msgs kinematics
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========== 分层构建 ==========
# Layer 1: 纯 C++ 工具层（角度工具 + 速度估计 + 三次 Hermite 插值 + 可行性检查 + 航路点生成）
# Layer 2: ROS 适配层（状态上报器 + 可视化器）+ 节点类实现层
add_library(${PROJECT_NAME}_lib
  src/angle_utils.cpp
  src/velocity_estimator.cpp
  src/status_reporter.cpp
  src/dump_visualizer.cpp
  src/cubic_hermite_interpolator.cpp
  src/dump_feasibility.cpp
  src/waypoint_generator.cpp
  src/dump_trajectory_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# Layer 3: ROS 节点入口
add_executable(dump_trajectory_node src/dump_trajectory_node_main.cpp)
target_link_libraries(dump_trajectory_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

# ========== 安装 ==========
install(TARGETS ${PROJECT_NAME}_lib dump_trajectory_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
