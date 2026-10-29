cmake_minimum_required(VERSION 3.0.2)
project(smooth_trajectory_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  kinematics
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs kinematics
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========== 分层构建 ==========
# Layer 1: 纯 C++ 工具层（angle_utils）+ ROS 节点类实现层
add_library(${PROJECT_NAME}_lib
  src/angle_utils.cpp
  src/smooth_trajectory_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# Layer 2: ROS 节点入口
add_executable(smooth_trajectory_node src/smooth_trajectory_node_main.cpp)
target_link_libraries(smooth_trajectory_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

# ========== 安装 ==========
install(TARGETS ${PROJECT_NAME}_lib smooth_trajectory_node
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
