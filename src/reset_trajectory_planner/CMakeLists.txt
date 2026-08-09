cmake_minimum_required(VERSION 3.0.2)
project(reset_trajectory_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========== 分层构建 ==========
# Layer 1: 纯 C++ 工具层（angle_utils）+ ROS 节点类实现层，
# 打包为共享库，便于单元测试与其他节点复用
add_library(${PROJECT_NAME}_lib
  src/angle_utils.cpp
  src/reset_trajectory_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# Layer 2: ROS 节点入口（只含 main），链接算法/节点库
add_executable(reset_trajectory_node src/reset_trajectory_node_main.cpp)
target_link_libraries(reset_trajectory_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

# ========== 安装 ==========
install(TARGETS ${PROJECT_NAME}_lib reset_trajectory_node
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
