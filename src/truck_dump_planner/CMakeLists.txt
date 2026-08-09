cmake_minimum_required(VERSION 3.0.2)
project(truck_dump_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 核心算法库（坐标转换 + 车厢建模 + 可视化 + 规划 + 节点逻辑）
add_library(${PROJECT_NAME}_lib
  src/coords.cpp
  src/truck_model.cpp
  src/visualization.cpp
  src/dig_planner.cpp
  src/reset_planner.cpp
  src/planner_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# ROS 节点：订阅卡车位姿 -> 包络/卸载点/轨迹 -> 发布 RViz MarkerArray
add_executable(dump_planner_node src/dump_planner_node.cpp)
target_link_libraries(dump_planner_node ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_lib dump_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
