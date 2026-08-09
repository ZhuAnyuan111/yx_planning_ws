cmake_minimum_required(VERSION 3.0.2)
project(dig_trajectory_planner)

set(CMAKE_CXX_STANDARD 17)
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
# Layer 1: 纯 C++ 工具层（地形 + 运动学 + DP核心 + 后处理）
add_library(${PROJECT_NAME}_lib
  src/terrain_profile.cpp
  src/excavator_fk.cpp
  src/dig_planner_dp.cpp
  src/trajectory_postprocess.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# ========== 安装 ==========
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
