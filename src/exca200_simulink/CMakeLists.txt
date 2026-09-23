cmake_minimum_required(VERSION 3.0.2)
project(exca200_simulink)

## 使用C++11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## 查找catkin的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

## 声明catkin的包
catkin_package()

## 包含头文件的目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

install(DIRECTORY meshes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY launch urdf config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Python 脚本（可视化闭环仿真：执行模拟器 / deg→rad 适配 / 场景输入）
catkin_install_python(PROGRAMS
  scripts/exec_sim.py
  scripts/traj_to_urdf_adapter.py
  scripts/scenario_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## 可执行文件
add_executable(sy200_urdf_control src/sy200_urdf_control.cpp)

## 连接需要的库
target_link_libraries(sy200_urdf_control
  ${catkin_LIBRARIES}
)
