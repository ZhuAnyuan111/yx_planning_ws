# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;geometry_msgs;std_msgs;visualization_msgs;kinematics".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldump_trajectory_planner_lib".split(';') if "-ldump_trajectory_planner_lib" != "" else []
PROJECT_NAME = "dump_trajectory_planner"
PROJECT_SPACE_DIR = "/root/yx_planning_ws/install"
PROJECT_VERSION = "0.1.0"
