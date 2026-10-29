cmake_minimum_required(VERSION 3.5)
project(data_to_nas_and_ubuntu2204)

add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  grid_map_msgs
  interfaces
)

find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)
pkg_check_modules(LIBJPEG REQUIRED libjpeg)
pkg_check_modules(LIBPNG REQUIRED libpng)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs grid_map_msgs interfaces
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${LIBJPEG_INCLUDE_DIRS}
  ${LIBPNG_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${LIBJPEG_LIBRARY_DIRS}
  ${LIBPNG_LIBRARY_DIRS}
)

add_executable(data_to_NAS_and_ubuntu2204_node
  src/data_to_NAS_and_ubuntu2204_node.cpp
)

add_dependencies(data_to_NAS_and_ubuntu2204_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(data_to_NAS_and_ubuntu2204_node
  ${catkin_LIBRARIES}
  ${CURL_LIBRARIES}
  ${OPENSSL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${LIBJPEG_LIBRARIES}
  ${LIBPNG_LIBRARIES}
  pthread
)

target_compile_options(data_to_NAS_and_ubuntu2204_node PRIVATE
  ${GSTREAMER_CFLAGS_OTHER}
  ${LIBJPEG_CFLAGS_OTHER}
  ${LIBPNG_CFLAGS_OTHER}
)

install(TARGETS data_to_NAS_and_ubuntu2204_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
