cmake_minimum_required(VERSION 3.0.2)
project(gocator_publisher)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

generate_dynamic_reconfigure_options(cfg/gocator_publisher_params.cfg)

catkin_package(
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(gocator_publisher src/gocator_publisher.cpp)
target_link_libraries(gocator_publisher ${catkin_LIBRARIES})