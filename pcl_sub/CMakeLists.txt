cmake_minimum_required(VERSION 3.0.2)
project(gocator_subsriber)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
catkin_package(
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(gocator_subsriber src/gocator_subsriber.cpp)
target_link_libraries(gocator_subsriber ${catkin_LIBRARIES})