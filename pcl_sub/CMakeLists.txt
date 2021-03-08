cmake_minimum_required(VERSION 3.0.2)
project(gocator_subscriber)
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
add_executable(gocator_subscriber src/gocator_subscriber.cpp)
target_link_libraries(gocator_subscriber ${catkin_LIBRARIES})