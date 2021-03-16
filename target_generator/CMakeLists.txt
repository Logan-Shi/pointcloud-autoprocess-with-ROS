cmake_minimum_required(VERSION 3.0.2)
project(target_generator)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(target_generator src/target_generator.cpp)
target_link_libraries(target_generator ${catkin_LIBRARIES})