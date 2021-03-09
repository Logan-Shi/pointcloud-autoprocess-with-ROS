cmake_minimum_required(VERSION 3.0.2)
project(measure_node)

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

# Set source files for the ROS node
SET(SRCS_NODE 
    src/measure_node.cpp
    src/measure_node_main.cpp
)

add_executable(${PROJECT_NAME} ${SRCS_NODE})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})