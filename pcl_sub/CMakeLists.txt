cmake_minimum_required(VERSION 3.0.2)
project(measure_node)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  #pcl_ros
  roslib
  roscpp
  sensor_msgs
)

find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory (src SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})