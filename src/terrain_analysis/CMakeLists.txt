cmake_minimum_required(VERSION 3.0.2)
project(terrain_analysis)

set(CMAKE_CXX_FLAGS "-std=c++14")
add_definitions(-DUSE_GRIDMAP)

set(
  CATKIN_PACKAGE_DEPENDENCIES
    tf
    roscpp
    rospy
    pcl_conversions
    # msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    grid_map_core
    grid_map_ros
    grid_map_msgs
    message_runtime
    message_generation
)

find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_generation
)

find_package(PCL REQUIRED)


add_message_files(
  FILES
  CustomMsg.msg
  CustomPoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(terrain_analysis_node src/terrain_analysis.cpp)
add_dependencies(terrain_analysis_node terrain_analysis_gencpp)
target_link_libraries(terrain_analysis_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})