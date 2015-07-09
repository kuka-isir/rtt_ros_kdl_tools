cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_kdl_tools)

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  geometry_msgs
  kdl_parser
  roscpp
  rtt_ros
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  trajectory_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

orocos_library(${PROJECT_NAME}
   src/tools.cpp
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

orocos_install_headers(
include/${PROJECT_NAME}/tools.hpp
)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS kdl rtt_ros
  )
