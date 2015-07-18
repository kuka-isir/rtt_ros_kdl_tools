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
find_package(orocos_kdl REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
)

orocos_library(tools
   src/tools.cpp
   src/chainjnttojacdotsolver.cpp
)

target_link_libraries(tools
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
)


orocos_install_headers(DIRECTORY
include/${PROJECT_NAME}
)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
  )
