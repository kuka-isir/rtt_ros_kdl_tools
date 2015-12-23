cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_kdl_tools)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  geometry_msgs
  kdl_parser
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  trajectory_msgs
)

find_package(orocos_kdl REQUIRED)

find_package(OROCOS-RTT)

if(OROCOS-RTT_FOUND)
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
endif()

#catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
)

if(OROCOS-RTT_FOUND)
  orocos_library(${PROJECT_NAME}
    src/tools.cpp
    src/chainjnttojacdotsolver.cpp
    )
else()
  add_library(${PROJECT_NAME}
    src/tools.cpp
    src/chainjnttojacdotsolver.cpp
    )
endif()

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
)

if(OROCOS-RTT_FOUND)

  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

  orocos_generate_package(
    INCLUDE_DIRS include
    DEPENDS rtt_rosclock rtt_rosparam
    )

  orocos_install_headers(  
    include/${PROJECT_NAME}/tools.hpp
    include/${PROJECT_NAME}/chainjnttojacdotsolver.hpp
    include/${PROJECT_NAME}/chain_utils.hpp
    include/${PROJECT_NAME}/mqueue_connpolicy.hpp
    )

else()
  message(WARNING "OROCOS not found, compiling with NO_OROCOS flag")
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS NO_OROCOS)
endif()

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_jdot tests/test_jdot.cpp src/chainjnttojacdotsolver.cpp)
    target_link_libraries(test_jdot ${orocos_kdl_LIBRARIES})
    
    catkin_add_gtest(test_chain_utils tests/test_chain_utils.cpp src/chain_utils.cpp)
    target_link_libraries(test_chain_utils ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
