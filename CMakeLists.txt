cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_kdl_tools)
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-std=c++11)

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
  eigen_conversions
  kdl_conversions
  trac_ik_lib
)

find_package(orocos_kdl REQUIRED)

find_package(OROCOS-RTT)

if(OROCOS-RTT_FOUND)
    message(STATUS "Building with OROCOS")
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
    orocos_use_package(rtt_roscomm)
    orocos_use_package(rtt_rosparam)
    orocos_use_package(rtt_rosclock)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
)

if(OROCOS-RTT_FOUND)
  orocos_library(${PROJECT_NAME}
    src/tools.cpp
    src/chaincogsolver.cpp
    src/chainjnttojacdotsolver.cpp
    src/chain_utils_base.cpp
    src/chain_utils.cpp
    )

  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

  orocos_generate_package(
    INCLUDE_DIRS include
    DEPENDS rtt_rosclock rtt_rosparam
    )

  orocos_install_headers(
    include/${PROJECT_NAME}/tools.hpp
    include/${PROJECT_NAME}/chainjnttojacdotsolver.hpp
    include/${PROJECT_NAME}/chain_utils_base.hpp
    include/${PROJECT_NAME}/chain_utils.hpp
    include/${PROJECT_NAME}/mqueue_connpolicy.hpp
    include/${PROJECT_NAME}/chaincogsolver.hpp
    )
else()
  catkin_package(INCLUDE_DIRS include)

  add_library(${PROJECT_NAME}
    src/tools.cpp
    src/chaincogsolver.cpp
    src/chainjnttojacdotsolver.cpp
    src/chain_utils_base.cpp
    src/chain_utils.cpp
    )

  message(WARNING "OROCOS not found, compiling with NO_OROCOS flag")
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS NO_OROCOS)

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
  )
endif()

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
   ${trac_ik_lib_LIBRARIES}
)

# if(CATKIN_ENABLE_TESTING)
#     catkin_add_gtest(test_jdot tests/test_jdot.cpp src/chainjnttojacdotsolver.cpp)
#     target_link_libraries(test_jdot ${orocos_kdl_LIBRARIES})
#
#     catkin_add_gtest(test_chain_utils tests/test_chain_utils.cpp src/chain_utils.cpp)
#     target_link_libraries(test_chain_utils ${catkin_LIBRARIES} ${PROJECT_NAME})
# endif()
