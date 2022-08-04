cmake_minimum_required(VERSION 2.8.3)
project(gtsam_ros_package)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
)

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS *boost libraries here*) 
include_directories(${Boost_INCLUDE_DIRS}) 

###### GTSAM STUFF STARTS HERE

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
#include(GtsamBuildTypes) # Load build type flags and default to Debug mode
#include(GtsamTesting)    # Easy functions for creating unit tests and scripts
#include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
#include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
###################################################################################
# Build static library from common sources
#set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
#add_library(${CONVENIENCE_LIB_NAME} STATIC include/car/car_lib.h src/car_lib.cpp)
#target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)

###### GTSAM STUFF ENDS HER

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ImuFactorsExample.cpp)

target_link_libraries(${PROJECT_NAME}_node
  gtsam
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_gps_node src/ImuFactorsGPSExample.cpp)

target_link_libraries(${PROJECT_NAME}_gps_node
  gtsam
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)