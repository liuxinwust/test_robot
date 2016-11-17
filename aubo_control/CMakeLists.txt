cmake_minimum_required(VERSION 2.8.3)
project(aubo_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aubo_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs aubo_msgs tf
#  DEPENDS system_lib
)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( 
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message(STATUS " CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH}")

add_subdirectory(src/joint_control_pcan)
add_subdirectory(src/joint_state_sub)
add_subdirectory(src/real_state_rviz)




