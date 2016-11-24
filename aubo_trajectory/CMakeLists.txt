cmake_minimum_required(VERSION 2.8.3)
project(aubo_trajectory)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS aubo_msgs std_msgs sensor_msgs control_msgs actionlib_msgs actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)


find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  CATKIN_DEPENDS
    aubo_msgs 
    std_msgs
    sensor_msgs
    control_msgs
    actionlib
    moveit_core
    moveit_ros_planning_interface
    interactive_markers

)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_subdirectory(trajectory_client)
add_subdirectory(trajectory_gen)
add_subdirectory(trajectory_goal)



