cmake_minimum_required(VERSION 2.8.3)
project(aubo_kinematics)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core
  moveit_ros_planning pluginlib tf_conversions)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aubo_i5_kin aubo_i5_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS boost
)


###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_library(aubo_i5_kin src/aubo_kin.cpp)
set_target_properties(aubo_i5_kin PROPERTIES COMPILE_DEFINITIONS "AUBO_I5_PARAMS")


add_library(aubo_i5_moveit_plugin src/aubo_moveit_plugin.cpp)
set_target_properties(aubo_i5_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "AUBO_I5_PARAMS")
target_link_libraries(aubo_i5_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  aubo_i5_kin)



#############
## Install ##
#############

install(TARGETS aubo_i5_kin aubo_i5_moveit_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES aubo_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

