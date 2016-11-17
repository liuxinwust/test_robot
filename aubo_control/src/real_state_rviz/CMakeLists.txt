## Declare a cpp executable
add_executable(real_state_rviz real_state_rviz.cpp)
target_link_libraries(real_state_rviz ${catkin_LIBRARIES})

install(TARGETS real_state_rviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

