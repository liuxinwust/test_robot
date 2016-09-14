add_executable(trajectory_gen src/trajectory_gen.cpp)
target_link_libraries(trajectory_gen ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS trajectory_gen DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
