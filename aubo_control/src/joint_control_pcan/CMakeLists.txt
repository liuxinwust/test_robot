# find libpcan
FIND_PACKAGE(pcan REQUIRED)

# find libcontrol
FIND_PACKAGE(Comm REQUIRED)

INCLUDE_DIRECTORIES(${libcommAPI_INCLUDE_DIRS})
ADD_EXECUTABLE(joint_control_pcan joint_control_pcan.cpp)
TARGET_LINK_LIBRARIES(joint_control_pcan ${catkin_LIBRARIES} ${libcommAPI_LIBS} ${LIBPCAN_LIB} -DNO_RT)
