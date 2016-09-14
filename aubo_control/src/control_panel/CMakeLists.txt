# find pthread
find_package(Threads)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed  
set(CMAKE_AUTOMOC ON)

set(CMAKE_PREFIX_PATH "/opt/Qt5.6.1/5.6/gcc_64/lib/cmake")


# Find the QtWidgets library
find_package(Qt5Widgets)

# Add the include directories for the Qt 5 Widgets module to
# the compile lines.
include_directories(${Qt5Widgets_INCLUDE_DIRS} ${libOURAPI_INCLUDE_DIRS})

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

# Add compiler flags for building executables (-fPIE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

qt5_wrap_ui(ui_FILES mainwindow.ui)

add_executable(control_panel 
	       main.cpp
               mainwindow.h mainwindow.cpp
	       ${ui_FILES})

#Link the helloworld executable to the Qt 5 widgets library.
target_link_libraries(control_panel Qt5::Widgets ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${libOURAPI_LIBS})
