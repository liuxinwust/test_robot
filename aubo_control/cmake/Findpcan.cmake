# - Try to find libpcan
# Once done this will define
#
# LIBPCAN_FOUND - system has libpcan
# LIBPCAN_INCLUDE_DIR - the libpcan include directory
# LIBPCAN_LIB - link these to use libpcan

find_path(LIBPCAN_INCLUDE_DIR
	NAMES libpcan.h
	PATHS /usr/include
	      /usr/local/include
	      $ENV{INCLUDE}
)

find_library(LIBPCAN_LIB
	NAMES pcan
	PATHS /usr/lib
	      /usr/local/lib
)

if(LIBPCAN_INCLUDE_DIR)
	message(STATUS "Found libpcan include dir: ${LIBPCAN_INCLUDE_DIR}")
else(LIBPCAN_INCLUDE_DIR)
	message(STATUS "Could NOT find libpcan headers.")
endif(LIBPCAN_INCLUDE_DIR)

if(LIBPCAN_LIB)
	message(STATUS "Found libpcan library: ${LIBPCAN_LIB}")
else(LIBPCAN_LIB)
	message(STATUS "Could NOT find libpcan library.")
endif(LIBPCAN_LIB)

if(LIBPCAN_INCLUDE_DIR AND LIBPCAN_LIB)
	set(LIBPCAN_FOUND TRUE)
else(LIBPCAN_INCLUDE_DIR AND LIBPCAN_LIB)
	set(LIBPCAN_FOUND FALSE)
	if(LibPCAN_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find libpcan. Please install libpcan.")
	endif(LibPCAN_FIND_REQUIRED)
endif(LIBPCAN_INCLUDE_DIR AND LIBPCAN_LIB)
