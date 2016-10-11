# - Try to find libOURAPI
# Once done this will define
#
# libOURAPI_FOUND - system has libOURAPI
# libOURAPI_INCLUDE_DIRS - the libOURAPI include directories
# libOURAPI_LIBS - link these to use libOURAPI


find_path(libcommAPI_INCLUDE_DIR
	NAMES jointcontrolapi.h
	PATHS ${PROJECT_SOURCE_DIR}/include/comm
	      $ENV{INCLUDE}
)


if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	find_library(libI5-CAN
		NAMES I5-CAN
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib64
	)
else()
        find_library(libI5-CAN
                NAMES I5-CAN
                PATHS ${PROJECT_SOURCE_DIR}/lib/lib32
        )
endif()


set(libcommAPI_INCLUDE_DIRS ${libcommAPI_INCLUDE_DIR})

set(libcommAPI_LIBS ${libI5-CAN})


if(libcommAPI_INCLUDE_DIRS)
	message(STATUS "Found CommAPI include dir: ${libcommAPI_INCLUDE_DIRS}")
else(libcommAPI_INCLUDE_DIRS)
	message(STATUS "Could NOT find CommAPI headers.")
endif(libcommAPI_INCLUDE_DIRS)


if(libcommAPI_LIBS)
	message(STATUS "Found CommAPI library: ${libcommAPI_LIBS}")
else(libcommAPI_LIBS)
	message(STATUS "Could NOT find CommAPI library.")
endif(libcommAPI_LIBS)

if(libcommAPI_INCLUDE_DIRS AND libcommAPI_LIBS)
	set(libAPI_FOUND TRUE)
else(libcommAPI_INCLUDE_DIRS AND libcommAPI_LIBS)
	set(libAPI_FOUND FALSE)
	if(libcommAPI_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find CommAPI.")
	endif(libcommAPI_FIND_REQUIRED)
endif(libcommAPI_INCLUDE_DIRS AND libcommAPI_LIBS)
