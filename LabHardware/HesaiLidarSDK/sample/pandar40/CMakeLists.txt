cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED )
find_package(PCL REQUIRED )


set (CMAKE_CXX_STANDARD 11)

find_file(HAVE_HESAILIDARSDK_H_ PATHS "/usr/local/include" "/usr/include")
message(STATUS "HAVE_HESAILIDARSDK_H_ == ${HAVE_HESAILIDARSDK_H_}")	# might be HAVE_HESAILIDARSDK_H_-NOTFOUND

macro(FIND_IN_PATH TARGET_H TARGET_H_IN_PATH)
	find_path(
		${TARGET_H_IN_PATH}
		NAMES ${TARGET_H}
		PATHS "${CMAKE_SYSTEM_INCLUDE_PATH}/include"
	)

	#message(STATUS "System include == ${CMAKE_SYSTEM_INCLUDE_PATH}")
	message(STATUS "\"${TARGET_H}\" in path == ${${TARGET_H_IN_PATH}}")
endmacro()

#set(CMAKE_REQUIRED_FLAGS "-std=c++11")
#set(CMAKE_REQUIRED_INCLUDES "/usr/local/include") 

include(CheckIncludeFileCXX)
#include(CheckIncludeFile)
#include(CheckIncludeFiles)
macro(CHECK_INCLUDE_FILE_CXX_ERROR INCLUDE_FILE HAVE_FILE)
    CHECK_INCLUDE_FILE_CXX(${INCLUDE_FILE} ${HAVE_FILE})
    if(NOT ${HAVE_FILE})
        unset(HAVE_FILE CACHE)
        message( FATAL_ERROR "${INCLUDE_FILE} is not found" )
    endif()
endmacro()

#CHECK_INCLUDE_FILE_CXX("hesaiLidarSDK.h" HAVE_HESAILIDARSDK_H "-I/usr/local/include")
CHECK_INCLUDE_FILE_CXX("hesaiLidarSDK.h" HAVE_HESAILIDARSDK_H)
#CHECK_INCLUDE_FILE("hesaiLidarSDK.h" HAVE_HESAILIDARSDK_H)
#CHECK_INCLUDE_FILES("hesaiLidarSDK.h" HAVE_HESAILIDARSDK_H)
message(STATUS "HAVE_HESAILIDARSDK_H == ${HAVE_HESAILIDARSDK_H}")
message(STATUS "HESAILIDARSDK_H_PATH == ${CMAKE_REQUIRED_INCLUDES}")
unset(HAVE_HESAILIDARSDK_H CACHE)

#CHECK_INCLUDE_FILE_CXX_ERROR("hesaiLidarSDK.h" HAVE_HESAILIDARSDK_H)
#CHECK_INCLUDE_FILE_CXX_ERROR("point_types.h" HAVE_POINT_TYPES_H)

FIND_IN_PATH("hesaiLidarSDK.h" HAVE_HESAILIDARSDK_H_IN_PATH)
FIND_IN_PATH("point_types.h" HAVE_POINT_TYPES_H_IN_PATH)
message(STATUS "\"hesaiLidarSDK.h\" in path  == ${HAVE_HESAILIDARSDK_H_IN_PATH}")
message(STATUS "\"point_types.h\" in path  == ${HAVE_POINT_TYPES_H_IN_PATH}")

if((NOT ${HAVE_HESAILIDARSDK_H_IN_PATH} STREQUAL "") AND (NOT ${HAVE_POINT_TYPES_H_IN_PATH} STREQUAL ""))
include_directories(
#	${CMAKE_CURRENT_SOURCE_DIR}/../../include       # Use a command line parameter to provide the include files directory is feasible
	${HAVE_HESAILIDARSDK_H_IN_PATH}
	${HAVE_POINT_TYPES_H_IN_PATH}
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
endif()

message (STATUS "Debug include dir == ${CMAKE_CURRENT_SOURCE_DIR}/../../include")
message (STATUS "Debug boost include dir == ${Boost_INCLUDE_DIRS}")
message (STATUS "Debug pcl include dir ==${PCL_INCLUDE_DIRS}")
message (STATUS "Debug hesaiLidarSDK dir ==${hesaiLidarSDK}")
message (STATUS "Debug boost library dir == ${Boost_LIBRARIES}")
message (STATUS "Debug pcl library dir == ${PCL_LIBRARIES}")
message (STATUS "_____________________________________________________________________\
")

add_executable(pandar40
	pandar40.cc
)

message (STATUS "HesaiLidarSDK_LIBS before initialized == ${HesaiLidarSDK_LIBS}")

find_library(HesaiLidarSDK_LIBS hesaiLidarSDK)

message (STATUS "HesaiLidarSDK_LIBS after initialized == ${HesaiLidarSDK_LIBS}")
if( NOT HesaiLidarSDK_LIBS STREQUAL "HesaiLidarSDK_LIBS-NOTFOUND")
#  message( SEND_ERROR "You can't do that" )
	target_link_libraries(pandar40
		hesaiLidarSDK
		${Boost_LIBRARIES}
		${PCL_LIBRARIES}
	)
elseif(HesaiLidarSDK_LIBS STREQUAL "HesaiLidarSDK_LIBS-NOTFOUND")
  message( FATAL_ERROR "HesaiLidarSDK_LIBS not found, CMake will exit." )
endif()


