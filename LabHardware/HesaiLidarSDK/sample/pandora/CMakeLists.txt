cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED )
find_package(PCL REQUIRED )
find_package(OpenCV REQUIRED )


set (CMAKE_CXX_STANDARD 11)

# When sdk compiled with Camera_Enable , This definition must be add to cmakelists
add_definitions("-DHESAI_WITH_CAMERA")

macro(FIND_IN_PATH TARGET_H TARGET_H_IN_PATH)
        find_path(
                ${TARGET_H_IN_PATH}
                NAMES ${TARGET_H}
                PATHS "${CMAKE_SYSTEM_INCLUDE_PATH}/include"
        )

        #message(STATUS "System include == ${CMAKE_SYSTEM_INCLUDE_PATH}")
        message(STATUS "\"${TARGET_H}\" in path == ${${TARGET_H_IN_PATH}}")
endmacro()

FIND_IN_PATH("hesaiLidarSDK.h" HAVE_HESAILIDARSDK_H_IN_PATH)
FIND_IN_PATH("point_types.h" HAVE_POINT_TYPES_H_IN_PATH)
message(STATUS "\"hesaiLidarSDK.h\" in path  == ${HAVE_HESAILIDARSDK_H_IN_PATH}")
message(STATUS "\"point_types.h\" in path  == ${HAVE_POINT_TYPES_H_IN_PATH}")

if((NOT ${HAVE_HESAILIDARSDK_H_IN_PATH} STREQUAL "") AND (NOT ${HAVE_POINT_TYPES_H_IN_PATH} STREQUAL ""))
	include_directories(
	#	${CMAKE_CURRENT_SOURCE_DIR}/../../include	# Use a command line parameter to provide the include files directory is feasible
		${HAVE_HESAILIDARSDK_H_IN_PATH}
		${HAVE_POINT_TYPES_H_IN_PATH}
		${Boost_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
	)
endif()

add_executable(pandora
	pandora.cc
)

message (STATUS "HesaiLidarSDK_LIBS before initialized == ${HesaiLidarSDK_LIBS}")
find_library(HesaiLidarSDK_LIBS hesaiLidarSDK)
message (STATUS "HesaiLidarSDK_LIBS after initialized == ${HesaiLidarSDK_LIBS}")

if( NOT HesaiLidarSDK_LIBS STREQUAL "HesaiLidarSDK_LIBS-NOTFOUND")
	target_link_libraries(pandora
		hesaiLidarSDK
		${Boost_LIBRARIES}
		${PCL_LIBRARIES}
		${OpenCV_LIBS}
		jpeg
	)
elseif(HesaiLidarSDK_LIBS STREQUAL "HesaiLidarSDK_LIBS-NOTFOUND")
	message( FATAL_ERROR "HesaiLidarSDK_LIBS not found, CMake will exit." )
endif()

