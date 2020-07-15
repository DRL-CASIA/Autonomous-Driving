cmake_minimum_required(VERSION 2.8.3)
project(remote_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  coolhigh_dbw_msgs
  dbc
  nodelet
  sensor_msgs
  std_msgs
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE
    include/
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/RemoteControl.cpp
  src/RemoteControl.h
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)


