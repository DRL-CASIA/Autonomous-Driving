cmake_minimum_required(VERSION 2.8.3)
project(dbw_coolhigh_can)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  can_msgs
  coolhigh_dbw_msgs
  dbc
  geometry_msgs
  nodelet
  roscpp
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
  src/nodelet.cpp
  src/DbwNode.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_dbw_node
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_dbw_node
  ${PROJECT_NAME}
)
set_target_properties(${PROJECT_NAME}_dbw_node PROPERTIES OUTPUT_NAME dbw_node PREFIX "")

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_dbw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

