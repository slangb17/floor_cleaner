cmake_minimum_required(VERSION 2.8.3)
project(floor_cleaner)

set(ROS_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  roscpp 
  interactive_markers 
  visualization_msgs 
  geometry_msgs   
  tf2
  tf2_ros 
  std_msgs
  move_base_msgs 
  actionlib
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES floor_cleaner
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(move_base src/move_base.cpp)
add_executable(path_maker src/path_maker.cpp)
add_executable(path_base src/path_base.cpp)
add_executable(node_maker src/node_maker.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(move_base
  ${catkin_LIBRARIES}
)

target_link_libraries(path_maker
  ${catkin_LIBRARIES}
)

target_link_libraries(path_base
  ${catkin_LIBRARIES}
)

target_link_libraries(node_maker
  ${catkin_LIBRARIES}
)

