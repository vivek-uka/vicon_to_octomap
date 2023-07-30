cmake_minimum_required(VERSION 2.8.3)
project(vicon_to_octomap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
if (octomap_FOUND)
  message(STATUS "Package 'octomap' found.")
else ()
  message(FATAL_ERROR "Package 'octomap' not found.")
endif ()

catkin_package(
  LIBRARIES exploration
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES ${SOURCES}
  src/convexhull.cpp
)

add_executable(convex_node src/demo.cpp ${SOURCES})
target_link_libraries(convex_node ${catkin_LIBRARIES})

add_executable(generate_octomap src/genocto.cpp ${SOURCES})
target_link_libraries(generate_octomap ${catkin_LIBRARIES})