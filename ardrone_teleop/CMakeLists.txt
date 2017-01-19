cmake_minimum_required(VERSION 2.8.3)
project(ardrone_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)
find_package(Curses REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ardrone_teleop
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CURSES_INCLUDE_DIR})

## Declare a C++ executable
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})
target_link_libraries(keyboard_teleop ${CURSES_LIBRARIES})
