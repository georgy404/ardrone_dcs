cmake_minimum_required(VERSION 2.8.3)
project(ardrone_mission_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Catkin specific configuration
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ardrone_mission_planner
)

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(mission_planner
    include/mission_planner.h
    src/mission_planner.cpp
    src/main.cpp
)

add_dependencies(mission_planner
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(mission_planner
    ${catkin_LIBRARIES}
)
