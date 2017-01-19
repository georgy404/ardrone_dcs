cmake_minimum_required(VERSION 2.8.3)
project(ardrone_pid)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  dynamic_reconfigure
  control_toolbox
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

generate_dynamic_reconfigure_options(
  cfg/PidParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ardrone_pid
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ardrone_pid
  include/ardrone_pid.h 
  src/ardrone_pid.cpp 
  src/main.cpp
)

add_dependencies(ardrone_pid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(ardrone_pid ${catkin_LIBRARIES})
