cmake_minimum_required(VERSION 2.8.3)
project(ardrone_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    interactive_markers
    roscpp
    rviz
    std_msgs
    tf
    visualization_msgs
    ardrone_msgs
    ardrone_autonomy
)
find_package(Qt4 REQUIRED COMPONENTS 
    QtCore
    QtGui
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ardrone_rviz_plugins
)

include(
    ${QT_USE_FILE}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

qt4_wrap_cpp(QT_MOC
    ## Control panel
    include/control_panel/control_panel.h
    include/control_panel/targets_table.h
    include/control_panel/table_delegate.h
    include/control_panel/ros_thread_cp.h

    ## Teleop panel
    include/teleop_panel/teleop_panel.h
    include/teleop_panel/ros_thread_tp.h

    ## Info panel
    include/info_panel/info_panel.h
    include/info_panel/ros_thread_ip.h
)

set(SOURCES
    ## Control panel
    include/control_panel/control_panel.h
    src/control_panel/control_panel.cpp

    include/control_panel/targets_table.h
    src/control_panel/targets_table.cpp

    include/control_panel/table_delegate.h
    src/control_panel/table_delegate.cpp

    include/control_panel/ros_thread_cp.h
    src/control_panel/ros_thread_cp.cpp

    ## Teleop panel
    include/teleop_panel/teleop_panel.h
    src/teleop_panel/teleop_panel.cpp

    include/teleop_panel/ros_thread_tp.h
    src/teleop_panel/ros_thread_tp.cpp

    ## Info panel
    include/info_panel/info_panel.h
    src/info_panel/info_panel.cpp

    include/info_panel/ros_thread_ip.h
    src/info_panel/ros_thread_ip.cpp
)

add_library(
    ${PROJECT_NAME}
    ${SOURCES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

## Markers server
add_executable(markers_server
    include/markers_server/markers_server.h
    src/markers_server/markers_server.cpp

    include/markers_server/line_strip.h
    src/markers_server/line_strip.cpp
)
target_link_libraries(markers_server
    ${catkin_LIBRARIES}
)
