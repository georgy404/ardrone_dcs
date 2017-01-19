cmake_minimum_required(VERSION 2.8.3)
project(ardrone_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	geometry_msgs
	nav_msgs
	roscpp
	std_msgs
)

add_message_files(
	FILES
	Mode.msg
	NavPose.msg
	Mission.msg
)

add_service_files(
    FILES
    AddMarker.srv
    DelMarker.srv
    ClearMarkers.srv
)

generate_messages(
	DEPENDENCIES
	ardrone_msgs
	geometry_msgs
	nav_msgs
	std_msgs
)

catkin_package(
#	INCLUDE_DIRS include
#   LIBRARIES ardrone_msgs
#   CATKIN_DEPENDS roscpp rospy std_msgs
#   DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
