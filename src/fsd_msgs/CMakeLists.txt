cmake_minimum_required(VERSION 3.0.2)
project(fsd_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	genmsg
	std_msgs
	roscpp
	geometry_msgs
)

add_message_files(
	FILES
	Cone.msg
	ControlCommand.msg
	VehicleState.msg
	Map.msg
	CANFrame.msg
	StateFromRTK.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
	std_msgs
	geometry_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${roscpp_INCLUDE_DIRS}
)
