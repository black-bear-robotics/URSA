cmake_minimum_required(VERSION 2.8.3)
project(ursa)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	message_generation
	std_msgs
	geometry_msgs
	nav_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
	FILES
    PathfinderTarget.srv
)


generate_messages(
    DEPENDENCIES
    std_msgs
	geometry_msgs
	nav_msgs
)


