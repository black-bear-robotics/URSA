#!/usr/bin/env python
# Broadcast dummy data for the pathfinding node to catch.

import rospy
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from geometry_msgs.msg import Pose, PoseStamped


def send_grid(pub):
	# Send a basic OccupancyGrid for the pathfinder node to unpack and read
	meta = MapMetaData()
	meta.width = 10
	meta.height = 10
	meta.resolution = 1
	
	origin = Pose()
	origin.position.x = 0
	origin.position.z = 0
	origin.position.y = 0
	origin.orientation.x = float(0)
	origin.orientation.y = float(0)
	origin.orientation.z = float(0)
	origin.orientation.w = float(0)
	meta.origin = origin

	grid = OccupancyGrid()
	grid.info = meta
	grid.data = [0,0,0,0,0,0,0,0,0,99,99,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,99,99,99,0,0,0,0,0,0,0,0,99,99,0,0,0,0,0,0,99,99,0,0,0,0,0,0,0,0,99,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

	pub.publish(grid)


def send_robot_pos(pub):
	pose = PoseStamped()
	pose.header.frame_id = "map"
	pose.pose.position.x = 0.33
	pose.pose.position.y = 0.22
	pose.pose.position.z = 0.0
	pose.pose.orientation.x = 0
	pose.pose.orientation.y = 0
	pose.pose.orientation.z = 0
	pose.pose.orientation.w = 0
	pub.publish(pose)


def main():
	rate = rospy.Rate(0.1)
	pub_grid = rospy.Publisher('projected_map', OccupancyGrid, latch=True, queue_size=10)	
	pub_pos = rospy.Publisher('orb_slam2_stereo/pose', PoseStamped, latch=True, queue_size=10)	
	while not rospy.is_shutdown():
		send_grid(pub_grid)
		send_robot_pos(pub_pos)
		rate.sleep()


if __name__ == '__main__':
    try:
		rospy.init_node('test_path')
		main()
    except rospy.ROSInterruptException:
        pass
    
