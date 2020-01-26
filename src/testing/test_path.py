#!/usr/bin/env python
# Broadcast dummy data for the pathfinding node to catch.

import rospy, math, tf2_ros
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped
from tf_conversions import transformations
from ursa.srv import PathfinderTarget


def send_grid(pub):
	# Send a basic OccupancyGrid for the pathfinder node to unpack and read
	meta = MapMetaData()
	meta.width = 10
	meta.height = 10
	meta.resolution = 1
	
	origin = Pose()
	origin.position.x = -3
	origin.position.z = 0
	origin.position.y = -3
	origin.orientation = Quaternion(*transformations.quaternion_from_euler(0, 0, 0))
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
	pose.pose.orientation = Quaternion(*transformations.quaternion_from_euler(0, 0, 0))
	pub.publish(pose)


def test_change_target():
	rospy.wait_for_service('pathfinder_target')
	srv = rospy.ServiceProxy('pathfinder_target', PathfinderTarget)
	try:
		srv(Point(0,0,0))
	except:
		return


def send_static_tf(broadcaster):
	ts = TransformStamped()
	ts.header.stamp = rospy.Time.now()
	ts.header.frame_id = "world"
	ts.child_frame_id = "map"
	ts.transform.translation.x = float(0.0)
	ts.transform.translation.y = float(0.0)
	ts.transform.translation.z = float(0.0)
	ts.transform.rotation = Quaternion(*transformations.quaternion_from_euler(0, 0, 0))
	broadcaster.sendTransform(ts)


def main():
	rate = rospy.Rate(0.1)
	pub_grid = rospy.Publisher('projected_map', OccupancyGrid, latch=True, queue_size=10)	
	pub_pos = rospy.Publisher('orb_slam2_stereo/pose', PoseStamped, latch=True, queue_size=10)
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	while not rospy.is_shutdown():
		send_grid(pub_grid)
		send_robot_pos(pub_pos)
		send_static_tf(broadcaster)
		rate.sleep()


if __name__ == '__main__':
    try:
		rospy.init_node('test_path')
		main()
    except rospy.ROSInterruptException:
        pass
    
