import math, tf
from geometry_msgs.msg import Quaternion

def point_to_grid(point, grid):
	"""Returns an (x,y) tuple from a Point (e.g. from a Pose) and OccupancyGrid"""
	origin = grid.info.origin
	res = grid.info.resolution
	x = (point.x - origin.position.x) // res
	y = (point.y - origin.position.y) // res
	return (int(x),int(y))


def pfval(value):
	"""Transform OccupancyGrid values to pathfinding values
	OccupancyGrid and the pathfinding library treat grid values differently.
	OccupancyGrid uses -1 for unknown and 0-100 as an increasing probability
	of a cell being occupied.
	The pathfinding library uses 0 or -1 for an impassible obstacle, and
	higher numbers as a cost to traverse."""
	if value == -1:
		return 70 #avoid the unknown, but don't rule it out
	elif value == 0:
		return 1 #Lowest possible traversible cost
	elif value > 75:
		return 0 #Treat as obstacle if there is reasonable confidence
	else:
		return value #arbitrary weight


def path_angle(node1, node2):
	"""Returns the angle between two pathfinding nodes.
	e.g. node1 is (0,0) and node2 is (1,1) the angle is 45 degrees."""
	dx = node2[0] - node1[0]
	dy = node2[1] - node1[1]
	angle = math.degrees(math.atan2(dy, dx))
	return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))