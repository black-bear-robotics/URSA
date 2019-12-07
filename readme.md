## Installing
1. Clone into `src/` subdirectory of catkin workspace.

2. Ensure catkin workspace is added to ROS package path, by adding `source ~/catkin_workspace_path/devel/setup.bash`.


## Running individual nodes
```
roscore
rosrun ursa pathfinder
```

## Launch Files
### Launch all URSA nodes
The ursa.launch file brings up all custom Python nodes.
```
roslaunch ursa/launch/ursa.launch
```
### Launch robot
This top-level file brings up everything in ursa.launch as well as every external node the robot depends on. This is our one-stop start script.
```
roslaunch ursa/launch/robot.launch
```
