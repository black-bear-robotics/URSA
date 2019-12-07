## Installing
1. Clone into `src/` subdirectory of catkin workspace.

2. Ensure catkin workspace is added to ROS package path, by adding `source ~/catkin_workspace_path/devel/setup.bash`.


## Running individual nodes
```
roscore
rosrun ursa pathfinder
```
Eventually we'll want to setup a launch file so roslaunch and bring them all up at once.
