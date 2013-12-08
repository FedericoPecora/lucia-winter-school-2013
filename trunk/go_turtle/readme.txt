

Steps to run the go_turtle_node (for goto --> do_action):

=============================================================
-With real robot-
=============================================================

1- Copy the go_turtle package into the catkin workspace and build the package.

2- Open a terminal1:
ssh turtlebot@TURTLEBOT_ADDRESS
passward: turtlebot
roslaunch turtlebot_bringup minimal.launch

3- Open a terminal2: (load map)
ssh turtlebot@TURTLEBOT_ADDRESS
passward: turtlebot
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/Maps/t101_map.yaml  (ph2_map.yaml can also be used for experiments in peishome)

4- Open a terminal3 (optional):
roslaunch turtlebot_rviz_launchers view_navigation.launch

5- Open a terminal4:
rosrun go_turtle go_turtle_node


=============================================================
-In simulation-
=============================================================

1- copy the map files in one folder (.pmg and .yaml).

2- Open a terminal1:
roslaunch turtlebot_gazebo turtlebot_empty_world.launch

3- Open a terminal2: (load map)
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=PATH_OF_MAP_FILE (e.g. /home/user/catkin_ws/src/go_turtle/maps/t101_map.yaml)

4- Open a terminal3 (optional):
roslaunch turtlebot_rviz_launchers view_navigation.launch

5- Open a terminal4:
rosrun go_turtle go_turtle_node


=============================================================
go_turtle_node Subscribes:
=============================================================

1- A topic of type "test_topic/actions" and name "/move_action" that contains the following four floats:

float64 x
float64 y
float64 action
float64 timeout

Where:
x and y are the x,y pose in the map in meters.
timeout is the deadline time for task execution in seconds.
action = 1,2,-1,-2,0,3 where;
1  = goto xy and rotate once anti-clockwise
-1 = goto xy and rotate once clockwise
2  = goto xy and rotate twice anti-clockwise
-2 = goto xy and rotate twice clockwise
0  = only goto xy
3  = only rotate three times

=============================================================
go_turtle_node Publishes:
=============================================================

1- A topic of type "std_msgs/String" and name "action_feedback" that contains the following feedbacks:

ACTIVE
SUCCEEDED
TIME_OUT
ABORTED
RECALLED
REJECTED
LOST

=============================================================
go_turtle_node depends on:
=============================================================

actionlib
roscpp
std_msgs
tf

=============================================================
			EOF
=============================================================
