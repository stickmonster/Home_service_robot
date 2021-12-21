
#!/bin/sh


xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/my_world.world" &

sleep 7

xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/World/my_world.yaml" &

sleep 7

xterm -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" & 

sleep 7

xterm -e "source devel/setup.bash; rosrun add_markers add_markers" &

sleep 7

xterm -e "source devel/setup.bash; rosrun pick_objects pick_objects"


