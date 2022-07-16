#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/my_robot/worlds/eric.world

xterm  -e "cd ../.. && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e "cd ../.. && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/maps/eric_map.yaml initial_pose_x:=5.206 initial_pose_y:=3.87 initial_pose_a:=-1.5708" &
sleep 5
xterm  -e "cd ../.. && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
