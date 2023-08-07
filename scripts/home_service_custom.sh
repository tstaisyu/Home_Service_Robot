#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch my_robot world.launch" & 
sleep 5
xterm  -e  " roslaunch localization amcl.launch" & 
sleep 5
xterm  -e  " roslaunch localization rviz.launch " &
sleep 5
xterm  -e  " rosrun add_markers add_markers_custom" &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects_custom" 
