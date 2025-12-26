#!/bin/bash

#####################################################################
# This script attempts to identify Issues.
#####################################################################

echo "*****************************************************************"
echo "*** TRY to blindly terminate the gazebo/moveit/ros executable.***"
echo "*****************************************************************"

pkill -f -9 'gazebo'
sleep 1;
pkill -f -9 'moveit'
sleep 1;
pkill -f -9 'ros'
sleep 1;

#####################################################################
# echo "*** Below, to see the current gazebo / ros process."
#####################################################################
echo ""
ps aux | grep gazebo
ps aux | grep ros
sleep 1;

#####################################################################
# Try to get a more detailed report from ros2 doctor."
#####################################################################
# ros2 doctor --report
