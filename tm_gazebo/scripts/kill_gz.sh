#!/bin/bash

##########################################################
# This script should be to kill the GUI Gazebo executables.
##########################################################

echo ""
echo "*** Gazebo= `ign gazebo --versions`"
# cat /etc/lsb-release
# env | grep ros
echo "***********************************************"
echo "*** Check the gazebo process before killing it."
echo "***********************************************"
ps aux | grep ign
sleep 1;

###############################################################################
# To kill the GUI Gazebo executables. (you can change "ign gazebo" to "gazebo")
###############################################################################
pkill -f -9 'ign gazebo'

echo ""
echo "**********************************************************************"
echo "*** To kill ign gazebo process done. Check the current gazebo process."
echo "**********************************************************************"
sleep 1;
echo ""
ps aux | grep ign

# pkill -f -9 'ros'
# echo "*** Below, to see the current ros process."
# ps aux | grep ros
