#!/bin/bash

#########################################################
# This script demo the tm5-900 cobot on the Gazebos GUI.#
#########################################################

# In Bash directory path
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo ${SCRIPTS_DIR}

# cd "${SCRIPTS_DIR}"/..
. "$SCRIPTS_DIR"/kill_all.sh

# Source the main ROS setup file (to workspace )
cd ../../..
source /opt/ros/humble/setup.bash
echo ""
echo "******* ROS_DISTRO= $ROS_DISTRO *******"
echo ""
echo "********************************************************"
echo "Prepare to run TM5-900 Cobot on Gazebo GUI and RVIZ2 ..."
echo "********************************************************"

# Source your workspace setup file (if you have one)
source ./install/setup.bash
echo ""
Wait_INTERVAL="5";
echo "Wait... for ${Wait_INTERVAL} secondes"
echo ""
echo "**********************************************************************"
echo "If tm5-900 demo works fine, press CTRL + C to shut everything down... "
echo "**********************************************************************"
CURRENT_TIME=$(date +"%T")
echo "Time now : ${CURRENT_TIME}"
sleep ${Wait_INTERVAL};
CURRENT_TIME=$(date +"%T")
echo "Time after delay: ${CURRENT_TIME}"
echo "**************************************************************************"

# Running the tm5-900 demo file
ros2 launch tm5-900_moveit_config tm5-900_run_move_group_gz.launch.py

################################################################################
# If the user has correctly installed the gazebo environment,                  #
# then just execute this demo, and will kill other suspicious Gazebo "zombie"~ #
#                $ cd your_workspace/tm_gazebo/scripts                         #
#                $ bash run_gz_motionplanning.sh                               #
################################################################################
