#!/bin/bash

#########################################################
# This script test the tm5-900 cobot on the Gazebos GUI #
#########################################################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source the main ROS setup file
cd ../../..
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# terminal1: To kill the GUI Gazebo executables
if timeout 1s gnome-terminal -- bash -c "cd $SCRIPT_DIR && ./kill_gz.sh; exec bash" ; then
    echo "kill all Gazebo process... successfully"
else
    echo "fetching timed out, shut down"
    exit 1
fi

sleep 2s

# Display gazebo version
echo ign gazebo= $`ign gazebo --versions`

# terminal2: Run Gazebo GUI Demo
echo ""
if timeout 1s gnome-terminal -- bash -c "cd $SCRIPT_DIR && ros2 launch tm_gazebo tm5-900_gazebo.launch.py; exec bash" ; then
    echo "Demo TM5-900 Robot Model on Gazebo GUI successfully"
else
    echo "Fetching timed out, shut down"
    exit 1
fi

# terminal3: Run send_goal.sh action to move serval positions.
sleep 15s
echo ""
if timeout 1s gnome-terminal --geometry=80x16-0-0 -- bash -c "cd $SCRIPT_DIR && ./send_goal.sh; exec bash" ; then
    echo "ROS2 action send_goal to move"
else
    echo "Fetching timed out, shut down"
    exit 1
fi
###########################################################
# If you are interested in the Gazebo process pid,        #
# for example, to run TM Gazebo first ~                   #
#       terminal1$ cd your_workspace/tm_gazebo/scripts    #
#       terminal1$ bash demo_gz_goalaction.sh             #
# then, to kill gazeb PIDs with the kill command.         #                                                           
#       terminal2$ bash kill_gz_pid.sh                    #
###########################################################
