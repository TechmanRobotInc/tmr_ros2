#!/bin/bash

##########################################################
# This script should be to kill the GUI Gazebo executables.
##########################################################

# Find all gazebo processes and extract their PIDs
ps aux | grep gazebo
echo ""
Delay_INTERVAL="1";
echo "Wait... for ${Delay_INTERVAL} secondes"
echo ""
echo "****************************************************"
echo "To find all gazebo processes and extract their PIDs."
echo "****************************************************"
CURRENT_TIME=$(date +"%T")
echo "Time now : ${CURRENT_TIME}"
sleep ${Delay_INTERVAL};
CURRENT_TIME=$(date +"%T")
echo "Time after delay: ${CURRENT_TIME}"
echo "****************************************************"

##################################################
# To kill the GUI Gazebo each executables by PIDS.
##################################################
pids=$(ps aux | grep gazebo | grep -v grep | awk '{print $2}')

# Check if any gazebo processes were found
if [ -z "$pids" ]; then
  echo "No gazebo processes found."
else
  # Force to kill all gazebo process
  for pid in $pids; do
    kill $pids
    echo "Successfully kill gazebo process with PID $pid"
  done
  echo "All gazebo processes have been killed."
fi
