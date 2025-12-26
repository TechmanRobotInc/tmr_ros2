#!/bin/bash

################################################################################################################################
# This script (send_goal.sh) demo send_goal action to multi-positions.
#
# Example: First, you can launch the gazebo simulator and spawn the robot model into gazebo and rviz.
# Syntax : $ ros2 launch tm_gazebo <tm_robot_type>_gazebo.launch.py
# If the prefix `<tm_robot_type>` is tm12s, [key-in] shell cmd $ ros2 launch tm_gazebo tm12s_gazebo.launch.py
# Then, enter the scripts directory path in another new terminal and execute this bash. [key-in] shell cmd $ $ bash send_goal.sh
#################################################################################################################################

echo "moving the cobot to the simple goal positions..."
ros2 action send_goal /tmr_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.00], time_from_start: { sec: 0, nanosec: 500 } },
      { positions: [0.8, 0.0, 0.8, 0.0, 0.0, -0.83], time_from_start: { sec: 5, nanosec: 500 } },
      { positions: [1.2, 0.0, 1.2, 0.0, 0.0, -1.26], time_from_start: { sec: 7, nanosec: 500 } },
      { positions: [1.6, 0.0, 1.6, 0.0, 0.0, -1.57], time_from_start: { sec: 8, nanosec: 500 } }
    ]
  }
}"

echo ""
Wait_INTERVAL="2";
echo "Wait... for ${Wait_INTERVAL} secondes"
echo ""

echo "moving the cobot return the simple home position..."
ros2 action send_goal /tmr_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      { positions: [1.6, 0.0, 1.6, 0.0, 0.0, -1.57], time_from_start: { sec: 0, nanosec: 500 } },
      { positions: [1.2, 0.0, 1.2, 0.0, 0.0, -1.26], time_from_start: { sec: 2, nanosec: 500 } },
      { positions: [0.8, 0.0, 0.8, 0.0, 0.0, -0.83], time_from_start: { sec: 5, nanosec: 500 } },
      { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.00], time_from_start: { sec: 8, nanosec: 500 } },
    ]
  }
}"
