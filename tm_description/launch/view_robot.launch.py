############################################################################################### 
# view_robot.launch.py
# Description: Launch TM Cobot in RVIZ
############################################################################################### 

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

# Prerequisites: joint-state-publisher-gui
# Terminal: [key-in] shell cmd $ sudo apt install ros-humble-joint-state-publisher
#                              $ sudo apt install ros-humble-joint-state-publisher-gui
# Usage: Spawn a Techman robot model in the Rviz2.
# Example: Take TM5-900 Techman robot model as the default, so set 'tm5-900.urdf.xacro' in robot_description_config
# Terminal: [key-in] shell cmd $ ros2 launch tm_description view_robot.launch.py


def generate_launch_description():

    print('##############################################################')
    print('This launch file use the TM5-900 robot_model_file for example.')
    print('##############################################################')

    # Robot Description Configuration
    robot_model_file = 'tm5-900.urdf.xacro'
    robot_model_path = 'xacro'

    # Specify the paths/directories to TM/ROS package definition
    project_description_pkg = 'tm_description'
    description_dir = get_package_share_directory(project_description_pkg)
    robot_description_file = os.path.join(description_dir, robot_model_path, robot_model_file)
    rviz_path_file = '/rviz/view_robot.rviz'
    rviz_config_file = description_dir + rviz_path_file

    # -------------------------------------------------------------------------
    # Load the robot_description
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    # -------------------------------------------------------------------------	

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description
        ]
    )

    # Publish joint states
    joint_state_slider = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output=['screen']
    )

    # Visualize in RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description
        ]
    )

    # List of nodes to be launched
    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher_node,
            joint_state_slider,
            rviz_node,
        ]
    )
