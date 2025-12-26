############################################################################################### 
#  tm20_gazebo.launch.py
#  Launch TM Cobot in Gazebo Fortress simulation
############################################################################################### 

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    print('#####################################################################################################')
    print('This launch file use the robot_model_file://$(find tm_gazebo) vs meshes file://$(find tm_description)')
    print('#####################################################################################################')

    # Robot Description and Controller Configuration
    robot_model_file = 'tm20.urdf.xacro'
    robot_model_path = 'xacro'
    robot_controller_file = 'tm_controllers.yaml'

    # Specify the paths/directories to TM/ROS package definition
    project_description_pkg = 'tm_description'
    description_dir = get_package_share_directory(project_description_pkg)
    project_gazebo_pkg = 'tm_gazebo'
    gazebo_dir = get_package_share_directory(project_gazebo_pkg)
    project_ros_gz_sim_pkg = 'ros_gz_sim'
    ros_gz_sim_dir = get_package_share_directory(project_ros_gz_sim_pkg)
    gazebo_models_path = gazebo_dir + '/models'
    gazebo_worlds_path = gazebo_dir + '/worlds'
    world_path = os.path.join(gazebo_dir, 'worlds', 'test_world.world')
    models_path = os.path.join(gazebo_dir, 'worlds')
    world_file = "empty.sdf"

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(project_gazebo_pkg), "config", robot_controller_file]
    )

    rviz_path_file = '/rviz/view_model.rviz'
    rviz_config_file = gazebo_dir + rviz_path_file

    # Set Gazebo environment variables/ resource path
    set_env_vars_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gazebo_models_path)

    # -------------------------------------------------------------------------
    # Load the robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare(project_gazebo_pkg),
                    robot_model_path,
                    robot_model_file,
                ]
            ),
            ' ',
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    # -------------------------------------------------------------------------

    # Visualize in RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        emulate_tty=True,
        arguments=['-d', rviz_config_file],
    )

    # Publish TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            robot_description
        ],
    )

    # joint_state_broadcaster: controller_manager
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            ],
    )

    # Declare the handler that rviz start when joint_state_broadcaster_spawner exits
    joint_state_broadcaster_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    # arm_controller: controller_manager
    tm_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'tmr_arm_controller',
            '--controller-manager',
            '/controller_manager',
            ],
    )

    # Spawn GZ nodes
    ros_gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_description_content,
            '-allow_renaming',
            'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
            ],
    )

    # Gazebo Setup: to launch the simulator and Gazebo world
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments={'gz_args': [' -r -v 4 ', world_file]}.items(),
    )

    # Gazebo - ROS Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # List of nodes to be launched
    return LaunchDescription(
        [
            set_env_vars_resources,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            joint_state_broadcaster_handler,
            tm_arm_controller_spawner,
            ros_gz_spawn_entity,
            start_gz_sim,
            ros_gz_bridge,
        ]
    )
