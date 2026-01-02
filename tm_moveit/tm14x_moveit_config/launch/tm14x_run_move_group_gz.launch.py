############################################################################################### 
#  tm14x_run_move_group_gz.launch.py
#   
#  Various portions of the code are based on original source from 
#  The reference: "https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo/launch"
#  and are used in accordance with the following license.
#  "https://github.com/moveit/moveit2/blob/main/LICENSE.txt"
############################################################################################### 

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
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

import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    args = []
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    # Launch configuration variables
    run_simulator = LaunchConfiguration('sim')
    run_rviz = LaunchConfiguration('use_rviz')

    load_run_simulator_argument = DeclareLaunchArgument(
        name='sim',
        default_value='true',
        description='Use the virtual TM Robot Simulation')

    load_run_rviz_argument = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='To start RVIZ')

    print('#####################################################################################################')
    print('This launch file use the robot_model_file://$(find tm_gazebo) vs meshes file://$(find tm_description)')
    print('#####################################################################################################')

    # Robot Description and Controller Configuration
    robot_model_file = 'tm14x.urdf.xacro'
    robot_model_path = 'xacro'
    robot_controller_file = 'tm_controllers.yaml'

    # Specify the paths/directories to TM/ROS package definition
    # project_description_pkg = 'tm_description'
    # description_dir = get_package_share_directory(project_description_pkg)
    moveit_config_pkg = 'tm14x_moveit_config'
    moveit_dir = get_package_share_directory(moveit_config_pkg)
    project_gazebo_pkg = 'tm_gazebo'
    gazebo_dir = get_package_share_directory(project_gazebo_pkg)
    project_ros_gz_sim_pkg = 'ros_gz_sim'
    ros_gz_sim_dir = get_package_share_directory(project_ros_gz_sim_pkg)
    gazebo_models_path = gazebo_dir + '/models'
    gazebo_worlds_path = gazebo_dir + '/worlds'
    world_file = 'empty.sdf'
    # bridge_path_yaml = os.path.join(gazebo_dir, 'config', 'tm_gz_bridge.yaml')

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(project_gazebo_pkg), "config", robot_controller_file]
    )

    srdf_path_file = 'config/tm14x_gz.srdf'
    rviz_path_file = '/rviz/view_robot.rviz'
    rviz_config_file = gazebo_dir + rviz_path_file

    # Set Gazebo environment variables/ resource path
    # os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_models_path + ":" + gazebo_worlds_path
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

    # SRDF Configuration
    robot_description_semantic_config = load_file(moveit_config_pkg, srdf_path_file)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    # Controllers
    controllers_yaml = load_yaml(moveit_config_pkg, 'config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml, 'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    # Trajectory Execution Functionality
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1,
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Joint limits
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            moveit_config_pkg, 'config/joint_limits.yaml'
        )
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {'use_sim_time': True},
        ],
    )

    # Visualize in RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
            {'use_sim_time': True},
        ],
    )

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
        condition=IfCondition(run_rviz),
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
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        # parameters=[{
        #     'config_file': bridge_path_yaml,
        # }],
    )

    # joint driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        # name='tm_driver',
        output='screen',
        arguments=args,
        condition=UnlessCondition(run_simulator),
    )

    # List of nodes to be launched
    return LaunchDescription(
        [
            set_env_vars_resources,
            load_run_simulator_argument,
            load_run_rviz_argument,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            joint_state_broadcaster_handler,
            tm_arm_controller_spawner,
            ros_gz_spawn_entity,
            start_gz_sim,
            ros_gz_bridge,
            tm_driver_node,
            run_move_group_node,
        ]
    )
