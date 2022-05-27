import os
import sys
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = get_package_share_directory('tm_moveit_cpp_demo') + "/config/moveit_cpp.yaml"

    # Component yaml files are grouped in separate namespaces
    # Use URDF file: tm5x-700-nominal.urdf to do moveit demo
    # robot_description_config = load_file('tm_description', 'urdf/tm5x-700-nominal.urdf')
    # robot_description = {'robot_description' : robot_description_config}
    # Use Xacro file: tm5x-700.urdf.xacro to do moveit demo
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("tm_description"),
            "xacro",
            "tm5x-700.urdf.xacro",
        )
    )                                                    
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file('tm_moveit_config_tm5x-700', 'config/tm5x-700.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('tm_moveit_config_tm5x-700', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    controllers_yaml = load_yaml('tm_moveit_cpp_demo', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                           'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('tm_moveit_config_tm5x-700', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # MoveItCpp demo executable
    run_moveit_cpp_node = Node(
        package='tm_moveit_cpp_demo',
        # TODO(henningkayser): add debug argument
        # prefix='xterm -e gdb --args',
        executable='run_moveit_cpp',
        #name='run_moveit_cpp',
        output='screen',
        parameters=[
            moveit_cpp_yaml_file_name,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers]
        )

    # RViz
    rviz_config_file = get_package_share_directory('tm_moveit_cpp_demo') + "/launch/run_moveit_cpp.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic]
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
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # joint driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        #name='tm_driver',
        output='screen',
        arguments=args
    )
    '''
    fake_joint_driver_node = Node(
        package='fake_joint_driver',
        executable='fake_joint_driver_node',
        # TODO(JafarAbdi): Why this launch the two nodes (controller manager and the fake joint driver) with the same name!
        # name='fake_joint_driver_node',
        parameters=[
            {'controller_name': 'tmr_arm_controller'},
            os.path.join(get_package_share_directory("tm_moveit_cpp_demo"), "config", "tmr_controllers.yaml"),
            os.path.join(get_package_share_directory("tm_moveit_cpp_demo"), "config", "tmr_start_positions.yaml"),
            robot_description]
        )
    '''

    return LaunchDescription([ tm_driver_node, static_tf, robot_state_publisher, rviz_node, run_moveit_cpp_node ])
