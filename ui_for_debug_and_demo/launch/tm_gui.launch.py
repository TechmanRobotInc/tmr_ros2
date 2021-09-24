import sys
from launch import LaunchDescription
from launch_ros.actions import Node

#launch_arguments: expected format '<name>:=<value>', you can type robot_ip:=<ip value> or ip:=<ip value>  .
#example: ros2 launch ui_for_debug_and_demo tm_gui.launch.py robot_ip:=192.168.10.2

def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    # gui_demo executable
    ui_for_debug_and_demo_node = Node(
        package='ui_for_debug_and_demo',
        executable='robot_ui',
        output='screen',
        #The five different verbosity levels are, in order: DEBUG, INFO, WARN, ERROR, FATAL      
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    # tm driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        output='screen',
        #The five different verbosity levels are, in order: DEBUG, INFO, WARN, ERROR, FATAL
        arguments=args,
        #arguments=[args, '--ros-args', '--log-level', 'DEBUG'],
    )
	
    return LaunchDescription([ui_for_debug_and_demo_node,tm_driver_node])
