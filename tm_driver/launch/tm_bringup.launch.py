import sys
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    return LaunchDescription([
        Node(
            package='tm_driver',
            executable='tm_driver',
            output='screen',
            arguments=args,
        )
    ])