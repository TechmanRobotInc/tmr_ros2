import sys
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    args = sys.argv[4:]

    return LaunchDescription([
        Node(
            package='tm_driver',
            executable='tm_driver',
            output='screen',
            arguments=args,
        )
    ])


if __name__ == "__main__":
    from launch import LaunchService
    ls = LaunchService(debug=True)
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
