#import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

'''
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None
'''

def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    '''
    robot_description_config = load_file('tmr_description', 'urdf/tm5-900.urdf')
    robot_description = {'robot_description' : robot_description_config}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #name='robot_state_publisher',
        #output='both',
        #parameters=[robot_description]
    )
    '''

    tmr_driver_node = Node(
        package='tmr_driver',
        executable='tmr_driver',
        #name='tmr_driver',
        output='screen',
        arguments=args
    )

    return LaunchDescription([
        tmr_driver_node
    ])