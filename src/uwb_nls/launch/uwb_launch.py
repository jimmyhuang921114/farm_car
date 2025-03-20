from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwb_nls',
            executable='receiver',
            name='receiver',
            output='screen',
            arguments=['0', '2', '3', '5']
        ),
        Node(
            package='uwb_nls',
            executable='first_filter',
            name='first_filter',
            output='screen',
            arguments=['0', '2', '3', '5']
        ),
        Node(
            package='uwb_nls',
            executable='position_nls',
            name='position_nls',
            output='screen',
            arguments=['0', '2', '3', '5']
        ),
        Node(
            package='uwb_nls',
            executable='last_filter_nls',
            name='last_filter_nls',
            output='screen'
        ),
    ])
