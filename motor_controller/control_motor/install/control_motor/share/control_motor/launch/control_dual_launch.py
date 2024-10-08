from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_motor',
            namespace='',
            executable='control_dual',
            name='Control_motor'
        ),
        Node(
            package='control_motor',
            namespace='',
            executable='Twist2Speed',
            name='Twist2Speed'
        )
    ])
