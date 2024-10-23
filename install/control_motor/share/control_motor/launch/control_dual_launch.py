from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_motor',
            executable='control_dual',
            name='control_motor'
        ),
        Node(
            package='control_motor',
            executable='Twist2Speed',
            name='Twist2Speed'
        ),
        Node(
            package='control_motor',
            executable='rc_receiver',
            name='rc_receiver'
        ),
        Node(
            package='control_motor',
            executable='msg_choose',
            name='msg_choose'
        )
    ])
