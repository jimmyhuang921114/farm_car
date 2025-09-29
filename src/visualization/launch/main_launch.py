import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        # 启动 visualization
        ExecuteProcess(
            cmd=['ros2', 'run', 'visualization', 'visual'],
            output='screen'
        ),

        # 启动 UWB receiver
        ExecuteProcess(
            cmd=['ros2', 'run', 'uwb', 'receiver', '0', '2', '3', '5'],
            output='screen'
        ),

        # 启动 UWB filter
        ExecuteProcess(
            cmd=['ros2', 'run', 'uwb', 'filter', '0', '2', '3', '5'],
            output='screen'
        ),

        # 启动 UWB localization
        ExecuteProcess(
            cmd=['ros2', 'run', 'uwb', 'localization', '0', '2', '3', '5'],
            output='screen'
        ),

        # 启动 UWB last_filter
        ExecuteProcess(
            cmd=['ros2', 'run', 'uwb', 'last_filter', '0', '2', '3', '5'],
            output='screen'
        ),

        # 启动 超音波 modbus
        ExecuteProcess(
            cmd=['ros2', 'run', 'us_pkg8', 'us_modbus_receiver_node'],
            output='screen'
        ),

        # 启动 超音波 uart
        ExecuteProcess(
            cmd=['ros2', 'run', 'us_pkg8', 'us_uart_receiver_node'],
            output='screen'
        ),

        # 启动 Realsense 相机
        ExecuteProcess(
            cmd=['ros2', 'run', 'camera', 'realsense'],
            output='screen'
        ),

        # 启动 Bytetrack
        ExecuteProcess(
            cmd=['ros2', 'run', 'track', 'bytetrack', '--realsense'],
            output='screen'
        ),

        # 启动障碍物检测
        ExecuteProcess(
            cmd=['ros2', 'run', 'obstacle_detect_last', 'obs_detect'],
            output='screen'
        ),
    ])
