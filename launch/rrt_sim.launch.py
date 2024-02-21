from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner_cpp',
            executable='node_publisher',
            name='node_publisher'
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'
        ),
        ExecuteProcess(cmd=['foxglove-studio']),
    ])
