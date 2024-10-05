from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='find_wall_service_node',
            output='screen',
            #parameters=[],
            #arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])