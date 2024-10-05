from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
        # Launch the service node
    find_wall_service_node = Node(
        package="wall_follower",
        executable="find_wall_service_node",
        output="screen",
    )

    # Launch the wall_fallower node
    wall_follower_node = Node(
        package="wall_follower",
        executable="wall_follower_node",
        output="screen",
    )

    return LaunchDescription([
        # Launch the server node immediately 
        find_wall_service_node,
        
        # Launch the wall_follower node 
        wall_follower_node,
    ])