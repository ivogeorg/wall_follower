from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='wall_follower',
            executable='laser_scan_subscriber_node',
            output='screen'
        ),
    ])