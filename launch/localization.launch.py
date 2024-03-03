from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piagv',
            namespace='localization',
            executable='world_to_odom_publisher',
            name='world_to_odom_publisher',
        ),
    ])

