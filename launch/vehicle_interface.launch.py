from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piagv',
            namespace='vehicle',
            executable='vehicle_interface',
            name='vehicle_interface',
        ),
    ])

