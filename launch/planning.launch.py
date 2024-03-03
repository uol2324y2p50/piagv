from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piagv',
            namespace='planning',
            executable='simple_planner',
            name='simple_planner',
        ),
        Node(
            package='piagv',
            namespace='planning',
            executable='speed_limiter',
            name='speed_limiter',
            remappings=[
                ('cmd_vel_in', '/planning/cmd_vel'),
                ('cmd_vel_out', '/vehicle/cmd_vel'),
            ]
        ),
    ])

