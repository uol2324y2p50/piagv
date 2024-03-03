from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag0_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '-1', 'world', 'tag0_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag1_publisher',
            arguments=['0.096', '0', '0', '0', '0', '0', '-1', 'world', 'tag1_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag2_publisher',
            arguments=['0.192', '0', '0', '0', '0', '0', '-1', 'world', 'tag2_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag3_publisher',
            arguments=['0', '-0.0576', '0', '0', '0', '0', '-1', 'world', 'tag3_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag4_publisher',
            arguments=['0.096', '-0.0576', '0', '0', '0', '0', '-1', 'world', 'tag4_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag5_publisher',
            arguments=['0.192', '-0.0576', '0', '0', '0', '0', '-1', 'world', 'tag5_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag6_publisher',
            arguments=['0', '-0.1152', '0', '0', '0', '0', '-1', 'world', 'tag6_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag7_publisher',
            arguments=['0.096', '-0.1152', '0', '0', '0', '0', '-1', 'world', 'tag7_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_world_tag8_publisher',
            arguments=['0.192', '-0.1152', '0', '0', '0', '0', '-1', 'world', 'tag8_fixed']
        ),
        Node(
            package='tf2_ros',
            namespace='map_server',
            executable='static_transform_publisher',
            name='static_baselink_camera_publisher',
#            arguments=['0', '0', '0.2', '-0.579', '0.579', '-0.406', '0.406', 'base_link', 'camera']
            arguments=['0', '0', '0.265', '-0.590', '0.554', '-0.403', ' 0.428', 'base_link', 'camera']
        ),
    ])

