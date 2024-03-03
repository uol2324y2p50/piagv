from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Define a ComposableNode for the v4l2_camera
    v4l2_camera_node = ComposableNode(
        package='v4l2_camera',
        plugin='v4l2_camera::V4L2Camera',
        name='camera',
        namespace='apriltag',
        parameters=[{'video_device': '/dev/video0'}],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Define a ComposableNode for the image_rectify
    image_rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='image_rectify',
        namespace='apriltag',
        remappings=[
            ('image', '/apriltag/image_raw'),
            ('camera_info', '/apriltag/camera_info'),
            ('image_rect', '/apriltag/image_rect'),
        ],
        parameters=[PathJoinSubstitution([FindPackageShare('piagv'), 'config', 'image_rectify.yaml'])],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Define a ComposableNode for the apriltag_ros
    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag_detect',
        namespace='apriltag',
        remappings=[
            ('image_rect', '/apriltag/image_rect'),
            ('camera_info', '/apriltag/camera_info')
        ],
        parameters=[PathJoinSubstitution([FindPackageShare('piagv'), 'config', 'apriltag_detect.yaml'])],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Define the container to hold both nodes
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='apriltag_container',
        namespace='',
        composable_node_descriptions=[
            v4l2_camera_node,
            image_rectify_node,
            apriltag_node
        ],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the container to the launch description
    ld.add_action(apriltag_container)

    return ld
