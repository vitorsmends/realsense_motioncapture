from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_motioncapture',
            executable='image_filter',
            name='image_filter',
            output='screen'
        ),
        Node(
            package='realsense_motioncapture',
            executable='motion_capture',
            name='motion_capture',
            output='screen'
        ),
        Node(
            package='realsense_motioncapture',
            executable='depth_image_filter',
            name='depth_image_filter',
            output='screen'
        ),
    ])
