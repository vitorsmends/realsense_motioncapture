from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_motioncapture',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='realsense_motioncapture',
            executable='image_filter',
            name='image_filter_camera_1',
            output='screen',
            arguments=['/camera_1/image_raw']
        ),
        Node(
            package='realsense_motioncapture',
            executable='image_filter',
            name='image_filter_camera_color',
            output='screen',
            arguments=['/camera/camera/color/image_raw']
        ),
        Node(
            package='realsense_motioncapture',
            executable='motion_capture',
            name='motion_capture',
            output='screen',
            parameters=[
                {'topic_xz': '/camera_1/image_raw_filtered'},
                {'topic_y': '/camera/camera/color/image_raw_filtered'}
            ]
        ),
        Node(
            package='realsense_motioncapture',
            executable='depth_image_filter',
            name='depth_image_filter',
            output='screen'
        ),
    ])
