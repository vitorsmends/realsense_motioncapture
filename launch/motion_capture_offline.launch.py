from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
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
            output='screen'
        ),
    ])
