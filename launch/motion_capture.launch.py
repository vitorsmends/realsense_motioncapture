from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    rs_launch_path = os.path.join(
        FindPackageShare('realsense2_camera').find('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={
                'enable_rgbd': 'true',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'enable_color': 'true',
                'enable_depth': 'true',
                'rgb_camera.profile': '640x480x30',
                'depth_module.profile': '640x480x30'
            }.items()
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
