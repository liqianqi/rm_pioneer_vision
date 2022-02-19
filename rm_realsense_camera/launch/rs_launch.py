import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_config = os.path.join(
        get_package_share_directory('rm_realsense_camera'),
        'config/d455.yaml'
    )

    realsense2_camera_node = Node(
        package='realsense2_camera',
        namespace='camera',
        executable='realsense2_camera_node',
        parameters=[camera_config],
    )

    return LaunchDescription([realsense2_camera_node])
