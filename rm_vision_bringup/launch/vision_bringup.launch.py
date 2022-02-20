import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_robot_type = DeclareLaunchArgument(
        name='robot', default_value='standard')

    mv_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mindvision_camera'), 'launch', 'mv_launch.py')))

    rm_auto_aim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_auto_aim'), 'launch', 'rm_auto_aim.launch.py')))

    rm_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_serial_driver'), 'launch', 'serial_driver.launch.py')))

    urdf_dir = os.path.join(get_package_share_directory(
        'rm_pioneer_description'), 'urdf/')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(
                ['xacro ', urdf_dir,
                 LaunchConfiguration('robot'), '.urdf.xacro'])}
        ]
    )

    return LaunchDescription([
        declare_robot_type,
        mv_camera_launch,
        rm_auto_aim_launch,
        rm_serial_launch,
        robot_state_publisher,
    ])
