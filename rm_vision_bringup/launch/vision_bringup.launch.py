import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # robot type
    robot_type_launch_arg = DeclareLaunchArgument(
        name='robot', default_value='guard')
    robot = LaunchConfiguration('robot')

    # params file path
    config_dir = os.path.join(get_package_share_directory(
        'rm_vision_bringup'), 'config')
    params_file = [config_dir, '/', robot, '_params.yaml']
    camera_info_url = ['package://rm_vision_bringup/config/',
                       robot, '_camera_info.yaml']

    # xacro file path
    urdf_dir = os.path.join(get_package_share_directory(
        'rm_pioneer_description'), 'urdf')
    xacro_file = Command(
        ['xacro ', urdf_dir, '/', robot, '.urdf.xacro'])

    mv_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mindvision_camera'), 'launch', 'mv_launch.py')),
        launch_arguments={'params_file': params_file,
                          'camera_info_url': camera_info_url,
                          'use_sensor_data_qos': 'true'}.items())

    rm_auto_aim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_auto_aim'), 'launch', 'auto_aim.launch.py')),
        launch_arguments={'detector_params_file': params_file,
                          'debug': 'true'}.items())

    rm_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_serial_driver'), 'launch', 'serial_driver.launch.py')))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': xacro_file}]
    )

    return LaunchDescription([
        robot_type_launch_arg,
        mv_camera_launch,
        rm_auto_aim_launch,
        rm_serial_launch,
        robot_state_publisher,
    ])
