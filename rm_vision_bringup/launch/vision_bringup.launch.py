import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

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

    xacro_path = os.path.join(get_package_share_directory(
        'rm_pioneer_description'), 'urdf', 'gimbal.urdf.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro ', str(xacro_path)]), value_type=str)},
            {'publish_frequency': 1000.0}
        ],
    )

    return LaunchDescription([
        mv_camera_launch,
        rm_auto_aim_launch,
        rm_serial_launch,
        robot_state_publisher,
    ])
