import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_robot_type = DeclareLaunchArgument(
        name='robot', default_value='standard')

    rviz_config_path = os.path.join(get_package_share_directory(
        'rm_pioneer_description'), 'launch', 'view_model.rviz')

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

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        declare_robot_type,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
