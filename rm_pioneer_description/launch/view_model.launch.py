import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory(
        'pioneer_description'), 'urdf', 'gimbal.urdf.xacro')

    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro ', str(xacro_path)]), value_type=str)},
            # {'publish_frequency': 1000.0}
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    return LaunchDescription([model_node])
