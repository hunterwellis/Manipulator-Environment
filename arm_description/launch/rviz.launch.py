import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # package path
    desc_pkg_path = get_package_share_directory('arm_description')

    # configuration path
    rviz_config = os.path.join(
        desc_pkg_path,
        'rviz',
        'arm_description.rviz'
    )

    # config launch argument
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Path to the config file'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        rviz_arg,
        rviz
    ])
