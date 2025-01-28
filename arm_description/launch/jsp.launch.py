import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # package path
    desc_pkg_path = get_package_share_directory('arm_description')

    # load urdf file
    urdf_file_path = os.path.join(
        desc_pkg_path,
        'model',
        'arm.urdf.xacro'
    )

    gui_flag = LaunchConfiguration('jsp_gui')
    jsp_gui = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='joint state publisher GUI flag')

    joint_state_publisher_gui = Node(
        condition=IfCondition(gui_flag),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_file_path])

    joint_state_publisher = Node(
        condition=UnlessCondition(gui_flag),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_file_path])

    return LaunchDescription([
        jsp_gui,
        joint_state_publisher_gui,
        joint_state_publisher
    ])
