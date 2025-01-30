import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # package path
    desc_pkg_path = get_package_share_directory('arm_description')

    # load arm description files
    urdf_path = os.path.join(desc_pkg_path, 'model', 'arm.urdf.xacro')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # option to spawn into gazebo environment
    spawn_flag = LaunchConfiguration('spawn')
    gz_spawn = DeclareLaunchArgument(
        name='spawn',
        default_value='false',
        choices=['true', 'false'],
        description='gazebo spawn flag')

    # Gazebo spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc,
                   '-x', '0.05',
                   '-y', '0.0',
                   '-z', '0.02',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'arm',
                   '-allow_renaming', 'false'],
        output='screen',
        condition=IfCondition(spawn_flag)
    )

    # state pub
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        gz_spawn,
        spawn_robot,
        robot_state_publisher
    ])
