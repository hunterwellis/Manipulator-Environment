import os
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
    sdf_file = os.path.join(desc_pkg_path, 'model', 'arm.urdf.xacro')
    with open(sdf_file, 'r') as infp:
        arm_desc_urdf = infp.read()

    sdf_file = os.path.join(desc_pkg_path, 'model', 'arm.sdf')
    with open(sdf_file, 'r') as infp:
        arm_desc_sdf = infp.read()

    # option to spawn into gazebo environment
    spawn_flag = LaunchConfiguration('jsp_gui')
    gz_spawn = DeclareLaunchArgument(
        name='spawn',
        default_value='true',
        choices=['true', 'false'],
        description='gazebo spawn flag')

    # Gazebo spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string',
            arm_desc_sdf,
            '-name',
            'hermes_robot_description',
            '-x',
            '0',
            '-y',
            '0',
            '-z',
            '.4',
        ],
        output='screen',
        condition=IfCondition(spawn_flag)
    )

    # state pub
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': arm_desc_urdf},
        ]
    )

    return LaunchDescription([
        gz_spawn,
        spawn_robot,
        robot_state_publisher
    ])
