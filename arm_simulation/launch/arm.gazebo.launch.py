#!/usr/bin/env python3

import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Launch arguments
ARGUMENTS = {
    DeclareLaunchArgument('robot_name', default_value='arm',
                          description='name of robot'),
    DeclareLaunchArgument('prefix', default_value='',
                          description='prefix for robot joints and links'),
    DeclareLaunchArgument('add_world', default_value='true',
                          choices=['true', 'false'],
                          description='add world link flag'),
    DeclareLaunchArgument('world', default_value='arm_world',
                          description='Gazebo sim world'),
    DeclareLaunchArgument('base_link', default_value='base_link',
                          description='name of the base link'),
}


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)

    # Package Paths
    desc_pkg_path = os.path.join(
        get_package_share_directory('arm_description'))
    sim_pkg_path = os.path.join(
        get_package_share_directory('arm_simulation'))
    ros_gz_pkg = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    # ctrl_pkg_path = os.path.join(
    #     get_package_share_directory('arm_control'))

    # Decription files
    urdf_file_path = os.path.join(
        desc_pkg_path,
        'model',
        'arm.urdf.xacro'
    )
    rviz_file_path = os.path.join(
        desc_pkg_path,
        'rviz',
        'arm_description.rviz'
    )

    # Simulation files
    gz_bridge_path = os.path.join(
        sim_pkg_path,
        'config',
        'bridge.yaml'
    )

    world_config_path = os.path.join(
        sim_pkg_path,
        'worlds',
        'arm_world.sdf'
    )

    # Set gazebo sim resources
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(sim_pkg_path, 'worlds'), ':' +
            str(Path(desc_pkg_path).parent.resolve())
            ]
        )

    # @TODO: gazebo environment launch files
    # gz_file_path = os.path.join(
    #     sim_pkg_path,
    #     'launch',
    #     ''
    # )

    # Launch config settings
    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')

    # Set the pose configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # set launch config settings
    jsp_gui_arg = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='joint state publisher gui flag')

    rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_file_path,
        description='RViz config path')

    urdf_model_path_arg = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_file_path,
        description='URDF model path')

    rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'robot_name:=', LaunchConfiguration('robot_name'), ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
        'add_world:=', LaunchConfiguration('add_world'), ' ',
        'base_link:=', LaunchConfiguration('base_link'), ' ',
    ]), value_type=str)
    
    # pose arguments
    x_arg= DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x component of initial position, meters')

    y_arg = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='y component of initial position, meters')

    z_arg = DeclareLaunchArgument(
        name='z',
        default_value='0.05',
        description='z component of initial position, meters')

    roll_arg = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    pitch_arg = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    yaw_arg = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')

    # Gazebo simulation setup
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -v 4 ', world_config_path])])

    # Nodes
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content}])

    # Launch gui depending on jsp_gui condition
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(jsp_gui))

    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_file_path],
        condition=IfCondition(jsp_gui))

    # RViz node
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],)

    # Gazebo node
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'arm',
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ]
    )

    # Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_path,
        }],
        output='screen'
    )

    # add launch options/arguments
    ld.add_action(jsp_gui_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(urdf_model_path_arg)
    ld.add_action(rviz_arg)

    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(roll_arg)
    ld.add_action(pitch_arg)
    ld.add_action(yaw_arg)

    # RViz and JSP
    ld.add_action(joint_state_pub)
    ld.add_action(joint_state_pub_gui)
    ld.add_action(robot_state_pub)
    ld.add_action(rviz)

    # Gazebo
    ld.add_action(gazebo_resource_path)
    ld.add_action(gazebo_launch)
    ld.add_action(gz_spawn)
    ld.add_action(bridge)

    return ld
