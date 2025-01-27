#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Launch arguments
ARGUMENTS = {
    DeclareLaunchArgument('robot_name', default_value='arm',
                          description='name of robot'),
    DeclareLaunchArgument('prefix', default_value='',
                          description='prefix for robot joints and links'),
    DeclareLaunchArgument('add_world', default_value='true',
                          choices=['true', 'false'],
                          description='add world link flag'),
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
    ctrl_pkg_path = os.path.join(
        get_package_share_directory('arm_control'))

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
    world_file_path = os.path.join(
        sim_pkg_path,
        'worlds',
        'arm_world.sdf'
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

    rviz_cmd = DeclareLaunchArgument(
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
        condition=IfCondition(jsp_gui))

    # RViz node
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],)

    ld.add_action(jsp_gui_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(urdf_model_path_arg)
    ld.add_action(rviz_cmd)

    ld.add_action(joint_state_pub)
    ld.add_action(joint_state_pub_gui)
    ld.add_action(robot_state_pub)
    ld.add_action(start_rviz_cmd)

    return ld
