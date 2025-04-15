import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    robot_description_path = os.path.join(
        get_package_share_directory('arm_description'))

    arm_robot_sim_path = os.path.join(
        get_package_share_directory('arm_simulation'))

    cv_pkg_path = os.path.join(
        get_package_share_directory('arm_cv'))

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(arm_robot_sim_path, 'worlds'), ':' +
            str(Path(robot_description_path).parent.resolve())
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='arm_world',
                          description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )

    xacro_file = os.path.join(robot_description_path,
                              'model',
                              'arm.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.05',
                   '-y', '0.0',
                   '-z', '1.04',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'arm_robot',
                   '-allow_renaming', 'false'],
    )

    rviz_config_file = os.path.join(
        robot_description_path,
        "rviz",
        "arm_description.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time": True},],
    )

    urdf_path = os.path.join(robot_description_path, 'model', 'arm.urdf.xacro')

    doc = xacro.process_file(urdf_path, mappings={'use_sim': 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[params,
                    {'use_sim_time': True}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    load_controllers = []
    for controller in [
        # "forward_position_controller",
        "arm_controller",
        "ee_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    return LaunchDescription(
        [
            gazebo_resource_path,
            arguments,
            gazebo,
            gz_spawn_entity,
            robot_state_publisher,
            bridge,
            rviz_node,
            static_tf,
        ]
        + load_controllers
    )
