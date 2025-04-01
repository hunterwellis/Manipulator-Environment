import os
import xacro
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # package paths
    desc_pkg_path = get_package_share_directory('arm_description')
    sim_pkg_path = get_package_share_directory('arm_simulation')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(desc_pkg_path, 'model', 'arm.urdf.xacro')

    # world path
    world_file = os.path.join(
        sim_pkg_path,
        'worlds',
        'arm_word.sdf'
    )

    # world path
    world_file = os.path.join(
        sim_pkg_path,
        'worlds',
        'arm_solo.sdf'
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(sim_pkg_path, 'worlds'), ':' +
            str(Path(desc_pkg_path).parent.resolve())
            ]
        )

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg,
                         'launch',
                         'gz_sim.launch.py'
                         )
        ),
        launch_arguments=[('gz_args', [world_file, ' -v 4', ' -r'])]
    )

    jsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'jsp.launch.py'
                         )
        ),
        launch_arguments={'jsp_gui': 'true'}.items()
    )

    # state_publisher = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(desc_pkg_path,
    #                      'launch',
    #                      'arm.launch.py'
    #                      )
    #     ),
    #     launch_arguments={'spawn': 'true'}.items()
    # )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'rviz.launch.py'
                         )
        )
    )

    # bridge = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(sim_pkg_path,
    #                      'launch',
    #                      'bridge.launch.py'
    #                      )
    #     )
    # )

    # controller = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(sim_pkg_path,
    #                      'launch',
    #                      'controller.launch.py')
    #     )
    # )

    doc = xacro.process_file(urdf_path, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    # Gazebo spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
                   '-string', robot_desc,
                   # '-topic', '/robot_description'
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'arm',
                   '-allow_renaming', 'false'],
        output='screen',
    )

    # state pub
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[params]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_forward_position_controller],
            )
        ),
        gazebo_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        rviz,
        jsp
    ])
