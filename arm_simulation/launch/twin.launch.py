import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # package paths
    desc_pkg_path = get_package_share_directory('arm_description')
    sim_pkg_path = get_package_share_directory('arm_simulation')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    # world path
    world_file = os.path.join(
        sim_pkg_path,
        'worlds',
        'arm_word.sdf'
    )

    # bridge path
    bridge_file = os.path.join(
        sim_pkg_path,
        'config',
        'bridge.yaml'
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

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'arm.launch.py'
                         )
        ),
        launch_arguments={'spawn': 'true'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'rviz.launch.py'
                         )
        )
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        gazebo_resource_path,
        gz_sim,
        jsp,
        state_publisher,
        bridge,
        rviz
    ])
