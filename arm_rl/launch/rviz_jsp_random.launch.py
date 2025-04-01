import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    desc_pkg_path = os.path.join(
        get_package_share_directory('arm_description')
    )

    jsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path, 'launch', 'jsp.launch.py')
        ),
        launch_arguments={'jsp_gui': 'false'}.items()
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path, 'launch', 'arm.launch.py')
        ),
        launch_arguments={'spawn': 'false'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path, 'launch', 'rviz.launch.py')
        )
    )

    random_joint_publisher = Node(
        package='arm_rl',
        executable='random_jsp.py',
        name='random_joint_publisher',
        output='screen'
    )

    return LaunchDescription([
        jsp,
        state_publisher,
        rviz,
        random_joint_publisher
    ])
