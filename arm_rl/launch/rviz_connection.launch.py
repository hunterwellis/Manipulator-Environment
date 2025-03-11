import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # package path
    desc_pkg_path = os.path.join(
        get_package_share_directory('arm_description'))
    rl_pkg_path = os.path.join(
        get_package_share_directory('arm_rl'))

    urdf_file_path = os.path.join(
        desc_pkg_path,
        'model',
        'arm.urdf.xacro'
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_file_path],
        parameters=[{'use_gui': True, 'sources_list': ['/rl_jsp']}]
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'arm.launch.py'
                         )
        ),
        launch_arguments={'spawn': 'false'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_path,
                         'launch',
                         'rviz.launch.py'
                         )
        )
    )

    arm_sig = Node(
        package = 'arm_description',
        executable = 'arm_ctrl.py')

    return LaunchDescription([
        jsp,
        state_publisher,
        rviz,
        arm_sig
    ])
