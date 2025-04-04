from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
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
        load_joint_state_controller,
        load_forward_position_controller
    ])
