from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='usb_cam',
             executable='usb_cam_node_exe',
             output='screen'),
        Node(package='arm_cv',
             executable='yolov8_obb_publisher.py',
             output='screen'),
    ])
