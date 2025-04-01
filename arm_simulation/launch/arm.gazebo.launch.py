import os
from launch import LaunchDescription
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # define package paths
    desc_pkg_path = get_package_share_directory('arm_description')
    sim_pkg_path = get_package_share_directory('arm_simulation')
    moveit_pkg_path = get_package_share_directory('arm_moveit')

    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # gazebo directorys and paths
    robot_models_path = os.path.join(
        desc_pkg_path,
        'model',
        'meshes')

    sim_models_path = os.path.join(
        sim_pkg_path,
        'models')

    default_world_file = 'pnp.world'
    sim_default_world_path = os.path.join(
        sim_pkg_path,
        default_world_file)

    default_ros_gz_bridge_path = os.path.join(
        sim_pkg_path,
        'config',
        'ros_gz_bridge.yaml'
    )

    # Launch config
    jsp_gui = LaunchConfiguration('jsp_gui')
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    load_controllers = LaunchConfiguration('load_controllers')
    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='laod_controllers',
        default_value='true',
        description='Flag to enable loading of ROS2 controllers'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Flag to enable RViz')

    use_camera = LaunchConfiguration('use_camera')
    declare_use_camera_cmd = DeclareLaunchArgument(
        name='use_camera',
        default_value='false',
        description='Flag to enable the RGBD camera for Gazebo point cloud simulation')

    use_gazebo = LaunchConfiguration('use_gazebo')
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo')

    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    world_file = LaunchConfiguration('world_file')
    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=sim_default_world_path,
        description='World file name')

    world_path = PathJoinSubstitution([
        sim_pkg_path,
        'worlds',
        world_file
    ])

    # set pose config variables
    x = LaunchConfiguration('x')
    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x component of initial position, meters')

    y = LaunchConfiguration('y')
    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='y component of initial position, meters')

    z = LaunchConfiguration('z')
    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.05',
        description='z component of initial position, meters')

    roll = LaunchConfiguration('roll')
    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    pitch = LaunchConfiguration('pitch')
    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    yaw = LaunchConfiguration('yaw')
    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')

    # ROS2 launches
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(desc_pkg_path,
                         'launch',
                         'rsp_sim.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': jsp_gui,
            'use_camera': use_camera,
            'use_gazebo': use_gazebo,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
        }.items()
    )

    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_path,
                         'launch',
                         'load_ros2_controllers.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(load_controllers)
    )

    # Gazebo launches
    set_env_vars_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{sim_models_path}:{robot_models_path}')

    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', [' -r -v 4 ', world_path])]
    )

    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_path
        }],
        output='screen'
    )

    gz_ros_img_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera_head/depth_image',
            '/camera_head/image',
        ],
        remappings=[
            ('/camera_head/depth_image', '/camera_head/depth/image_rect_raw'),
            ('/camera_head/image', '/camera_head/color/image_raw'),
        ],
    )

    urdf_path = os.path.join(desc_pkg_path, 'model', 'arm.urdf.xacro')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            # '-topic', '/robot_description',
            '-string', robot_desc,
            '-name', 'arm',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ])

    return LaunchDescription([
        # Launch options
        declare_jsp_gui_cmd,
        declare_load_controllers_cmd,
        declare_use_camera_cmd,
        declare_use_gazebo_cmd,
        declare_use_rviz_cmd,
        declare_use_robot_state_pub_cmd,
        declare_use_sim_time_cmd,
        declare_world_cmd,
        # Pose arguments
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        # Actions
        set_env_vars_resources,
        robot_state_publisher_cmd,
        load_controllers_cmd,
        sim_cmd,
        gz_ros_bridge,
        gz_ros_img_bridge_cmd,
        start_gazebo_ros_spawner_cmd,
    ])
