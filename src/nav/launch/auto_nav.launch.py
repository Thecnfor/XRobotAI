import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    nav2_default_params = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')
    params_file = LaunchConfiguration('params_file')
    goal_mode = LaunchConfiguration('goal_mode')
    relative_dx = LaunchConfiguration('relative_dx')
    relative_dy = LaunchConfiguration('relative_dy')
    relative_dyaw = LaunchConfiguration('relative_dyaw')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/workspace/XRobotAI/usd/build.yaml')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True')
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='True')
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_default_params)
    declare_goal_mode = DeclareLaunchArgument(
        'goal_mode',
        default_value='relative')
    declare_relative_dx = DeclareLaunchArgument(
        'relative_dx',
        default_value='1.0')
    declare_relative_dy = DeclareLaunchArgument(
        'relative_dy',
        default_value='0.0')
    declare_relative_dyaw = DeclareLaunchArgument(
        'relative_dyaw',
        default_value='0.0')
    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom')
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'slam': slam,
            'params_file': params_file,
        }.items(),
    )

    auto_nav = Node(
        package='nav',
        executable='auto_nav',
        output='screen',
        parameters=[{
            'laser_in': '/laser_scan',
            'laser_out': '/scan',
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'publish_initial_pose': True,
            'initial_pose_source': 'odom',
            'goal_mode': goal_mode,
            'relative_dx': ParameterValue(relative_dx, value_type=float),
            'relative_dy': ParameterValue(relative_dy, value_type=float),
            'relative_dyaw': ParameterValue(relative_dyaw, value_type=float),
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        declare_autostart,
        declare_slam,
        declare_params_file,
        declare_goal_mode,
        declare_relative_dx,
        declare_relative_dy,
        declare_relative_dyaw,
        declare_odom_frame,
        declare_base_frame,
        auto_nav,
        nav2_bringup,
    ])
