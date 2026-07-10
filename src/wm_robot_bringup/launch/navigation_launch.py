from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Declare arguments with defaults pointing to package share
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('wm_robot_bringup'), 'maps', 'my_map1.yaml'
        ]),
        description='Full path to map yaml file'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wm_robot_bringup'), 'config', 'nav2_params.yaml'
        ]),
        description='Full path to nav2 params file'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':          'false',
            'map':                   LaunchConfiguration('map'),
            'params_file':           LaunchConfiguration('params_file'),
            'use_composition':       'False',
            'use_collision_monitor': 'False',
        }.items()
    )

    return LaunchDescription([map_arg, params_arg, nav2])
