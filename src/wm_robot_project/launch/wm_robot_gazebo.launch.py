from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package names
    robot_pkg = 'wm_robot_project'
    
    
    # Path configurations
    urdf_path = PathJoinSubstitution([
        FindPackageShare(robot_pkg), 
        'urdf', 
        'wm_robot.urdf.xacro'
    ])
    
    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare(robot_pkg), 
        'config', 
        'gazebo_bridge.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(robot_pkg), 
        'rviz', 
        'urdf_config.rviz'
    ])

    # 🔥 التعديل هنا فقط
    world_path = 'empty.sdf'

    # Add the package share directory to the Gazebo resource path
    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([FindPackageShare(robot_pkg), '..'])
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Gazebo Sim launch inclusion
    gz_sim_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': [world_path, ' -r']
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=['-topic', 'robot_description'],
        output='screen'
    )

    # ROS-Gazebo bridge
    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{
            'config_file': gazebo_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()
    
    ld.add_action(use_sim_time_arg)
    ld.add_action(gz_resource_path)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_sim_launch)
    ld.add_action(spawn_robot_node)
    ld.add_action(parameter_bridge_node)
    ld.add_action(rviz2_node)
    
    return ld