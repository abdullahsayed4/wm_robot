from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('wm_robot_project'), 
            'urdf', 
            'wm_robot.urdf.xacro'
        ]),
        description='Path to the URDF/XACRO file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('wm_robot_project'), 
            'rviz', 
            'urdf_config.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # Use launch configurations
    urdf_path = LaunchConfiguration('urdf_path')
    rviz_config_path = LaunchConfiguration('rviz_config')
    
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        urdf_path_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node
    ])