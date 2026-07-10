import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_project_pkg = get_package_share_directory('wm_robot_project')
    urdf_file = os.path.join(robot_project_pkg, 'urdf', 'wm_robot.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    ekf_config = PathJoinSubstitution([
        FindPackageShare('wm_robot_bringup'),
        'config',
        'ekf_config.yaml'
    ])

    # 1. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. WmRobotNode — Teensy → motors + encoders + odometry
    robot_node = Node(
        package='wm_robot_hardware',
        executable='robot_node',
        name='wm_robot_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'wheel_radius': 0.1,
            'wheel_separation': 0.4,
            'cmd_vel_timeout': 1.0,
        }]
    )

    # 3. Static TF — base_link → imu_link
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        arguments=[
            '0.0', '0.0', '0.05',
            '0.0', '0.0', '0.0',
            'base_link', 'imu_link'
        ]
    )

    # 4. RPLIDAR
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        robot_node,
        imu_tf,
        rplidar_node,
    ])