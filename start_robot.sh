#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/wm_robot/install/setup.bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller
