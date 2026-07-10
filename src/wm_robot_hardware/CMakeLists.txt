cmake_minimum_required(VERSION 3.8)
project(wm_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# ── المكتبة الأصلية (Hardware Interface) ──────────────────────
add_library(${PROJECT_NAME} SHARED
  src/wm_robot_base_hardware_interface.cpp
  src/cytron_driver.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

pluginlib_export_plugin_description_file(
  hardware_interface wm_robot_hardware_interface.xml)

# ── الـ Node الجديد (بدون ros2_control) ───────────────────────
add_executable(robot_node
  src/robot_node.cpp
  src/cytron_driver.cpp
)

ament_target_dependencies(robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(robot_node
  ${Boost_LIBRARIES}
)

# ── IMU Node (Python) ──────────────────────────────────────────
install(PROGRAMS
  scripts/imu_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── install ────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME} robot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()