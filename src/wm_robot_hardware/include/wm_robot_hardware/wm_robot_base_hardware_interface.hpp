#pragma once

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <cmath> // عشان M_PI
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "cytron_driver.hpp"

namespace wm_robot_driver
{

class WmRobotBaseHardwareInterface : public hardware_interface::SystemInterface
{
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WmRobotBaseHardwareInterface );

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // الـ Logger والـ Clock بأسماء مشروعك الجديد
  rclcpp::Logger logger_ = rclcpp::get_logger("wm_robot_hardware");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // كائن الدرايفر اللي عملناه
  TeensyDriver driver_;

  // --- Shared memory (البيانات اللي الـ ROS 2 Control بيحتاجها) ---
  
  // الأوامر (Commands): سرعة الموتور الشمال، سرعة الموتور اليمين، وسرعة الـ Actuator
  std::vector<double> wheel_velocity_commands_;

  // الحالة (States): مواضع وسرعات العجل (Feedback)
  std::vector<double> wheel_positions_;
  std::vector<double> wheel_velocities_;
  // --- التحويلات (Utilities) ---
  // الروبوت المتحرك غالباً بيتعامل بالـ Radian، فالدوال دي هتفضل مفيدة
  double degToRad(double deg) { return deg / 180.0 * M_PI; }
  double radToDeg(double rad) { return rad / M_PI * 180.0; }
};

} // namespace