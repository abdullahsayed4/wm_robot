#include "wm_robot_hardware/wm_robot_base_hardware_interface.hpp"
#include <sstream>

namespace wm_robot_driver {

// ╔══════════════════════════════════════════════════════════╗
// ║                   1.  on_init()                          ║
// ╚══════════════════════════════════════════════════════════╝
hardware_interface::CallbackReturn WmRobotBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(logger_, "=== on_init: بدء تهيئة الـ Hardware Interface ===");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "فشل تهيئة الـ SystemInterface الأساسية");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string serial_port = info_.hardware_parameters.at("serial_port");
  int baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(logger_, "Serial Port: %s | Baud Rate: %d",
              serial_port.c_str(), baud_rate);

  // حجم ثابت 2 عشان الـ Driver جوّاه num_wheels_ = 2 دايماً
  wheel_velocity_commands_.assign(2, 0.0);
  wheel_positions_.assign(2, 0.0);
  wheel_velocities_.assign(2, 0.0);

  bool success = driver_.init(serial_port, baud_rate);
  if (!success) {
    RCLCPP_ERROR(logger_, "فشل الاتصال بالـ Microcontroller على %s",
                 serial_port.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "✓ تمّ تهيئة الـ Hardware Interface بنجاح");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ╔══════════════════════════════════════════════════════════╗
// ║            2.  export_state_interfaces()                 ║
// ╚══════════════════════════════════════════════════════════╝
// بنمشي على أول 2 joints بس من الـ URDF (left + right wheel)
// [0] = left_wheel_joint  → position + velocity
// [1] = right_wheel_joint → position + velocity
std::vector<hardware_interface::StateInterface>
WmRobotBaseHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < 2; ++i)
  {
    state_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &wheel_positions_[i]
    );
    state_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &wheel_velocities_[i]
    );
  }

  RCLCPP_INFO(logger_, "✓ تم تسجيل %zu State Interfaces",
              state_interfaces.size());
  return state_interfaces;
}

// ╔══════════════════════════════════════════════════════════╗
// ║           3.  export_command_interfaces()                ║
// ╚══════════════════════════════════════════════════════════╝
std::vector<hardware_interface::CommandInterface>
WmRobotBaseHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < 2; ++i)
  {
    command_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &wheel_velocity_commands_[i]
    );
  }

  RCLCPP_INFO(logger_, "✓ تم تسجيل %zu Command Interfaces",
              command_interfaces.size());
  return command_interfaces;
}

// ╔══════════════════════════════════════════════════════════╗
// ║               4.  on_activate()                         ║
// ╚══════════════════════════════════════════════════════════╝
hardware_interface::CallbackReturn WmRobotBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(logger_, "=== on_activate: تشغيل الـ Hardware Interface ===");

  std::fill(wheel_velocity_commands_.begin(),
            wheel_velocity_commands_.end(), 0.0);
  std::fill(wheel_positions_.begin(), wheel_positions_.end(), 0.0);
  std::fill(wheel_velocities_.begin(), wheel_velocities_.end(), 0.0);

  RCLCPP_INFO(logger_, "✓ الـ Hardware Interface شغّال وجاهز");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ╔══════════════════════════════════════════════════════════╗
// ║              5.  on_deactivate()                        ║
// ╚══════════════════════════════════════════════════════════╝
hardware_interface::CallbackReturn WmRobotBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(logger_, "=== on_deactivate: إيقاف الـ Hardware Interface ===");

  // Safety Stop — بنبعت صفر للعجلتين عشان الروبوت يقف فوراً
  std::fill(wheel_velocity_commands_.begin(),
            wheel_velocity_commands_.end(), 0.0);

  driver_.update(wheel_velocity_commands_,
                 wheel_positions_,
                 wheel_velocities_);

  RCLCPP_INFO(logger_, "✓ تم إرسال أمر الإيقاف — الـ Hardware Interface متوقف");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ╔══════════════════════════════════════════════════════════╗
// ║                  6.  read()                             ║
// ╚══════════════════════════════════════════════════════════╝
// بتبعت آخر أوامر للتيينزي وبتاخد الـ Feedback الطازة فوراً
// في نفس اللفة — Controller يشوف قراءات حقيقية بدون تأخير
hardware_interface::return_type WmRobotBaseHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  driver_.update(wheel_velocity_commands_,
                 wheel_positions_,
                 wheel_velocities_);

  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500,
      "READ → Pos[L=%.3f R=%.3f] Vel[L=%.3f R=%.3f]",
      wheel_positions_[0], wheel_positions_[1],
      wheel_velocities_[0], wheel_velocities_[1]);

  return hardware_interface::return_type::OK;
}

// ╔══════════════════════════════════════════════════════════╗
// ║                  7.  write()                            ║
// ╚══════════════════════════════════════════════════════════╝
// بتخزن الأوامر الجديدة من الـ Controller في wheel_velocity_commands_
// الـ read() في اللفة الجاية هي اللي هتبعتها للتيينزي
hardware_interface::return_type WmRobotBaseHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500,
      "WRITE → VelCmd[L=%.3f R=%.3f] rad/s",
      wheel_velocity_commands_[0],
      wheel_velocity_commands_[1]);

  return hardware_interface::return_type::OK;
}

}  // namespace wm_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    wm_robot_driver::WmRobotBaseHardwareInterface,
    hardware_interface::SystemInterface)