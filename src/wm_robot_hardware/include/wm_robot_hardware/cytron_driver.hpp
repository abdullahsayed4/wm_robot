#ifndef TEENSY_DRIVER_H
#define TEENSY_DRIVER_H

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include "math.h"
#include "time.h"

namespace wm_robot_driver {

class TeensyDriver {
public:
  TeensyDriver();
  ~TeensyDriver();

  bool init(std::string port, int baudrate);

  // بيبعت الأوامر بس بدون ما يستنى رد
  void sendCommands(std::vector<double>& vel_commands);

  // update — محتاجه لـ on_deactivate وللـ robot_node
  void update(std::vector<double>& vel_commands,
              std::vector<double>& wheel_states,
              std::vector<double>& wheel_velocities);

  void getwheelPositions(std::vector<double>& wheel_positions);
  void getwheelVelocities(std::vector<double>& wheel_velocities);

private:
  bool initialised_ = false;

  // Serial
  boost::asio::io_service   io_service_;
  boost::asio::serial_port  serial_port_;

  // Read thread
  std::thread        read_thread_;
  std::atomic<bool>  running_{false};
  std::mutex         data_mutex_;

  // Data
  std::string version_;
  int num_wheels_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("teensy_driver");
  rclcpp::Clock  clock_  = rclcpp::Clock(RCL_ROS_TIME);

  // Internal
  void readLoop();
  bool transmit(std::string msg, std::string& err);
  bool receive(std::string& inMsg);

  template <typename T>
  void parseValuesToVector(const std::string msg, std::vector<T>& values);
};

}  // namespace wm_robot_driver
#endif  // TEENSY_DRIVER_H