#include "wm_robot_hardware/cytron_driver.hpp"
#include <chrono>
#include <stdexcept>
#include <thread>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include <termios.h>   // tcflush
#include <unistd.h>

#define FW_VERSION "2.1.0"

namespace wm_robot_driver {

// ═══════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════
TeensyDriver::TeensyDriver() : serial_port_(io_service_) {}

TeensyDriver::~TeensyDriver() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

// ═══════════════════════════════════════════════════════════
//  init()
// ═══════════════════════════════════════════════════════════
bool TeensyDriver::init(std::string port, int baudrate) {
  version_ = FW_VERSION;

  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_ERROR(logger_, "Failed to connect to serial port %s !!", port.c_str());
    return false;
  }

  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(
      static_cast<uint32_t>(baudrate)));
  serial_port_.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));

  RCLCPP_INFO(logger_, "Successfully connected to serial port %s @ %d",
              port.c_str(), baudrate);

  // مسح الـ buffer عند البداية
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  {
    int fd = serial_port_.native_handle();
    tcflush(fd, TCIOFLUSH);
    RCLCPP_INFO(logger_, "Serial buffer flushed.");
  }

  // تجهيز الـ vectors
  num_wheels_ = 2;
  hw_positions_.resize(num_wheels_, 0.0);
  hw_velocities_.resize(num_wheels_, 0.0);

  // ابدأ الـ read thread
  running_ = true;
  read_thread_ = std::thread(&TeensyDriver::readLoop, this);

  initialised_ = true;
  RCLCPP_INFO(logger_, "Driver initialised. Read thread started.");
  return true;
}

// ═══════════════════════════════════════════════════════════
//  readLoop() — thread مستقل بيقرأ من الـ Serial باستمرار
// ═══════════════════════════════════════════════════════════
void TeensyDriver::readLoop() {
  while (running_) {
    std::string inMsg;
    if (receive(inMsg)) {

      // تأكد إن فيه A
      size_t lastA = inMsg.rfind('A');
      if (lastA == std::string::npos) continue;

      // خد آخر رسالة كاملة بس
      std::string cleanMsg = inMsg.substr(lastA);

      // تأكد إن فيها B و C و D
      if (cleanMsg.find('B') == std::string::npos ||
          cleanMsg.find('C') == std::string::npos ||
          cleanMsg.find('D') == std::string::npos) continue;

      std::vector<double> data;
      parseValuesToVector<double>(cleanMsg, data);

      if (data.size() == 4) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        hw_positions_[0]  = data[0];  // posL  (rad)
        hw_positions_[1]  = data[1];  // posR  (rad)
        hw_velocities_[0] = data[2];  // velL  (rad/s)
        hw_velocities_[1] = data[3];  // velR  (rad/s)
      } else {
        RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
            "Parse failed: expected 4 values, got %zu | raw: [%s]",
            data.size(), inMsg.c_str());
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  sendCommands() — بيبعت الأوامر بس بدون ما يستنى رد
// ═══════════════════════════════════════════════════════════
void TeensyDriver::sendCommands(std::vector<double>& vel_commands) {
  std::string outMsg = "L";
  outMsg += std::to_string(vel_commands[0]);
  outMsg += "R";
  outMsg += std::to_string(vel_commands[1]);
  outMsg += "\n";

  std::string err;
  if (!transmit(outMsg, err)) {
    RCLCPP_ERROR_THROTTLE(logger_, clock_, 1000,
        "Failed to send commands: %s", err.c_str());
  }
}

// ═══════════════════════════════════════════════════════════
//  update() — للـ robot_node وللـ on_deactivate
// ═══════════════════════════════════════════════════════════
void TeensyDriver::update(std::vector<double>& vel_commands,
                          std::vector<double>& wheel_states,
                          std::vector<double>& wheel_velocities) {
  // log vel_commands
  std::string logInfo = "Wheel Vel Cmd: ";
  for (int i = 0; i < 2; i++) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << vel_commands[i];
    logInfo += std::to_string(i) + ": " + ss.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  // بعت الأمر بس
  sendCommands(vel_commands);

  // رجّع آخر قيم اتقرأت من الـ thread
  getwheelPositions(wheel_states);
  getwheelVelocities(wheel_velocities);
}

// ═══════════════════════════════════════════════════════════
//  getwheelPositions() / getwheelVelocities()
// ═══════════════════════════════════════════════════════════
void TeensyDriver::getwheelPositions(std::vector<double>& wheel_positions) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  wheel_positions = hw_positions_;
}

void TeensyDriver::getwheelVelocities(std::vector<double>& wheel_velocities) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  wheel_velocities = hw_velocities_;
}

// ═══════════════════════════════════════════════════════════
//  transmit()
// ═══════════════════════════════════════════════════════════
bool TeensyDriver::transmit(std::string msg, std::string& err) {
  boost::system::error_code ec;
  const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());
  boost::asio::write(serial_port_, sendBuffer, ec);

  if (!ec) {
    return true;
  } else {
    err = ec.message();
    return false;
  }
}

// ═══════════════════════════════════════════════════════════
//  receive() — blocking بس في thread منفصل
// ═══════════════════════════════════════════════════════════
bool TeensyDriver::receive(std::string& inMsg) {
  char c;
  inMsg = "";
  boost::system::error_code ec;

  while (running_) {
    size_t n = boost::asio::read(serial_port_,
                                  boost::asio::buffer(&c, 1), ec);
    if (ec) {
      RCLCPP_ERROR_THROTTLE(logger_, clock_, 1000,
          "Serial read error: %s", ec.message().c_str());
      return false;
    }

    if (n == 0) continue;

    if (c == '\n') {
      return true;
    } else if (c == '\r') {
      continue;
    } else {
      inMsg += c;
    }

    if (inMsg.length() > 100) {
      RCLCPP_WARN(logger_, "Receive buffer overflow, clearing...");
      inMsg = "";
    }
  }
  return false;
}

// ═══════════════════════════════════════════════════════════
//  parseValuesToVector()
// ═══════════════════════════════════════════════════════════
template <typename T>
void TeensyDriver::parseValuesToVector(const std::string msg,
                                        std::vector<T>& values) {
  values.clear();

  size_t firstA = msg.find('A', 0);
  if (firstA == std::string::npos) return;

  size_t prevIdx = firstA + 1;

  for (size_t i = 1;; ++i) {
    char currentIdentifier = 'A' + i;
    size_t currentIdx = msg.find(currentIdentifier, 0);

    try {
      if (currentIdx == std::string::npos) {
        std::string lastSegment = msg.substr(prevIdx);
        if (!lastSegment.empty()) {
          if constexpr (std::is_same<T, int>::value)
            values.push_back(std::stoi(lastSegment));
          else if constexpr (std::is_same<T, double>::value)
            values.push_back(std::stod(lastSegment));
        }
        break;
      }

      std::string segment = msg.substr(prevIdx, currentIdx - prevIdx);
      if constexpr (std::is_same<T, int>::value)
        values.push_back(std::stoi(segment));
      else if constexpr (std::is_same<T, double>::value)
        values.push_back(std::stod(segment));

    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Parse error in segment: %s", e.what());
    }

    prevIdx = currentIdx + 1;
  }
}

template void TeensyDriver::parseValuesToVector<double>(
    const std::string, std::vector<double>&);

}  // namespace wm_robot_driver