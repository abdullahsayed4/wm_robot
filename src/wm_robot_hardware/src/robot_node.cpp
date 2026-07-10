#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "wm_robot_hardware/cytron_driver.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class WmRobotNode : public rclcpp::Node
{
public:
  WmRobotNode() : Node("wm_robot_node")
  {
    this->declare_parameter("serial_port",      "/dev/ttyACM0");
    this->declare_parameter("baud_rate",        115200);
    this->declare_parameter("wheel_radius",     0.1);
    this->declare_parameter("wheel_separation", 0.4);
    this->declare_parameter("cmd_vel_timeout",  0.5);
    this->declare_parameter("use_mock_hardware", false);   // ← جديد

    serial_port_     = this->get_parameter("serial_port").as_string();
    baud_rate_       = this->get_parameter("baud_rate").as_int();
    wheel_radius_    = this->get_parameter("wheel_radius").as_double();
    wheel_sep_       = this->get_parameter("wheel_separation").as_double();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
    use_mock_hardware_ = this->get_parameter("use_mock_hardware").as_bool();

    vel_commands_.assign(2, 0.0);
    wheel_positions_.assign(2, 0.0);
    wheel_velocities_.assign(2, 0.0);

    if (use_mock_hardware_) {
      RCLCPP_WARN(get_logger(),
          "⚠ MOCK MODE مفعّل — لن يتم الاتصال بالـ Microcontroller الحقيقي");
    } else {
      if (!driver_.init(serial_port_, baud_rate_)) {
        RCLCPP_ERROR(get_logger(),
            "فشل فتح Serial Port: %s — تفعيل MOCK MODE تلقائياً بدلاً من الإغلاق",
            serial_port_.c_str());
        use_mock_hardware_ = true;   // ← بدل الكراش، نرجع لـ mock تلقائيًا
      } else {
        RCLCPP_INFO(get_logger(),
            "✓ Serial متصل: %s @ %d", serial_port_.c_str(), baud_rate_);
      }
    }

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&WmRobotNode::cmdVelCallback, this, std::placeholders::_1));

    odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        20ms,
        std::bind(&WmRobotNode::controlLoop, this));

    last_cmd_time_  = this->now();
    last_loop_time_ = this->now();

    RCLCPP_INFO(get_logger(), "✓ WmRobotNode جاهز — Control Loop @ 50Hz%s",
        use_mock_hardware_ ? " (MOCK MODE)" : "");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_x_      = msg->linear.x;
    angular_z_     = msg->angular.z;
    last_cmd_time_ = this->now();
  }

  void controlLoop()
  {
    auto now = this->now();
    double dt = (now - last_loop_time_).seconds();
    last_loop_time_ = now;
    if (dt <= 0.0 || dt > 1.0) dt = 0.02;

    double time_since_cmd = (now - last_cmd_time_).seconds();
    if (time_since_cmd > cmd_vel_timeout_) {
      linear_x_  = 0.0;
      angular_z_ = 0.0;
    }

    vel_commands_[0] = (linear_x_ - angular_z_ * wheel_sep_ / 2.0) / wheel_radius_;
    vel_commands_[1] = (linear_x_ + angular_z_ * wheel_sep_ / 2.0) / wheel_radius_;

    if (use_mock_hardware_) {
      // ── Mock: نفترض إن العجل بينفّذ الأمر تمامًا (Open-loop) ──
      wheel_velocities_[0] = vel_commands_[0];
      wheel_velocities_[1] = vel_commands_[1];
      wheel_positions_[0] += wheel_velocities_[0] * dt;
      wheel_positions_[1] += wheel_velocities_[1] * dt;
    } else {
      driver_.update(vel_commands_, wheel_positions_, wheel_velocities_);
    }

    computeOdometry(dt);
    publishJointStates(now);
    publishOdometry(now);
    publishTF(now);
  }

  void computeOdometry(double dt)
  {
    double v_L = wheel_velocities_[0] * wheel_radius_;
    double v_R = wheel_velocities_[1] * wheel_radius_;
    double linear  = (v_R + v_L) / 2.0;
    double angular = (v_R - v_L) / wheel_sep_;

    odom_x_     += linear * std::cos(odom_theta_) * dt;
    odom_y_     += linear * std::sin(odom_theta_) * dt;
    odom_theta_ += angular * dt;

    while (odom_theta_ >  M_PI) odom_theta_ -= 2.0 * M_PI;
    while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;

    odom_vx_  = linear;
    odom_vth_ = angular;
  }

  void publishJointStates(const rclcpp::Time& now)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now;
    js.name         = {"wheel1_j", "wheel2_j"};
    js.position     = {wheel_positions_[0],  wheel_positions_[1]};
    js.velocity     = {wheel_velocities_[0], wheel_velocities_[1]};
    joint_state_pub_->publish(js);
  }

  void publishOdometry(const rclcpp::Time& now)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_theta_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";
    odom.pose.pose.position.x    = odom_x_;
    odom.pose.pose.position.y    = odom_y_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x  = odom_vx_;
    odom.twist.twist.angular.z = odom_vth_;
    odom.pose.covariance[0]   = 0.001;
    odom.pose.covariance[7]   = 0.001;
    odom.pose.covariance[35]  = 0.01;
    odom.twist.covariance[0]  = 0.001;
    odom.twist.covariance[7]  = 0.001;
    odom.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom);
  }

  void publishTF(const rclcpp::Time& now)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_theta_);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_footprint";
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x    = q.x();
    tf.transform.rotation.y    = q.y();
    tf.transform.rotation.z    = q.z();
    tf.transform.rotation.w    = q.w();

    tf_broadcaster_->sendTransform(tf);
  }

  wm_robot_driver::TeensyDriver driver_;
  std::string serial_port_;
  int         baud_rate_;
  double      wheel_radius_;
  double      wheel_sep_;
  double      cmd_vel_timeout_;
  bool        use_mock_hardware_ = false;   // ← جديد

  std::vector<double> vel_commands_;
  std::vector<double> wheel_positions_;
  std::vector<double> wheel_velocities_;

  double linear_x_  = 0.0;
  double angular_z_ = 0.0;
  double odom_x_     = 0.0;
  double odom_y_     = 0.0;
  double odom_theta_ = 0.0;
  double odom_vx_    = 0.0;
  double odom_vth_   = 0.0;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_loop_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_state_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                    tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WmRobotNode>());
  rclcpp::shutdown();
  return 0;
}
