import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__('simple_serial_receiver')

        self.arduino_ = serial.Serial('/dev/ttyACM0', 921600, timeout=1)

        self.get_logger().info("Serial receiver started")

        self.pub_ = self.create_publisher(String, 'serial_receiver', 10)

        self.timer_ = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        if self.arduino_.in_waiting > 0:
            try:
                data = self.arduino_.readline().decode("utf-8").strip()
            except Exception as e:
                self.get_logger().error(str(e))
                return

            msg = String()
            msg.data = data

            self.pub_.publish(msg)


def main():
    rclpy.init()
    node = SimpleSerialReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()