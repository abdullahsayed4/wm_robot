import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__('simple_serial_transmitter')
        self.arduino_ = serial.Serial('/dev/ttyACM0', 115200)
        self.get_logger().info("led_task_subscriber Started")
        self.sub_=self.create_subscription(String, '/serial_transmitter', self.msgcallback, 10)

    def msgcallback(self, msg):
        self.arduino_.write(msg.data.encode("utf-8"))
        self.get_logger().info(f"LED Command: {msg.data}")

def main():
    rclpy.init()
    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()