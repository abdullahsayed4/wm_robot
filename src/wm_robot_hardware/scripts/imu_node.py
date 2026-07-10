#!/usr/bin/env python3
"""
imu_actuator_manual_node.py
────────────────────────────
Node واحدة بتعمل:
  • بتقرأ الـ IMU من الأردوينو وبتنشر على /imu/data
  • بتتحكم في الـ Linear Actuator يدوياً بالكيبورد:
        w → رفع (لحد ما يضرب limit switch فوق)
        s → نزول (لحد ما يضرب limit switch تحت)
        أي حاجة تانية / مفيش ضغط → وقف

Serial: /dev/ttyACM1 @ 115200

Publishes:
  /imu/data          → sensor_msgs/Imu
  /imu/calib_status  → std_msgs/String
  /actuator_status   → std_msgs/String
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import String

import serial
import serial.tools.list_ports
import threading
import sys
import tty
import termios
import time


# ── Covariances ───────────────────────────────────────────────────
ORIENT_COVARIANCE  = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]
ANG_VEL_COVARIANCE = [1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 1e-5]
LIN_ACC_COVARIANCE = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]


class ImuActuatorManualNode(Node):
    def __init__(self):
        super().__init__('imu_actuator_manual_node')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('serial_port',  '/dev/ttyACM1')
        self.declare_parameter('baud_rate',    115200)
        self.declare_parameter('frame_id',     'imu_link')

        port          = self.get_parameter('serial_port').value
        baud          = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # ── Serial ───────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'✅ Arduino connected: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Cannot open {port}: {e}')
            for p in serial.tools.list_ports.comports():
                self.get_logger().info(f'   {p.device} — {p.description}')
            raise SystemExit(1)

        # ── State ─────────────────────────────────────────────────
        self.serial_lock = threading.Lock()
        self.running     = True
        self.last_cmd_sent = None   # آخر أمر بعتناه فعلاً ('1', '0', أو None)

        # ── QoS ──────────────────────────────────────────────────
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ───────────────────────────────────────────
        self.imu_pub      = self.create_publisher(Imu,    '/imu/data',         imu_qos)
        self.calib_pub    = self.create_publisher(String, '/imu/calib_status', 10)
        self.actuator_pub = self.create_publisher(String, '/actuator_status',  10)

        # ── Read Thread (Serial من الأردوينو) ─────────────────────
        self.read_thread = threading.Thread(
            target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

        # ── Keyboard Thread ────────────────────────────────────────
        self.key_thread = threading.Thread(
            target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info(
            '🤖 IMU + Manual Actuator Node جاهز — اضغط w (رفع) / s (نزول)')

    # ═══════════════════════════════════════════════════════════
    #  Serial Read Loop — من الأردوينو
    # ═══════════════════════════════════════════════════════════
    def serial_read_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not raw:
                        continue

                    if raw.startswith('IMU,'):
                        self.parse_and_publish_imu(raw)

                    elif raw in ('LIFT_DONE', 'LOWER_DONE'):
                        self.get_logger().info(f'Arduino: {raw}')
                        self.publish_actuator_status(raw)
                        self.last_cmd_sent = None  # رجع للحالة IDLE

                    elif raw.startswith('INFO,') or raw.startswith('ERROR,') or raw.startswith('READY,'):
                        self.get_logger().info(f'Arduino: {raw}')

            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
            time.sleep(0.005)

    # ═══════════════════════════════════════════════════════════
    #  Keyboard Loop — w / s
    # ═══════════════════════════════════════════════════════════
    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running:
                key = sys.stdin.read(1)

                if key == 'w':
                    self.send_actuator_cmd('1')   # رفع
                elif key == 's':
                    self.send_actuator_cmd('0')   # نزول
                elif key == 'q':
                    self.get_logger().info('Quit key pressed.')
                    self.running = False
                    rclpy.shutdown()

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ═══════════════════════════════════════════════════════════
    #  Send Actuator Command (يبعت بس لو الأمر مختلف عن آخر أمر)
    # ═══════════════════════════════════════════════════════════
    def send_actuator_cmd(self, cmd):
        with self.serial_lock:
            if self.last_cmd_sent == cmd:
                return  # نفس الأمر شغال أصلاً، مش هنكرر
            self.ser.write((cmd + '\n').encode())
            self.last_cmd_sent = cmd

        if cmd == '1':
            self.get_logger().info('📤 W — رفع')
            self.publish_actuator_status('LIFTING')
        elif cmd == '0':
            self.get_logger().info('📤 S — نزول')
            self.publish_actuator_status('LOWERING')

    # ═══════════════════════════════════════════════════════════
    #  Publish Actuator Status
    # ═══════════════════════════════════════════════════════════
    def publish_actuator_status(self, status):
        msg = String()
        msg.data = status
        self.actuator_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════
    #  Parse & Publish IMU
    # ═══════════════════════════════════════════════════════════
    def parse_and_publish_imu(self, raw):
        parts = raw.split(',')
        if len(parts) != 16:
            self.get_logger().warn(f'Bad IMU line ({len(parts)} fields): {raw}')
            return

        try:
            _, _seq, qw, qx, qy, qz, gx, gy, gz, ax, ay, az, \
                cal_sys, cal_gyro, cal_acc, cal_mag = parts

            qw, qx, qy, qz = float(qw), float(qx), float(qy), float(qz)
            gx, gy, gz     = float(gx), float(gy), float(gz)
            ax, ay, az     = float(ax), float(ay), float(az)
            cal_sys        = int(cal_sys)

        except ValueError as e:
            self.get_logger().warn(f'IMU parse error: {e}')
            return

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation_covariance = ORIENT_COVARIANCE

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = ANG_VEL_COVARIANCE

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = LIN_ACC_COVARIANCE

        self.imu_pub.publish(msg)

        calib_msg = String()
        calib_msg.data = f'sys={cal_sys} gyro={cal_gyro} acc={cal_acc} mag={cal_mag}'
        self.calib_pub.publish(calib_msg)

        if cal_sys < 3:
            self.get_logger().warn(
                f'⚠️ IMU not calibrated — sys:{cal_sys} gyro:{cal_gyro} acc:{cal_acc} mag:{cal_mag}',
                throttle_duration_sec=5.0
            )

    # ═══════════════════════════════════════════════════════════
    #  Destroy
    # ═══════════════════════════════════════════════════════════
    def destroy_node(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuActuatorManualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()