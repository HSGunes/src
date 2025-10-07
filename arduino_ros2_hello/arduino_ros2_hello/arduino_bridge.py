#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        # Arduino Mega CH340 için port ayarı
        self.serial_port = '/dev/ttyUSB1'  # CH340 chip ttyUSB0 kullanır
        self.baud_rate = 115200
        self.ser = None

        # Publisher: Arduino'dan gelen cevaplar
        self.status_pub = self.create_publisher(String, 'arduino_status', 10)
        # Subscriber: Komutları dinle
        self.cmd_sub = self.create_subscription(String, 'arduino_cmd', self.send_cmd, 10)

        self.init_serial()
        self.timer = self.create_timer(0.05, self.read_serial)

    def init_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)
            self.get_logger().info("Serial bağlantısı başarılı!")
            self.get_logger().info(f"Port: {self.serial_port}, Baud: {self.baud_rate}")
        except Exception as e:
            self.get_logger().error(f"Serial bağlantı hatası: {e}")
            self.get_logger().info("Arduino Mega'nın bağlı olduğundan emin olun!")

    def send_cmd(self, msg):
        if self.ser:
            cmd = msg.data.strip() + '\n'
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(f"Arduino'ya gönderildi: {cmd.strip()}")

    def read_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                ros_msg = String()
                ros_msg.data = line
                self.status_pub.publish(ros_msg)
                self.get_logger().info(f"Arduino'dan: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
