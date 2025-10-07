#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class SimpleRplidarTest(Node):
    def __init__(self):
        super().__init__('simple_rplidar_test')
        
        # Subscribe to scan topic
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        self.get_logger().info("RPLIDAR test node başlatıldı...")
        self.get_logger().info("/scan topic'ini dinliyor...")
        
    def scan_callback(self, msg):
        """Scan verilerini işle"""
        self.get_logger().info(f"✅ RPLIDAR çalışıyor! {len(msg.ranges)} veri noktası")
        
        # İlk birkaç veriyi göster
        if len(msg.ranges) > 0:
            min_range = min(msg.ranges)
            max_range = max(msg.ranges)
            self.get_logger().info(f"   Min mesafe: {min_range:.2f}m")
            self.get_logger().info(f"   Max mesafe: {max_range:.2f}m")
            self.get_logger().info(f"   Açı aralığı: {msg.angle_min:.2f} - {msg.angle_max:.2f}")
        
        # Sadece ilk mesajı göster ve çık
        self.get_logger().info("🎉 RPLIDAR başarıyla çalışıyor!")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRplidarTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 