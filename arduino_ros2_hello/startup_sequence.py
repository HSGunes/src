#!/usr/bin/env python3
"""
Araç başlangıç sekansı: Önce sola, sonra sağa dönüş
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class StartupSequence(Node):
    def __init__(self):
        super().__init__('startup_sequence')
        
        # Komut göndermek için publisher
        self.cmd_pub = self.create_publisher(String, 'arduino_cmd', 10)
        
        # Durum mesajlarını dinlemek için subscriber
        self.status_sub = self.create_subscription(String, 'arduino_status', self.status_callback, 10)
        
        self.get_logger().info("Başlangıç sekansı başlatılıyor...")
        self.get_logger().info("Arduino Bridge'in hazır olması bekleniyor...")
        
        # 8 saniye bekle, sonra sekansı başlat (bridge'in tamamen hazır olması için)
        self.timer = self.create_timer(8.0, self.start_sequence)
        self.sequence_step = 0
        self.sequence_complete = False

    def status_callback(self, msg):
        """Arduino'dan gelen durum mesajlarını işle"""
        self.get_logger().info(f"Arduino durumu: {msg.data}")

    def start_sequence(self):
        """Başlangıç sekansını başlat"""
        if self.sequence_complete:
            return
            
        if self.sequence_step == 0:
            # İlk adım: Sola dön
            self.get_logger().info("=== ADIM 1: SOLA DÖNÜŞ ===")
            cmd = String()
            cmd.data = "STEER_LEFT"
            self.cmd_pub.publish(cmd)
            self.sequence_step = 1
            
            # 3 saniye bekle, sonra sağa dön
            self.timer.cancel()
            self.timer = self.create_timer(3.0, self.start_sequence)
            
        elif self.sequence_step == 1:
            # İkinci adım: Sağa dön
            self.get_logger().info("=== ADIM 2: SAĞA DÖNÜŞ ===")
            cmd = String()
            cmd.data = "STEER_RIGHT"
            self.cmd_pub.publish(cmd)
            self.sequence_step = 2
            
            # 3 saniye bekle, sonra tamamlandı mesajı
            self.timer.cancel()
            self.timer = self.create_timer(3.0, self.start_sequence)
            
        elif self.sequence_step == 2:
            # Sekans tamamlandı
            self.get_logger().info("=== BAŞLANGIÇ SEKANSI TAMAMLANDI ===")
            self.get_logger().info("Araç hazır! Tekerler önce sola, sonra sağa döndürüldü.")
            self.sequence_complete = True
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = StartupSequence()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 