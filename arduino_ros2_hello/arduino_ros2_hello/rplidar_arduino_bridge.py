#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import serial
import time
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class RplidarArduinoBridge(Node):
    def __init__(self):
        super().__init__('rplidar_arduino_bridge')
        
        # Serial bağlantı
        self.serial_port = '/dev/ttyUSB1'
        self.baud_rate = 115200
        self.ser = None
        self.init_serial()
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'arduino_status', 10)
        self.distance_pub = self.create_publisher(Float32, 'current_distance', 10)
        self.obstacle_pub = self.create_publisher(Float32, 'obstacle_distance', 10)
        self.movement_status_pub = self.create_publisher(String, 'movement_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(String, 'arduino_cmd', self.send_cmd, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # RPLIDAR subscriber (opsiyonel)
        try:
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
            self.rplidar_available = True
            self.get_logger().info("RPLIDAR bağlantısı kuruldu")
        except Exception as e:
            self.rplidar_available = False
            self.get_logger().warn(f"RPLIDAR bağlantısı kurulamadı: {e}")
            self.get_logger().info("RPLIDAR olmadan çalışacak")
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            
        # Timer
        self.timer = self.create_timer(0.1, self.read_serial)
        self.movement_timer = self.create_timer(0.05, self.movement_control)
        
        # Hareket kontrolü
        self.is_moving = False
        self.target_distance = 0.0
        self.current_distance = 0.0
        self.start_position = None
        self.current_position = None
        self.obstacle_distance = float('inf')
        self.min_safe_distance = 0.5  # 50cm minimum güvenli mesafe
        
        # RPLIDAR parametreleri (45 derece ön algılama için)
        self.detection_angle = 0.0  # Ön ±45 derece
        self.emergency_stop_distance = 1.5  # 2 metre engel algılama mesafesi
        self.min_range = 0.05  # RPLIDAR S2 minimum mesafe
        self.max_range = 30.0  # RPLIDAR S2 maksimum mesafe
        
        # PWM kontrolü
        self.current_pwm = 50  # Varsayılan PWM değeri
        self.obstacle_detected = False  # Engel algılama durumu
        
        # Hareket parametreleri
        self.base_speed = 0.3  # m/s (daha yavaş başlangıç)
        self.max_speed = 0.8   # m/s
        self.current_speed = 0.0
        
        # Zaman takibi
        self.movement_start_time = None
        self.last_scan_time = 0
        self.last_odom_time = 0
        
        # SLAM durumu
        self.slam_ready = False
        self.map_frame_available = False
        
        self.get_logger().info("RPLIDAR destekli Arduino bridge başlatıldı!")
        
    def init_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.get_logger().info("Serial bağlantısı başarılı!")
        except Exception as e:
            self.get_logger().error(f"Serial bağlantı hatası: {e}")
    
    def odom_callback(self, msg):
        """Odometry verilerini işle"""
        self.last_odom_time = time.time()
        self.current_position = msg.pose.pose.position
        
        # SLAM hazır mı kontrol et
        if not self.slam_ready and self.check_slam_ready():
            self.slam_ready = True
            self.get_logger().info("✅ SLAM sistemi hazır!")
        
        # Hareket sırasında mesafe takibi
        if self.is_moving and self.start_position:
            self.current_distance = self.calculate_distance(
                self.start_position, self.current_position
            )
            
            # Arduino'ya pozisyon bilgisi gönder
            if self.ser:
                try:
                    cmd = f"SET_POSITION:{self.current_position.x:.3f},{self.current_position.y:.3f}\n"
                    self.ser.write(cmd.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f"Pozisyon gönderme hatası: {e}")
    
    def check_slam_ready(self):
        """SLAM sisteminin hazır olup olmadığını kontrol et"""
        try:
            # Map frame'den base_link'e transform var mı?
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.map_frame_available = True
            return True
        except Exception:
            return False
    
    def calculate_distance(self, start_pos, current_pos):
        """İki nokta arasındaki mesafeyi hesapla"""
        dx = current_pos.x - start_pos.x
        dy = current_pos.y - start_pos.y
        return math.sqrt(dx*dx + dy*dy)
    
    def scan_callback(self, msg):
        """RPLIDAR verilerini işle"""
        if not self.rplidar_available:
            return
            
        self.last_scan_time = time.time()
        
        # Ön ±45 derecelik alanı kontrol et
        front_angle_rad = math.radians(self.detection_angle)
        center_index = len(msg.ranges) // 2
        angle_increment = msg.angle_increment
        
        # Tarama aralığı
        start_index = center_index - int(front_angle_rad / angle_increment)
        end_index = center_index + int(front_angle_rad / angle_increment)
        
        # Geçerli indeksleri kontrol et
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)
        
        # En yakın engeli bul (RPLIDAR S2 parametreleri ile)
        min_distance = float('inf')
        
        for i in range(start_index, end_index + 1):
            if (msg.ranges[i] > self.min_range and 
                msg.ranges[i] < self.max_range and 
                msg.ranges[i] < min_distance):
                min_distance = msg.ranges[i]
        
        self.obstacle_distance = min_distance if min_distance != float('inf') else 10.0
        
        # Engel algılama ve PWM kontrolü
        if self.obstacle_distance < self.emergency_stop_distance:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.current_pwm = 0
                self.get_logger().warn("🚨 ARAÇ ENGEL ALGILADI VE DURDU!")
                print("🚨 ARAÇ ENGEL ALGILADI VE DURDU!")
            
            # Acil durdurma
            self.emergency_stop()
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.current_pwm = 50
                self.get_logger().info("✅ Engel temizlendi, hareket devam ediyor")
                print("✅ Engel temizlendi, hareket devam ediyor")
        
        # Arduino'ya engel mesafesini ve PWM değerini gönder
        if self.ser:
            try:
                cmd = f"SET_OBSTACLE:{self.obstacle_distance}\n"
                self.ser.write(cmd.encode('utf-8'))
                
                # PWM değerini gönder
                cmd = f"SET_PWM:{self.current_pwm}\n"
                self.ser.write(cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Engel mesafesi/PWM gönderme hatası: {e}")
    
    def movement_control(self):
        """Hareket kontrolü ve mesafe takibi"""
        if not self.is_moving:
            return
        
        # Hedef mesafeye ulaşıldı mı?
        if self.current_distance >= self.target_distance:
            self.stop_movement()
            self.get_logger().info(f"🎯 Hedef mesafeye ulaşıldı: {self.current_distance:.2f}m")
            return
        
        # PWM kontrolü (engel varsa 0, yoksa 50)
        if self.obstacle_detected:
            target_speed = 0.0
        else:
            target_speed = self.base_speed
        
        # Hızı yumuşak geçişle ayarla
        self.current_speed = 0.9 * self.current_speed + 0.1 * target_speed
        
        # cmd_vel ile hareket kontrolü
        twist_msg = Twist()
        twist_msg.linear.x = self.current_speed
        twist_msg.angular.z = 0.0  # Düz gitme
        self.cmd_vel_pub.publish(twist_msg)
        
        # Arduino'ya hız ve PWM gönder
        if self.ser:
            try:
                cmd = f"SET_SPEED:{self.current_speed}\n"
                self.ser.write(cmd.encode('utf-8'))
                
                cmd = f"SET_PWM:{self.current_pwm}\n"
                self.ser.write(cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Hız/PWM gönderme hatası: {e}")
    
    def send_cmd(self, msg):
        if self.ser:
            try:
                cmd = msg.data.strip() + '\n'
                self.ser.write(cmd.encode('utf-8'))
                self.get_logger().info(f"Komut gönderildi: {cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"Komut gönderme hatası: {e}")
    
    def read_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.process_arduino_message(line)
            except Exception as e:
                self.get_logger().error(f"Serial okuma hatası: {e}")
    
    def process_arduino_message(self, message):
        """Arduino'dan gelen mesajları işle"""
        if message.startswith("STATUS:"):
            # Durum mesajını parse et
            parts = message.split(":")
            for part in parts[1:]:
                if part.startswith("MOVING"):
                    self.is_moving = part.split(",")[0].split(":")[1] == "1"
                elif part.startswith("TARGET"):
                    self.target_distance = float(part.split(",")[0].split(":")[1])
                elif part.startswith("OBSTACLE"):
                    self.obstacle_distance = float(part.split(",")[0].split(":")[1])
                elif part.startswith("PWM"):
                    self.current_pwm = int(part.split(",")[0].split(":")[1])
        
        elif message == "MOVEMENT_STARTED":
            self.get_logger().info("🚀 Hareket başladı!")
            self.movement_start_time = time.time()
            self.is_moving = True
            # Başlangıçta PWM'i 128'e ayarla
            self.current_pwm = 64
            self.obstacle_detected = False
        
        elif message == "STOPPED":
            self.get_logger().warn("🛑 Hareket durduruldu!")
            #self.is_moving = False
        
        # Mesajı yayınla
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)
    
    def go_straight_distance(self, distance_meters):
        """Belirtilen mesafeye kadar düz git"""
        if not self.slam_ready:
            self.get_logger().warn("⚠️ SLAM sistemi henüz hazır değil!")
            return
        
        self.target_distance = distance_meters
        self.current_distance = 0.0
        self.start_position = self.current_position
        self.movement_start_time = None
        self.is_moving = True
        self.current_pwm = 50  # Başlangıç PWM değeri
        self.obstacle_detected = False
        
        # Arduino'ya başlangıç pozisyonunu gönder
        if self.ser and self.start_position:
            try:
                cmd = f"SET_START_POS:{self.start_position.x:.3f},{self.start_position.y:.3f}\n"
                self.ser.write(cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Başlangıç pozisyonu gönderme hatası: {e}")
        
        # Arduino'ya komut gönder
        self.send_cmd(String(data=f"GO_STRAIGHT:{distance_meters}"))
        self.get_logger().info(f"🎯 {distance_meters}m düz hareket başlatıldı")
        
        # Durum mesajı
        status_msg = String()
        status_msg.data = f"Starting {distance_meters}m straight movement with SLAM"
        self.movement_status_pub.publish(status_msg)
    
    def emergency_stop(self):
        """Acil durdurma"""
        # cmd_vel ile durdur
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        
        self.send_cmd(String(data="STOP"))
        #self.is_moving = False
        self.get_logger().warn("🚨 Acil durdurma - Engel çok yakın!")
        
        # Durum mesajı
        status_msg = String()
        status_msg.data = "EMERGENCY_STOP: Obstacle too close"
        self.movement_status_pub.publish(status_msg)
    
    def slow_down(self):
        """Yavaşla"""
        self.current_speed = self.base_speed * 0.3
        self.get_logger().warn(f"⚠️ Yavaşlama - Engel mesafesi: {self.obstacle_distance:.2f}m")
    
    def stop_movement(self):
        """Hareketi durdur"""
        # cmd_vel ile durdur
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        
        self.send_cmd(String(data="STOP"))
        self.is_moving = False
        self.get_logger().info("✅ Hareket tamamlandı")
        
        # Durum mesajı
        status_msg = String()
        status_msg.data = "Movement completed with SLAM"
        self.movement_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RplidarArduinoBridge()
    
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
