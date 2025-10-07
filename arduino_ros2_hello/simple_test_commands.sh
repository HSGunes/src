#!/bin/bash

echo "=== Arduino ROS2 Serial Bridge - Basit Sistem ==="
echo ""

echo "1. Arduino kodunu yükleyin:"
echo "   - simple_arduino.ino dosyasını Arduino IDE'de açın"
echo "   - Arduino'ya yükleyin"
echo ""

echo "2. Node'ları başlatmak için:"
echo "   Terminal 1: ros2 run arduino_ros2_hello simple_bridge"
echo "   Terminal 2: ros2 run arduino_ros2_hello simple_test"
echo ""

echo "3. Manuel test komutları:"
echo "   LED aç: ros2 topic pub /led_control std_msgs/String 'data: led_on'"
echo "   LED kapat: ros2 topic pub /led_control std_msgs/String 'data: led_off'"
echo "   LED toggle: ros2 topic pub /led_control std_msgs/String 'data: led_toggle'"
echo "   Durum sorgula: ros2 topic pub /led_control std_msgs/String 'data: get_status'"
echo "   Mesaj gönder: ros2 topic pub /ros_to_arduino std_msgs/String 'data: Merhaba Arduino'"
echo ""

echo "4. Arduino'dan gelen verileri dinlemek için:"
echo "   ros2 topic echo /arduino_data"
echo ""

echo "5. Topic listesini görmek için:"
echo "   ros2 topic list"
echo ""

echo "6. Arduino'nun bağlı olduğu portu kontrol etmek için:"
echo "   ls /dev/ttyUSB*"
echo ""

echo "Not: Arduino'nun bağlı olduğundan ve doğru porta bağlandığından emin olun!" 