#!/bin/bash

echo "=== İLETİŞİM TESTİ ==="
echo ""

# ROS2 workspace'i source et
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Arduino Bridge node'u başlatılıyor..."
ros2 run arduino_ros2_hello arduino_bridge &
BRIDGE_PID=$!

echo "Bridge başlatıldı (PID: $BRIDGE_PID)"
echo "5 saniye bekleniyor..."
sleep 5

echo ""
echo "Arduino durumunu dinleme başlatılıyor..."
echo "Yeni bir terminal açın ve şu komutu çalıştırın:"
echo "ros2 topic echo /arduino_status"
echo ""

echo "Test komutları gönderiliyor..."
echo "1. Durum kontrolü..."
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: GET_STATUS' --once

echo "2. Sola dönüş testi..."
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: STEER_LEFT' --once

echo "3. 3 saniye bekleniyor..."
sleep 3

echo "4. Sağa dönüş testi..."
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: STEER_RIGHT' --once

echo ""
echo "Test tamamlandı!"
echo "Bridge'i durdurmak için: kill $BRIDGE_PID"
echo ""

# Bridge'i durdur
kill $BRIDGE_PID
echo "Arduino Bridge durduruldu." 