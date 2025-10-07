#!/bin/bash

echo "=== TEKER DÖNDÜRME TESTİ ==="
echo ""

# ROS2 workspace'i source et
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Arduino Bridge node'u başlatılıyor..."
ros2 run arduino_ros2_hello arduino_bridge &
BRIDGE_PID=$!

echo "Bridge başlatıldı (PID: $BRIDGE_PID)"
echo "3 saniye bekleniyor..."
sleep 3

echo ""
echo "Komutları test etmek için yeni bir terminal açın ve şu komutları çalıştırın:"
echo ""
echo "1. Sola dönüş:"
echo "   ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: STEER_LEFT'"
echo ""
echo "2. Sağa dönüş:"
echo "   ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: STEER_RIGHT'"
echo ""
echo "3. Durum kontrolü:"
echo "   ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: GET_STATUS'"
echo ""
echo "4. Arduino durumunu dinle:"
echo "   ros2 topic echo /arduino_status"
echo ""
echo "Çıkmak için Ctrl+C tuşlayın"
echo ""

# Bridge'i çalışır durumda tut
wait $BRIDGE_PID 