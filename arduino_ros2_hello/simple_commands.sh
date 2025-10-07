#!/bin/bash

echo "=== Arduino Komut Testi ==="
echo "ROS2 topic'leri üzerinden Arduino'ya komut gönderiliyor..."
echo ""

# ROS2 node'u arka planda başlat
echo "1. Arduino bridge node'u başlatılıyor..."
ros2 run arduino_ros2_hello arduino_bridge &
BRIDGE_PID=$!

# Node'un başlaması için bekle
sleep 3

echo "2. Test komutları gönderiliyor..."

# Test komutları
echo "   - Durum sorgusu..."
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'GET_STATUS'" --once

sleep 1

echo "   - Aktüatör uzatma..."
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'ACTUATOR_EXTEND'" --once

sleep 2

echo "   - Aktüatör durdurma..."
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'ACTUATOR_STOP'" --once

sleep 1

echo "   - PWM test (128)..."
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'PWM:128'" --once

sleep 1

echo "   - PWM sıfırlama..."
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'PWM:0'" --once

sleep 1

echo "3. Arduino'dan gelen mesajları dinle (5 saniye)..."
timeout 5s ros2 topic echo /arduino_status

echo ""
echo "4. Bridge node'u durduruluyor..."
kill $BRIDGE_PID 2>/dev/null

echo "Test tamamlandı!" 