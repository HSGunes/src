#!/bin/bash

echo "=== PWM TEST SCRIPT ==="
echo "PWM (Pulse Width Modulation) değerlerini test eder"
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
echo "=== PWM DEĞER TESTLERİ ==="
echo ""

echo "1. PWM: 0 (Tamamen kapalı - %0 güç)"
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: PWM:0' --once
sleep 2

echo "2. PWM: 64 (Düşük güç - %25 güç)"
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: PWM:64' --once
sleep 2

echo "3. PWM: 128 (Orta güç - %50 güç)"
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: PWM:128' --once
sleep 2

echo "4. PWM: 192 (Yüksek güç - %75 güç)"
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: PWM:192' --once
sleep 2

echo "5. PWM: 255 (Tam güç - %100 güç)"
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: PWM:255' --once
sleep 2

echo "6. PWM: 0 (Tekrar kapat)"
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: PWM:0' --once

echo ""
echo "=== DURUM KONTROLÜ ==="
ros2 topic pub /arduino_cmd std_msgs/msg/String 'data: GET_STATUS' --once

echo ""
echo "PWM test tamamlandı!"
echo "Bridge'i durdurmak için: kill $BRIDGE_PID"
echo ""

# Bridge'i durdur
kill $BRIDGE_PID
echo "Arduino Bridge durduruldu." 