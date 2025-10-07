#!/bin/bash

echo "=== RPLIDAR SLAM ile 20 Metre Düz Gitme Testi ==="
echo ""

# Launch dosyasını başlat
echo "1. RPLIDAR SLAM ve Arduino bridge başlatılıyor..."
ros2 launch arduino_ros2_hello rplidar_arduino_bridge.launch.py &
LAUNCH_PID=$!

# SLAM'in hazır olmasını bekle
echo "2. SLAM sisteminin hazır olması bekleniyor (30 saniye)..."
sleep 30

echo "3. 20 metre düz hareket başlatılıyor..."
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'GO_STRAIGHT_20M'" --once

echo "4. Durum takibi (120 saniye)..."
echo "   - Hareket durumu:"
timeout 120s ros2 topic echo /movement_status &
echo "   - Engel mesafesi:"
timeout 120s ros2 topic echo /obstacle_distance &
echo "   - Mevcut mesafe:"
timeout 120s ros2 topic echo /current_distance &
echo "   - Robot yolu:"
timeout 120s ros2 topic echo /robot_path &

echo "5. Launch durduruluyor..."
kill $LAUNCH_PID 2>/dev/null

echo "Test tamamlandı!" 