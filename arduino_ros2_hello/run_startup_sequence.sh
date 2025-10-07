#!/bin/bash

echo "=== ARAÇ BAŞLANGIÇ SEKANSI ==="
echo "Bu script aracın tekerlerini önce sola, sonra sağa döndürecek"
echo ""

# ROS2 workspace'i source et
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Arduino Bridge node'u başlatılıyor..."
echo "Arduino'nun /dev/ttyUSB0 portuna bağlı olduğundan emin olun!"
echo ""

# Arduino bridge'i arka planda başlat
ros2 run arduino_ros2_hello arduino_bridge &
BRIDGE_PID=$!

echo "Arduino Bridge başlatıldı (PID: $BRIDGE_PID)"
echo "5 saniye bekleniyor (bridge'in hazır olması için)..."
sleep 5

echo ""
echo "Bridge durumunu kontrol ediliyor..."
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "HATA: Arduino Bridge çöktü! Serial bağlantısını kontrol edin."
    echo "Arduino'nun bağlı olduğundan ve doğru portta olduğundan emin olun."
    exit 1
fi

echo "Bridge çalışıyor, başlangıç sekansı başlatılıyor..."
echo "1. Sola dönüş"
echo "2. Sağa dönüş"
echo ""

# Startup sequence'i çalıştır
python3 src/arduino_ros2_hello/startup_sequence.py

echo ""
echo "Sekans tamamlandı!"
echo "Arduino Bridge'i durdurmak için: kill $BRIDGE_PID"
echo ""

# Bridge'i durdur
if kill -0 $BRIDGE_PID 2>/dev/null; then
    kill $BRIDGE_PID
    echo "Arduino Bridge durduruldu."
else
    echo "Arduino Bridge zaten durmuş."
fi 