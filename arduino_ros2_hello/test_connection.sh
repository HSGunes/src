#!/bin/bash

echo "=== Arduino Mega CH340 Bağlantı Testi ==="
echo "Port: /dev/ttyUSB0"
echo ""

# Port kontrolü
if [ -e "/dev/ttyUSB0" ]; then
    echo "✓ /dev/ttyUSB0 portu bulundu"
    ls -la /dev/ttyUSB0
else
    echo "✗ /dev/ttyUSB0 portu bulunamadı"
    exit 1
fi

echo ""
echo "=== Serial Port Test ==="
echo "Arduino'ya test komutu gönderiliyor..."

# Python ile basit test
python3 -c "
import serial
import time

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)
    
    # Test komutu gönder
    ser.write(b'GET_STATUS\\n')
    time.sleep(0.5)
    
    # Cevap oku
    if ser.in_waiting > 0:
        response = ser.readline().decode('utf-8').strip()
        print(f'✓ Arduino cevabı: {response}')
    else:
        print('✗ Arduino\'dan cevap alınamadı')
    
    ser.close()
    
except Exception as e:
    print(f'✗ Hata: {e}')
"

echo ""
echo "=== ROS2 Node Test ==="
echo "ROS2 node'u başlatılıyor (5 saniye çalışacak)..."
echo "Ctrl+C ile durdurabilirsiniz"

# ROS2 node'u test et
timeout 5s ros2 run arduino_ros2_hello arduino_bridge || echo "Node test tamamlandı"

echo ""
echo "Test tamamlandı!" 