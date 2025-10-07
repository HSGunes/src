# Arduino ROS2 Serial Bridge - Arduino Mega CH340

Arduino Mega (CH340 chip) ve ROS2 Humble arasında serial bridge kullanarak publisher-subscriber mantığıyla çalışan bağlantı sistemi.

## Özellikler

- **Arduino Mega CH340 desteği**: `/dev/ttyUSB0` portu kullanır
- **Aktüatör kontrolü**: İleri/geri hareket kontrolü
- **Steering kontrolü**: Sol/sağ dönüş kontrolü
- **PWM kontrolü**: 0-255 arası PWM değeri ayarlama
- **Switch durumu**: Switch'lerin durumunu takip etme
- **Gerçek zamanlı iletişim**: Serial port üzerinden hızlı veri alışverişi

## Gereksinimler

### Arduino Tarafı
- Arduino Mega (CH340 USB-to-Serial chip)
- Aktüatör kontrol devresi
- Steering motor kontrol devresi
- Switch'ler (pin 43 ve 41)

### ROS2 Tarafı
- ROS2 Humble
- Python 3
- pyserial kütüphanesi

## Kurulum

### 1. Arduino Kodunu Yükleme
1. `arduino_code/simple_arduino.ino` dosyasını Arduino IDE'de açın
2. Arduino Mega'nızı USB ile bağlayın
3. **Board**: Arduino Mega 2560 seçin
4. **Port**: `/dev/ttyUSB0` seçin (CH340 chip için)
5. Kodu yükleyin

### 2. ROS2 Paketini Derleme
```bash
cd ~/ros2_ws
colcon build --packages-select arduino_ros2_hello
source install/setup.bash
```

### 3. Serial Port İzinleri
Arduino'nun bağlı olduğu port için izin verin:
```bash
sudo usermod -a -G dialout $USER
# Sistemi yeniden başlatın veya oturumu kapatıp açın
```

### 4. brltty Servisini Devre Dışı Bırakma
CH340 chip ile ilgili sorunları önlemek için:
```bash
sudo systemctl stop brltty
sudo systemctl disable brltty
```

## Kullanım

### 1. Bağlantı Testi
```bash
cd ~/ros2_ws/src/arduino_ros2_hello
./test_connection.sh
```

### 2. Arduino Bridge Node'unu Başlatma
```bash
ros2 run arduino_ros2_hello arduino_bridge
```

### 3. Komut Testi
```bash
cd ~/ros2_ws/src/arduino_ros2_hello
./simple_commands.sh
```

### 4. Manuel Test Komutları

#### Aktüatör Kontrolü
```bash
# Aktüatörü uzat
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'ACTUATOR_EXTEND'" --once

# Aktüatörü geri çek
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'ACTUATOR_RETRACT'" --once

# Aktüatörü durdur
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'ACTUATOR_STOP'" --once
```

#### Steering Kontrolü
```bash
# Sola dön
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'STEER_LEFT'" --once

# Sağa dön
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'STEER_RIGHT'" --once
```

#### PWM Kontrolü
```bash
# PWM değeri ayarla (0-255)
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'PWM:128'" --once

# PWM'i sıfırla
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'PWM:0'" --once
```

#### Durum Sorgulama
```bash
# Arduino durumunu sorgula
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'GET_STATUS'" --once
```

#### Topic'leri Dinleme
```bash
# Arduino'dan gelen verileri dinle
ros2 topic echo /arduino_status
```

## Topic Yapısı

### Publishers (Arduino'dan ROS2'ye)
- `/arduino_status` (std_msgs/String): Arduino'dan gelen tüm mesajlar

### Subscribers (ROS2'den Arduino'ya)
- `/arduino_cmd` (std_msgs/String): Arduino'ya gönderilecek komutlar

## Komut Listesi

### Aktüatör Komutları
- `ACTUATOR_EXTEND` - Aktüatörü uzatır
- `ACTUATOR_RETRACT` - Aktüatörü geri çeker
- `ACTUATOR_STOP` - Aktüatörü durdurur

### Steering Komutları
- `STEER_LEFT` - Sola döner
- `STEER_RIGHT` - Sağa döner

### PWM Komutları
- `PWM:value` - PWM değerini ayarlar (0-255)

### Sistem Komutları
- `GET_STATUS` - Arduino durumunu sorgular

## Yapılandırma

### Serial Port Ayarları
`arduino_bridge.py` dosyasında şu ayarları değiştirebilirsiniz:
```python
self.serial_port = '/dev/ttyUSB0'  # CH340 chip için
self.baud_rate = 115200            # Baud rate
```

### Arduino Pin Ayarları
`simple_arduino.ino` dosyasında pin tanımlamalarını değiştirebilirsiniz:
```cpp
const int pwmPin = 12;     // PWM çıkışı
const int R_Signal = 5;    // Aktüatör geri sinyali
const int F_Signal = 4;    // Aktüatör ileri sinyali
const int switch1 = 43;    // Switch 1
const int switch2 = 41;    // Switch 2
const int dir = 2;         // Steering yön
const int pulse = 7;       // Steering pulse
```

## Sorun Giderme

### Serial Bağlantı Hatası
- Arduino Mega'nın doğru porta bağlı olduğundan emin olun (`/dev/ttyUSB0`)
- Port izinlerini kontrol edin
- `brltty` servisinin durdurulduğundan emin olun
- Baud rate'in her iki tarafta da aynı olduğundan emin olun (115200)

### Port Bulunamıyor
```bash
# Mevcut portları kontrol et
ls -la /dev/ttyUSB* /dev/ttyACM*

# Driver'ı yeniden yükle
sudo modprobe -r ch341
sudo modprobe ch341
```

### Komut Çalışmıyor
- Arduino'dan gelen yanıtları kontrol edin
- Serial buffer'ın temiz olduğundan emin olun
- Switch durumlarını kontrol edin

## Test Scriptleri

### test_connection.sh
Bağlantıyı test eder ve Arduino'ya basit komut gönderir.

### simple_commands.sh
ROS2 topic'leri üzerinden Arduino'ya test komutları gönderir.

## Geliştirme

### Yeni Komut Ekleme
1. Arduino kodunda yeni komut işleyici ekleyin
2. ROS2 kodunda yeni subscriber oluşturun
3. String formatında veri gönderin

### Yeni Sensör Ekleme
1. Arduino'da yeni pin tanımlayın
2. Sensör okuma fonksiyonu ekleyin
3. ROS2'de yeni topic oluşturun

## Lisans

Bu proje MIT lisansı altında lisanslanmıştır. 