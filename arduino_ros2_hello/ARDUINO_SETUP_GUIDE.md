# Arduino ROS2 Bağlantı Kurulum Rehberi

Bu rehber, Arduino ile ROS2 arasında serial bağlantı kurmak için adım adım talimatları içerir.

## 🔍 Sorun Tespiti

Mevcut durum: Arduino portu bulunuyor (`/dev/ttyUSB0`) ancak cevap alınamıyor.

## 📋 Adım Adım Çözüm

### 1. Arduino Fiziksel Bağlantısı

```bash
# Arduino'nun bağlı olduğunu kontrol et
ls -la /dev/ttyUSB*
lsusb | grep -i arduino
```

**Kontrol edilecekler:**
- [ ] Arduino USB kablosu bağlı mı?
- [ ] USB kablosu veri kablosu mu? (sadece şarj kablosu değil)
- [ ] Farklı USB portu denendi mi?

### 2. Arduino IDE Ayarları

**Arduino IDE'de:**
1. **Tools > Board** > Arduino Mega 2560 (veya kullandığınız board)
2. **Tools > Port** > `/dev/ttyUSB0` seçin
3. **Tools > Serial Monitor** > **KAPATIN** (çok önemli!)
4. **Tools > Baud Rate** > 115200

### 3. Test Kodunu Yükleme

**Arduino IDE'de:**
1. `arduino_code/simple_test.ino` dosyasını açın
2. **Upload** butonuna tıklayın
3. Yükleme tamamlandığında "Upload complete" mesajını görmelisiniz

### 4. Serial Monitor Testi

**Arduino IDE'de:**
1. **Tools > Serial Monitor** açın
2. Baud rate: 115200
3. Aşağıdaki komutları gönderin:
   ```
   GET_STATUS
   LED_ON
   LED_OFF
   TEST
   ```
4. Her komut için cevap almalısınız
5. **Serial Monitor'ü kapatın**

### 5. Python Test Scripti

```bash
cd ~/ros2_ws/src/arduino_ros2_hello
python3 test_arduino_connection.py
```

### 6. ROS2 Bridge Testi

```bash
# Terminal 1: Bridge'i başlat
ros2 run arduino_ros2_hello arduino_bridge

# Terminal 2: Komut gönder
ros2 topic pub /arduino_cmd std_msgs/msg/String "data: 'GET_STATUS'" --once

# Terminal 3: Cevabı dinle
ros2 topic echo /arduino_status
```

## 🔧 Yaygın Sorunlar ve Çözümleri

### Sorun 1: "Permission denied"
```bash
sudo usermod -a -G dialout $USER
# Oturumu kapatıp tekrar giriş yapın
```

### Sorun 2: "Device busy"
```bash
# Arduino IDE'de Serial Monitor'ü kapatın
# Veya sistemi yeniden başlatın
```

### Sorun 3: "Port not found"
```bash
# Arduino'yu çıkarıp tekrar takın
# Farklı USB portu deneyin
# USB kablosunu değiştirin
```

### Sorun 4: "No response from Arduino"
```bash
# Arduino'ya doğru kodu yüklediğinizden emin olun
# Baud rate'in her iki tarafta da aynı olduğunu kontrol edin
# Arduino'nun reset butonuna basın
```

## 📊 Test Sonuçları

### Başarılı Bağlantı
```
✅ Arduino bulundu: /dev/ttyUSB0
✅ Cevap alındı: STATUS:OK
✅ LED kontrolü çalışıyor
✅ Test komutları başarılı
```

### Başarısız Bağlantı
```
❌ Arduino tespit edilemedi
❌ Cevap alınamadı
❌ Port bulunamadı
```

## 🚀 Gelişmiş Test

### Manuel Serial Test
```bash
# Arduino'ya doğrudan bağlan
screen /dev/ttyUSB0 115200

# Komutları gönder
GET_STATUS
LED_ON
LED_OFF
TEST

# Çıkmak için: Ctrl+A, K
```

### Baud Rate Testi
```python
# Farklı baud rate'leri dene
baud_rates = [9600, 19200, 38400, 57600, 115200]
for baud in baud_rates:
    test_arduino_connection('/dev/ttyUSB0', baud)
```

## 📝 Kontrol Listesi

- [ ] Arduino USB ile bağlı
- [ ] Arduino IDE'de doğru board seçili
- [ ] Arduino IDE'de doğru port seçili
- [ ] Arduino IDE'de Serial Monitor kapalı
- [ ] Test kodu Arduino'ya yüklendi
- [ ] Serial Monitor'de komutlar çalışıyor
- [ ] Python test scripti başarılı
- [ ] ROS2 bridge çalışıyor
- [ ] ROS2 topic'ler çalışıyor

## 🆘 Hala Sorun Varsa

1. **Arduino'yu değiştirin** - Farklı bir Arduino deneyin
2. **USB kablosunu değiştirin** - Veri kablosu olduğundan emin olun
3. **Farklı USB portu deneyin** - USB 2.0 portu tercih edin
4. **Sistemi yeniden başlatın** - Tüm servisleri temizler
5. **Arduino IDE'yi yeniden başlatın** - Cache'i temizler

## 📞 Destek

Sorun devam ederse:
1. `dmesg | tail` çıktısını kontrol edin
2. `lsusb` çıktısını kontrol edin
3. Arduino IDE'deki hata mesajlarını not edin
4. Test scriptinin tam çıktısını paylaşın 