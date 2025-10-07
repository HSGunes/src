# RobotAksi Lane Detection and Following System

Bu paket, klasik bilgisayar görüsü tekniklerini kullanarak şerit tespiti yapan ve aracın şeridin ortasını takip etmesini sağlayan bir ROS2 sistemidir.

## Özellikler

- **Klasik Bilgisayar Görüsü Pipeline**: Kanıtlanmış CV teknikleri
  - Grayscale Conversion
  - Gaussian Blur
  - Canny Edge Detection
  - Region of Interest (ROI)
  - Hough Line Transform
  - Line Averaging & Extrapolation
- **PID Kontrol**: Hassas direksiyon kontrolü için PID algoritması
- **Adaptif Hız Kontrolü**: Virajlarda otomatik yavaşlama
- **Temporal Smoothing**: Titreşimi önlemek için geçmiş verilerle yumuşatma
- **Serial Kontrolör Entegrasyonu**: Arduino ile uyumlu motor kontrolü

## Algorithm Pipeline

```
1. Kamera Görüntüsü
   ↓
2. Grayscale Conversion (Gri Tonlama)
   ↓
3. Gaussian Blur (Gürültü Azaltma)
   ↓
4. Canny Edge Detection (Kenar Tespiti)
   ↓
5. Region of Interest (İlgi Bölgesi)
   ↓
6. Hough Line Transform (Çizgi Tespiti)
   ↓
7. Line Separation (Sol/Sağ Ayrımı)
   ↓
8. Line Averaging (Çizgi Ortalaması)
   ↓
9. Temporal Smoothing (Yumuşatma)
   ↓
10. Lane Center Calculation (Merkez Hesaplama)
    ↓
11. PID Control → cmd_vel
```

## Kurulum

```bash
# Workspace'e gidin
cd ~/ros2_ws

# Paketi build edin
colcon build --packages-select robotaksi_lane_detection robotaksi_serial_control

# Environment'ı source edin
source install/setup.bash
```

## Kullanım

### Temel Çalıştırma

```bash
# Parametreli çalıştırma (önerilen)
ros2 launch robotaksi_lane_detection lane_following_with_params.launch.py

# Veya manuel parametrelerle
ros2 launch robotaksi_lane_detection lane_following.launch.py serial_port:=/dev/ttyUSB0
```

### Parametreleri Ayarlama

`config/lane_following_params.yaml` dosyasını düzenleyerek parametreleri ayarlayabilirsiniz:

```yaml
# Canny Edge Detection
canny_low_threshold: 50   # Düşük eşik - artırın gürültüyü azaltmak için
canny_high_threshold: 150 # Yüksek eşik

# Hough Transform
hough_threshold: 20       # Çizgi tespit hassasiyeti
min_line_length: 40       # Minimum çizgi uzunluğu
max_line_gap: 20          # Maksimum çizgi aralığı

# PID Kontrolör
kp: 1.0                   # Oransal kazanç
ki: 0.05                  # İntegral kazanç
kd: 0.3                   # Türev kazanç
```

### Görselleştirme

Debug görüntüsünü görmek için:

```bash
ros2 run rqt_image_view rqt_image_view /lane_detection_debug
```

### Durum Takibi

```bash
# Şerit merkez ofsetini takip edin
ros2 topic echo /lane_center_offset

# Araç durumunu takip edin
ros2 topic echo /arduino_status

# Komut hızlarını takip edin
ros2 topic echo /cmd_vel
```

## Sistem Mimarisi

```
Kamera → Lane Detection Node → cmd_vel → Serial Controller → Arduino
                ↓
        Debug Image & Status
```

### Yayınlanan Topic'ler

- `/cmd_vel` (geometry_msgs/Twist): Araç kontrol komutları
- `/lane_center_offset` (std_msgs/Float32): Şerit merkez ofseti
- `/lane_detection_debug` (sensor_msgs/Image): Debug görüntüsü

### Abone Olunan Topic'ler

- `/camera/camera/color/image_raw` (sensor_msgs/Image): Kamera görüntüsü

## Parametre Açıklamaları

### Gaussian Blur
- `gaussian_kernel_size`: Bulanıklaştırma çekirdeği boyutu (tek sayı olmalı)

### Canny Edge Detection
- `canny_low_threshold`: Alt eşik (50-100 arası önerilir)
- `canny_high_threshold`: Üst eşik (100-200 arası önerilir)

### Hough Transform
- `hough_threshold`: Minimum oy sayısı (çizgi tespit hassasiyeti)
- `min_line_length`: Minimum çizgi uzunluğu (gürültüyü filtreler)
- `max_line_gap`: Maksimum çizgi aralığı (kesik çizgiler için)

### Kontrol
- `enable_speed_control`: Adaptif hız kontrolünü etkinleştir
- `enable_smoothing`: Temporal yumuşatmayı etkinleştir

## Sorun Giderme

### Şerit Tespit Edilmiyor
1. `canny_low_threshold` değerini düşürün (30-40)
2. `hough_threshold` değerini düşürün (15-20)
3. `min_line_length` değerini düşürün (30-35)

### Çok Fazla Gürültü/Yanlış Çizgi
1. `canny_low_threshold` değerini artırın (60-80)
2. `hough_threshold` değerini artırın (25-30)
3. `min_line_length` değerini artırın (50-60)

### Araç Çok Agresif Dönüyor
1. `kp` değerini düşürün (0.7-0.8)
2. `max_angular_velocity` değerini sınırlayın (0.8)

### Araç Yavaş Tepki Veriyor
1. `kp` değerini artırın (1.2-1.5)
2. `kd` değerini artırın (0.4-0.5)

### Kesik Şeritler Tespit Edilmiyor
1. `max_line_gap` değerini artırın (30-40)
2. `min_line_length` değerini düşürün (30-35)

## Gelişmiş Kullanım

### Özel Parametre Dosyası
```bash
ros2 launch robotaksi_lane_detection lane_following_with_params.launch.py \
  params_file:=/path/to/your/params.yaml
```

### Farklı Kamera Topic'i
```bash
ros2 launch robotaksi_lane_detection lane_following_with_params.launch.py \
  camera_topic:=/your/camera/topic
```

### ROI Ayarlama
ROI (Region of Interest) parametreleri kameranızdan alınan gerçek piksel koordinatlarına göre ayarlanmıştır:
- **Sol Alt**: (3, 401) → %0.5 genişlik, %83.5 yükseklik
- **Sol Üst**: (215, 234) → %33.6 genişlik, %48.8 yükseklik
- **Sağ Üst**: (396, 230) → %61.9 genişlik, %47.9 yükseklik
- **Sağ Alt**: (588, 383) → %92 genişlik, %79.8 yükseklik

Bu özel ROI ayarları sizin kameranızdan elle seçilmiş koordinatlara dayalıdır ve şerit çizgilerini mükemmel şekilde kapsar.

## Geliştirici Notları

- **Klasik CV Pipeline**: Kanıtlanmış ve güvenilir yöntem
- **Hızlı İşleme**: Gerçek zamanlı işleme için optimize edilmiş
- **Parametre Ayarlama**: Kolay parametre ayarlama ile farklı koşullara uyum
- **Temporal Smoothing**: Titreşimi önleyen yumuşatma algoritması
- **Confidence Scoring**: Tespit güvenilirliğine dayalı kontrol

### Performans
- Sistem 30 FPS'ye kadar çalışabilir
- Minimum 640x480 çözünürlük önerilir
- Arduino ile 115200 baud rate kullanılır
- PWM değerleri 0-255 arasında ölçeklenir

### Algoritma Detayları
Bu implementasyon Udacity Self-Driving Car Nanodegree ve benzer projelerde kullanılan klasik bilgisayar görüsü pipeline'ını temel alır. Kaggle ve GitHub'da yaygın olarak kullanılan kanıtlanmış yöntemleri içerir. 