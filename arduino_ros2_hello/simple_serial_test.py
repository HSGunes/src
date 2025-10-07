#!/usr/bin/env python3

import serial
import time

def test_basic_communication():
    """Arduino ile temel iletişimi test et"""
    try:
        print("🔍 Arduino ile temel iletişim testi")
        print("=" * 40)
        
        # Serial bağlantısını aç
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
        print("✅ Serial port açıldı")
        
        # Arduino'nun hazır olmasını bekle
        time.sleep(3)
        print("⏳ Arduino hazırlanıyor...")
        
        # Buffer'ları temizle
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Basit karakter gönder
        print("📤 'A' karakteri gönderiliyor...")
        ser.write(b'A')
        time.sleep(0.5)
        
        # Cevap var mı kontrol et
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"📥 Cevap alındı: {response}")
        else:
            print("❌ Cevap alınamadı")
        
        # Yeni satır gönder
        print("📤 Yeni satır gönderiliyor...")
        ser.write(b'\n')
        time.sleep(0.5)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"📥 Cevap alındı: {response}")
        else:
            print("❌ Cevap alınamadı")
        
        # Test komutu gönder
        print("📤 'TEST' komutu gönderiliyor...")
        ser.write(b'TEST\n')
        time.sleep(1)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"📥 Cevap alındı: {response}")
        else:
            print("❌ Cevap alınamadı")
        
        # Arduino'dan gelen tüm verileri oku
        print("📤 5 saniye boyunca veri dinleniyor...")
        start_time = time.time()
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"📥 Veri: {data}")
            time.sleep(0.1)
        
        ser.close()
        print("✅ Test tamamlandı")
        
    except Exception as e:
        print(f"❌ Hata: {e}")

if __name__ == "__main__":
    test_basic_communication() 