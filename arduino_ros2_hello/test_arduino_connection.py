#!/usr/bin/env python3

import serial
import time
import glob
import sys

def find_arduino_port():
    """Arduino portunu bul"""
    ports = []
    
    # USB portları
    for port in glob.glob('/dev/ttyUSB*'):
        ports.append(port)
    
    # ACM portları
    for port in glob.glob('/dev/ttyACM*'):
        ports.append(port)
    
    return ports

def test_arduino_connection(port, baud_rate=115200):
    """Arduino bağlantısını test et"""
    try:
        print(f"Port {port} test ediliyor...")
        ser = serial.Serial(port, baud_rate, timeout=2)
        time.sleep(2)  # Arduino'nun hazır olmasını bekle
        
        # Serial buffer'ı temizle
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Test komutu gönder
        print("GET_STATUS komutu gönderiliyor...")
        ser.write(b'GET_STATUS\n')
        time.sleep(1)
        
        # Cevap oku
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"✓ Cevap alındı: {response}")
            ser.close()
            return True
        else:
            print("✗ Cevap alınamadı")
            ser.close()
            return False
            
    except Exception as e:
        print(f"✗ Bağlantı hatası: {e}")
        return False

def send_test_commands(port, baud_rate=115200):
    """Test komutları gönder"""
    try:
        ser = serial.Serial(port, baud_rate, timeout=2)
        time.sleep(2)
        
        commands = [
            "GET_STATUS",
            "LED_ON", 
            "LED_OFF",
            "TEST"
        ]
        
        for cmd in commands:
            print(f"\nKomut gönderiliyor: {cmd}")
            ser.write(f"{cmd}\n".encode('utf-8'))
            time.sleep(1)
            
            # Cevap oku
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                print(f"Cevap: {response}")
            else:
                print("Cevap alınamadı")
        
        ser.close()
        
    except Exception as e:
        print(f"Komut gönderme hatası: {e}")

def main():
    print("🔍 Arduino Bağlantı Test Aracı")
    print("=" * 40)
    
    # Portları bul
    ports = find_arduino_port()
    if not ports:
        print("❌ Hiçbir seri port bulunamadı!")
        print("💡 Arduino'nun USB ile bağlı olduğundan emin olun")
        return
    
    print(f"📋 Bulunan portlar: {ports}")
    
    # Her portu test et
    working_port = None
    for port in ports:
        if test_arduino_connection(port):
            working_port = port
            break
    
    if working_port:
        print(f"\n✅ Arduino bulundu: {working_port}")
        print("\n🧪 Test komutları gönderiliyor...")
        send_test_commands(working_port)
    else:
        print("\n❌ Arduino tespit edilemedi!")
        print("\n🔧 Sorun giderme önerileri:")
        print("1. Arduino'nun USB ile bağlı olduğundan emin olun")
        print("2. Arduino IDE'de doğru port seçili olduğundan emin olun")
        print("3. Arduino'ya test kodunu yükleyin (simple_test.ino)")
        print("4. Serial Monitor'ü kapatın (Arduino IDE'de)")
        print("5. USB kablosunu değiştirin")
        print("6. Farklı USB portu deneyin")

if __name__ == "__main__":
    main() 