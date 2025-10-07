/*
 * Basit Arduino ROS2 Serial Bridge
 * ArduinoJson kütüphanesi olmadan, sadece serial iletişim
 */

const int pwmPin = 12;    
const int R_Signal = 5;  
const int F_Signal = 4;   
const int switch1 = 43;   
const int switch2 = 41;   
const int dir = 2;       
const int pulse = 7;     
const int volt = 49;    

// Durum takibi için değişkenler
bool lastSwitch1State = false;
bool lastSwitch2State = false;
bool brakeMaxSent = false;
int currentPwmValue = 0;  // PWM değerini takip etmek için

void setup() {
  Serial.begin(115200); 

  pinMode(pwmPin, OUTPUT);
  pinMode(R_Signal, OUTPUT);
  pinMode(F_Signal, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(pulse, OUTPUT);
  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
  pinMode(volt, OUTPUT);

  analogWrite(pwmPin, 0);
  currentPwmValue = 0;  // Başlangıçta PWM 0
  digitalWrite(R_Signal, LOW);
  digitalWrite(F_Signal, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(pulse, LOW);
  digitalWrite(volt, LOW);
  
  Serial.println("Arduino hazır!");
}

void loop() {
  // Switch durumlarını kontrol et
  bool currentSwitch1 = digitalRead(switch1);
  bool currentSwitch2 = digitalRead(switch2);
  
  // Durum değişikliği varsa bildir
  if (currentSwitch1 != lastSwitch1State || currentSwitch2 != lastSwitch2State) {
    if (currentSwitch1 == HIGH && currentSwitch2 == LOW) {
      if (!brakeMaxSent) {
        Serial.println("BRAKE_MAX");
        brakeMaxSent = true;
      }
    } else {
      brakeMaxSent = false;
    }
    
    lastSwitch1State = currentSwitch1;
    lastSwitch2State = currentSwitch2;
  }

  // Serial komutları işle
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); 

    if (cmd == "ACTUATOR_EXTEND") {
      digitalWrite(R_Signal, HIGH);
      digitalWrite(F_Signal, LOW);
      Serial.println("ACTUATOR_EXTENDING");
    }
    else if (cmd == "ACTUATOR_RETRACT") {
      digitalWrite(R_Signal, LOW);
      digitalWrite(F_Signal, HIGH);
      Serial.println("ACTUATOR_RETRACTING");
    }
    else if (cmd == "ACTUATOR_STOP") {
      digitalWrite(R_Signal, LOW);
      digitalWrite(F_Signal, LOW);
      Serial.println("ACTUATOR_STOPPED");
    }
    else if (cmd == "STEER_LEFT") {
      digitalWrite(dir, HIGH);
      for (int i = 0; i < 500; i++) {
        digitalWrite(pulse, HIGH);
        delayMicroseconds(1000);
        digitalWrite(pulse, LOW);
        delayMicroseconds(1000);
      }
      Serial.println("STEERED_LEFT");
    }
    else if (cmd == "STEER_RIGHT") {
      digitalWrite(dir, LOW);
      for (int i = 0; i < 500; i++) {
        digitalWrite(pulse, HIGH);
        delayMicroseconds(1000);
        digitalWrite(pulse, LOW);
        delayMicroseconds(1000);
      }
      Serial.println("STEERED_RIGHT");
    }
    else if (cmd.startsWith("PWM:")) {
      int value = cmd.substring(4).toInt();
      value = constrain(value, 0, 255);
      analogWrite(pwmPin, value);
      currentPwmValue = value;  // PWM değerini kaydet
      Serial.print("PWM_SET:");
      Serial.println(value);
    }
    else if (cmd == "GET_STATUS") {
      Serial.print("STATUS:SW1:");
      Serial.print(currentSwitch1);
      Serial.print(",SW2:");
      Serial.print(currentSwitch2);
      Serial.print(",PWM:");
      Serial.println(currentPwmValue);  // Kaydedilen PWM değerini gönder
    }
    else {
      Serial.println("UNKNOWN_COMMAND");
    }
  }
  
  delay(10); // Küçük gecikme
} 