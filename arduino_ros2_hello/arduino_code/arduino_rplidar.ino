		/*
		 * Arduino ROS2 Serial Bridge - RPLIDAR Destekli
		 * RPLIDAR S2 ile uyumlu, SLAM tabanlı hareket kontrolü
		 */

		const int pwmPin = 12;    
		const int R_Signal = 5;  
		const int F_Signal = 4;   
		const int switch1 = 43;   
		const int switch2 = 41;   
		const int dir = 2;       
		const int pulse = 7;     
		const int volt = 49;    

		// Durum takibi
		bool lastSwitch1State = false;
		bool lastSwitch2State = false;
		bool brakeMaxSent = false;
		int currentPwmValue = 0;
		bool isMoving = false;
		bool emergencyStop = false;
		bool obstacleDetected = false;  // Engel algılama durumu
		float lastObstacleDistance = 999.0;  // Son engel mesafesi

		// SLAM tabanlı hareket parametreleri
		float targetDistance = 0.0;  // Hedef mesafe (ROS2'den gelecek)
		float currentSpeed = 0.0;    // Mevcut hız (ROS2'den gelecek)
		float obstacleDistance = 0.0; // Engel mesafesi (ROS2'den gelecek)
		float robotX = 0.0;          // Robot X pozisyonu (ROS2'den gelecek)
		float robotY = 0.0;          // Robot Y pozisyonu (ROS2'den gelecek)
		float startX = 0.0;          // Başlangıç X pozisyonu
		float startY = 0.0;          // Başlangıç Y pozisyonu

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

		  // Başlangıç durumu
		  stopMotors();
		  
		  Serial.println("Arduino RPLIDAR destekli hazır!");
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

		  // Engel kontrolü - otomatik PWM güncelleme
		  checkObstacleAndUpdatePWM();

		  // Serial komutları işle
		  if (Serial.available()) {
		    String cmd = Serial.readStringUntil('\n');
		    cmd.trim(); 
		    processCommand(cmd);
		  }
		  
		  // Acil durdurma kontrolü
		  if (emergencyStop) {
		    stopMotors();
		    isMoving = false;
		  }
		  
		  delay(10);
		}

		void checkObstacleAndUpdatePWM() {
		  // Engel mesafesi değişti mi kontrol et
		  if (obstacleDistance != lastObstacleDistance) {
		    lastObstacleDistance = obstacleDistance;
		    
		    // Engel algılandı (2 metre mesafede)
		    if (obstacleDistance < 2.0 && !obstacleDetected) {
		      obstacleDetected = true;
		      analogWrite(pwmPin, 0);  // PWM'i 0 yap
		      currentPwmValue = 0;
		      Serial.println("OBSTACLE_DETECTED");
		      Serial.println("PWM_SET:0");
		    }
		    // Engel temizlendi
		    else if (obstacleDistance >= 2.0 && obstacleDetected) {
		      obstacleDetected = false;
		      if (isMoving && !emergencyStop) {
			analogWrite(pwmPin, 128);  // PWM'i 50'ye geri döndür (128/255 ≈ 50%)
			currentPwmValue = 128;
			Serial.println("OBSTACLE_CLEARED");
			Serial.println("PWM_SET:128");
		      }
		    }
		  }
		}

		void processCommand(String cmd) {
		  if (cmd == "GO_STRAIGHT_20M") {
		    startStraightMovement(20.0);
		  }
		  else if (cmd.startsWith("GO_STRAIGHT:")) {
		    float distance = cmd.substring(12).toFloat();
		    startStraightMovement(distance);
		  }
		  else if (cmd == "STOP") {
		    emergencyStop = true;
		    stopMotors();
		    Serial.println("STOPPED");
		  }
		  else if (cmd == "RESUME") {
		    emergencyStop = false;
		    obstacleDetected = false;  // Engel durumunu sıfırla
		    if (isMoving) {
		      analogWrite(pwmPin, 128);  // PWM'i 50'ye geri döndür
		      currentPwmValue = 128;
		    }
		    Serial.println("RESUMED");
		  }
		  else if (cmd.startsWith("SET_SPEED:")) {
		    float speed = cmd.substring(10).toFloat();
		    setSpeed(speed);
		  }
		  else if (cmd.startsWith("SET_OBSTACLE:")) {
		    float distance = cmd.substring(13).toFloat();
		    obstacleDistance = distance;
		  }
		  else if (cmd.startsWith("SET_POSITION:")) {
		    // Format: SET_POSITION:x,y
		    int commaIndex = cmd.indexOf(',', 13);
		    if (commaIndex > 0) {
		      robotX = cmd.substring(13, commaIndex).toFloat();
		      robotY = cmd.substring(commaIndex + 1).toFloat();
		    }
		  }
		  else if (cmd.startsWith("SET_START_POS:")) {
		    // Format: SET_START_POS:x,y
		    int commaIndex = cmd.indexOf(',', 14);
		    if (commaIndex > 0) {
		      startX = cmd.substring(14, commaIndex).toFloat();
		      startY = cmd.substring(commaIndex + 1).toFloat();
		    }
		  }
		  else if (cmd.startsWith("SET_PWM:")) {
		    // Yeni PWM komutu - Python'dan gelen
		    int value = cmd.substring(8).toInt();
		    value = constrain(value, 0, 255);
		    analogWrite(pwmPin, value);
		    currentPwmValue = value;
		    Serial.print("PWM_SET:");
		    Serial.println(value);
		  }
		  else if (cmd == "GET_STATUS") {
		    sendStatus();
		  }
		  // Mevcut komutlar
		  else if (cmd == "ACTUATOR_EXTEND") {
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
		    // Eski PWM komutu - geriye uyumluluk için
		    int value = cmd.substring(4).toInt();
		    value = constrain(value, 0, 255);
		    analogWrite(pwmPin, value);
		    currentPwmValue = value;
		    Serial.print("PWM_SET:");
		    Serial.println(value);
		  }
		  else {
		    Serial.print("UNKNOWN_COMMAND: ");
		    Serial.println(cmd);
		  }
		}

		void startStraightMovement(float distance) {
		  targetDistance = distance;
		  isMoving = true;
		  emergencyStop = false;
		  obstacleDetected = false;  // Engel durumunu sıfırla
		  
		  // Motorları başlat
		  digitalWrite(F_Signal, HIGH);
		  digitalWrite(R_Signal, LOW);
		  analogWrite(pwmPin, 128); // %50 hız (128/255)
		  currentPwmValue = 128;
		  
		  Serial.println("MOVEMENT_STARTED");
		}

		void setSpeed(float speed) {
		  currentSpeed = speed;
		  int pwmValue = int(speed * 255);
		  pwmValue = constrain(pwmValue, 0, 255);
		  
		  // Engel yoksa PWM'i güncelle
		  if (!obstacleDetected) {
		    analogWrite(pwmPin, pwmValue);
		    currentPwmValue = pwmValue;
		  }
		}

		void stopMotors() {
		  digitalWrite(F_Signal, LOW);
		  digitalWrite(R_Signal, LOW);
		  analogWrite(pwmPin, 0);
		  currentPwmValue = 0;
		  //isMoving = false;
		}

		void sendStatus() {
		  Serial.print("STATUS:SW1:");
		  Serial.print(digitalRead(switch1));
		  Serial.print(",SW2:");
		  Serial.print(digitalRead(switch2));
		  Serial.print(",PWM:");
		  Serial.print(currentPwmValue);
		  Serial.print(",MOVING:");
		  Serial.print(isMoving ? "1" : "0");
		  Serial.print(",TARGET:");
		  Serial.print(targetDistance);
		  Serial.print(",OBSTACLE:");
		  Serial.print(obstacleDistance);
		  Serial.print(",OBSTACLE_DETECTED:");
		  Serial.print(obstacleDetected ? "1" : "0");
		  Serial.print(",POS_X:");
		  Serial.print(robotX);
		  Serial.print(",POS_Y:");
		  Serial.print(robotY);
		  Serial.print(",START_X:");
		  Serial.print(startX);
		  Serial.print(",START_Y:");
		  Serial.println(startY);
		} 
