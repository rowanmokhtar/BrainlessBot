#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "Kalman_Filter.h"
#include "MPU.h"
#include "Encoder.h"
#include "Motor_Control.h"
#include "Ultrasonic.h"
#include "ServoControl.h"
#include "Web_Interface.h"

// Global objects
MPU6050Sensor imu;
Encoder leftEncoder(ENCODER_LEFT_A, ENCODER_LEFT_B, WHEEL_DIAMETER, ENCODER_PPR);
Encoder rightEncoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B, WHEEL_DIAMETER, ENCODER_PPR);
MotorControl motors;
UltrasonicSensor ultrasonic;
ServoControl servoArm;
WebInterface webInterface;

// Timing variables
unsigned long lastPIDUpdate = 0;
unsigned long lastMPUUpdate = 0;
unsigned long lastDistanceCheck = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // setup Servo
  servoArm.begin();
  delay(500);
  
  // wifi setup
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(500);
  
  // motor setup
  motors.begin(&leftEncoder, &rightEncoder);
  delay(100);
  
  // encoder setup
  leftEncoder.begin();
  rightEncoder.begin();
  
  // imu setup
  
  if (!imu.begin()) {
    Serial.println("imu failed");
  } else {
    Serial.println("imu ready");
  }
  delay(100);
  
  // ultrasonic setup
  ultrasonic.begin();
  
  // web server setup

  webInterface.begin(&motors, &ultrasonic, &servoArm, &imu, &leftEncoder, &rightEncoder);
 
  // Initialize timing
  lastPIDUpdate = millis();
  lastMPUUpdate = millis();
  lastDistanceCheck = millis();
}

void loop() {
  unsigned long now = millis();
  
  //  Poll encoders as fast as possible for accurate readings
  leftEncoder.update();
  rightEncoder.update();
  
  // Handle web requests
  webInterface.handleClient();
  
  // Update MPU6050
  if (now - lastMPUUpdate >= MPU_UPDATE_INTERVAL) {
    imu.update();
    lastMPUUpdate = now;
  }
  
  // Check distance for auto-stop
  if (now - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    float distance = ultrasonic.getDistance();
    
    // Auto-stop with hysteresis
    if (distance < STOP_DISTANCE) {
      motors.setAutoStop(true);
    } else if (distance > RESUME_DISTANCE) {
      motors.setAutoStop(false);
    }
    
    lastDistanceCheck = now;
  }
  
  // Update motors with PID
  if (now - lastPIDUpdate >= PID_UPDATE_INTERVAL) {
    float dt = (now - lastPIDUpdate) / 1000.0;
    motors.update(dt);
    lastPIDUpdate = now;
  }
  
  // Small delay (encoder polling happens every loop iteration)
  delay(1);
}