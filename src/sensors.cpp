#include "sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

Sensors::Sensors() 
  : pitchFilter(ANGLE_FILTER_Q, ANGLE_FILTER_R),
    rollFilter(ANGLE_FILTER_Q, ANGLE_FILTER_R),
    pitch(0), roll(0), yaw(0),
    accelX(0), accelY(0), accelZ(0),
    gyroX(0), gyroY(0), gyroZ(0),
    distance(0) {}

void Sensors::begin() {
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize MPU6050
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("✓ MPU6050 connected");
  } else {
    Serial.println("✗ MPU6050 connection failed");
  }
  
  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("✓ Sensors initialized");
}

void Sensors::updateIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to physical units
  accelX = ax / 16384.0;
  accelY = ay / 16384.0;
  accelZ = az / 16384.0;
  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
  
  // Calculate orientation
  float rawPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float rawRoll = atan2(-accelX, accelZ) * 180.0 / PI;
  
  // Apply Kalman filtering
  pitch = pitchFilter.update(rawPitch);
  roll = rollFilter.update(rawRoll);
  
  // Simple yaw integration
  yaw += gyroZ * 0.02; // 20ms update rate
}

void Sensors::updateDistance() {
  distance = readUltrasonicDistance();
}

float Sensors::readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float dist = duration * 0.034 / 2;
  
  if (dist == 0 || dist > MAX_DISTANCE) {
    dist = MAX_DISTANCE;
  }
  
  return dist;
}

