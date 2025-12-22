#include "mpu.h"
#include "config.h"
#include <Arduino.h>

MPU6050Sensor::MPU6050Sensor() 
  : pitchFilter(ANGLE_Q, ANGLE_R),
    rollFilter(ANGLE_Q, ANGLE_R),
    compPitch(0), compRoll(0), compYaw(0),
    lastUpdateTime(0) {}

bool MPU6050Sensor::begin() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  
  if (!mpu.begin()) {
    Serial.println("✗ MPU6050 not found!");
    return false;
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 initialized");
  
  // Initial calibration
  delay(100);
  calibrate();
  
  lastUpdateTime = millis();
  return true;
}

void MPU6050Sensor::calibrate() {
  Serial.println("Calibrating MPU6050... Keep robot still!");
  
  float pitchSum = 0, rollSum = 0;
  const int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&accel, &gyro, &temp);
    
    float p, r;
    calculateAnglesFromAccel(p, r);
    pitchSum += p;
    rollSum += r;
    
    delay(10);
  }
  
  compPitch = pitchSum / samples;
  compRoll = rollSum / samples;
  
  Serial.println("MPU6050 calibrated");
  Serial.println("  Initial Pitch: " + String(compPitch, 2) + "°");
  Serial.println("  Initial Roll: " + String(compRoll, 2) + "°");
}

void MPU6050Sensor::calculateAnglesFromAccel(float &pitch, float &roll) {
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  
  // Calculate pitch and roll from accelerometer
  pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  roll = atan2(-ax, az) * 180.0 / PI;
}

void MPU6050Sensor::applyComplementaryFilter(float dt) {
  // Get accelerometer angles
  float accelPitch, accelRoll;
  calculateAnglesFromAccel(accelPitch, accelRoll);
  
  // Get gyroscope rates
  float gyroPitch = gyro.gyro.x * 180.0 / PI;
  float gyroRoll = gyro.gyro.y * 180.0 / PI;
  float gyroYaw = gyro.gyro.z * 180.0 / PI;
  
  // Complementary filter: 98% gyro, 2% accelerometer
  compPitch = COMPLEMENTARY_ALPHA * (compPitch + gyroPitch * dt) + 
              (1.0 - COMPLEMENTARY_ALPHA) * accelPitch;
              
  compRoll = COMPLEMENTARY_ALPHA * (compRoll + gyroRoll * dt) + 
             (1.0 - COMPLEMENTARY_ALPHA) * accelRoll;
  
  // Integrate yaw (no accelerometer correction for yaw)
  compYaw += gyroYaw * dt;
  
  // Apply Kalman filtering for additional smoothing
  compPitch = pitchFilter.update(compPitch);
  compRoll = rollFilter.update(compRoll);
}

void MPU6050Sensor::update() {
  // Get new sensor events
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Calculate time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = currentTime;
  
  // Apply sensor fusion
  applyComplementaryFilter(dt);
}

IMUData MPU6050Sensor::getData() const {
  IMUData data;
  data.pitch = compPitch;
  data.roll = compRoll;
  data.yaw = compYaw;
  data.accelX = accel.acceleration.x;
  data.accelY = accel.acceleration.y;
  data.accelZ = accel.acceleration.z;
  data.gyroX = gyro.gyro.x * 180.0 / PI;
  data.gyroY = gyro.gyro.y * 180.0 / PI;
  data.gyroZ = gyro.gyro.z * 180.0 / PI;
  data.temperature = temp.temperature;
  return data;
}

float MPU6050Sensor::getPitch() const { return compPitch; }
float MPU6050Sensor::getRoll() const { return compRoll; }
float MPU6050Sensor::getYaw() const { return compYaw; }
