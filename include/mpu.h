#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Kalman_filter.h"

struct IMUData {
  float pitch;        // Pitch angle (degrees)
  float roll;         // Roll angle (degrees)
  float yaw;          // Yaw angle (degrees)
  float accelX;       // X acceleration (m/s²)
  float accelY;       // Y acceleration (m/s²)
  float accelZ;       // Z acceleration (m/s²)
  float gyroX;        // X rotation rate (°/s)
  float gyroY;        // Y rotation rate (°/s)
  float gyroZ;        // Z rotation rate (°/s)
  float temperature;  // Temperature (°C)
};

class MPU6050Sensor {
private:
  Adafruit_MPU6050 mpu;
  
  // Kalman filters for sensor fusion
  KalmanFilter pitchFilter;
  KalmanFilter rollFilter;
  
  // Complementary filter variables
  float compPitch;
  float compRoll;
  float compYaw;
  
  // Raw sensor data
  sensors_event_t accel, gyro, temp;
  
  // Timing
  unsigned long lastUpdateTime;
  
  // Helper functions
  void calculateAnglesFromAccel(float &pitch, float &roll);
  void applyComplementaryFilter(float dt);

public:
  MPU6050Sensor();
  bool begin();
  void update();
  void calibrate();
  
  IMUData getData() const;
  float getPitch() const;
  float getRoll() const;
  float getYaw() const;
};

#endif