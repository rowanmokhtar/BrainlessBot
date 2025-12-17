#ifndef SENSORS_H
#define SENSORS_H

#include <MPU6050.h>
#include "kalman_filter.h"

class Sensors {
private:
  MPU6050 mpu;
  KalmanFilter pitchFilter;
  KalmanFilter rollFilter;
  
  float pitch, roll, yaw;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float distance;

public:
  Sensors();
  void begin();
  void updateIMU();
  void updateDistance();
  
  float getPitch() const { return pitch; }
  float getRoll() const { return roll; }
  float getYaw() const { return yaw; }
  float getDistance() const { return distance; }
  
private:
  float readUltrasonicDistance();
};

#endif
