#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "Kalman_filter.h"

class UltrasonicSensor {
private:
  KalmanFilter filter;
  float lastDistance;
  
  float readRawDistance();

public:
  UltrasonicSensor();
  void begin();
  float getDistance();
  float getFilteredDistance();
};

#endif