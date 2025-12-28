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
  float getDistance();          // Returns filtered distance with averaging
  float getFilteredDistance();  // Returns last filtered value
  float getRawDistance();        // Returns single raw reading (for debugging)
};

#endif