#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
private:
  float q;  // process noise covariance
  float r;  // measurement noise covariance
  float x;  // estimated value
  float p;  // estimation error covariance
  float k;  // kalman gain

public:
  KalmanFilter(float processNoise, float measurementNoise);
  float update(float measurement);
  void reset();
  float getValue() const;
};

#endif