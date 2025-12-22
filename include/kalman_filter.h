#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
private:
  float q;  // Process noise covariance
  float r;  // Measurement noise covariance
  float x;  // Estimated value
  float p;  // Estimation error covariance
  float k;  // Kalman gain

public:
  KalmanFilter(float processNoise, float measurementNoise);
  float update(float measurement);
  void reset();
  float getValue() const;
};

#endif