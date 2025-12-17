#include "kalman_filter.h"

KalmanFilter::KalmanFilter(float processNoise, float measurementNoise)
  : q(processNoise), r(measurementNoise), x(0), p(1), k(0) {}

float KalmanFilter::update(float measurement) {
  // Prediction
  p = p + q;
  
  // Update
  k = p / (p + r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;
  
  return x;
}

void KalmanFilter::reset() {
  x = 0;
  p = 1;
  k = 0;
}

float KalmanFilter::getValue() const {
  return x;
}

