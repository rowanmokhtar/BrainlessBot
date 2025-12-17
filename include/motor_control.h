#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pid.h"
#include "kalman_filter.h"

class MotorControl {
private:
  PIDController leftPID;
  PIDController rightPID;
  KalmanFilter leftFilter;
  KalmanFilter rightFilter;
  
  int targetSpeed;
  int targetTurn;

public:
  MotorControl();
  void begin();
  void setSpeed(int speed, int turn);
  void update(float dt);
  void stop();
  
private:
  void setMotorSpeed(int leftSpeed, int rightSpeed);
};

#endif
