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
 bool isAutoStopped; 

public:
  MotorControl();
  void begin();
  void setSpeed(int speed, int turn);
  void update(float dt);
  void stop();
  void autoStop();     
  void resume();       
  bool isAutoStopActive() const { return isAutoStopped; }
private:
  void setMotorSpeed(int leftSpeed, int rightSpeed);
};

#endif
