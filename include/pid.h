#ifndef PID_H
#define PID_H

class PIDController {
private:
  float kp, ki, kd;
  float prevError;
  float integral;
  float maxOutput;
  float minOutput;

public:
  PIDController(float p, float i, float d);
  float compute(float setpoint, float measured, float dt);
  void reset();
  void setLimits(float minOut, float maxOut);
  void setGains(float p, float i, float d);
};

#endif
