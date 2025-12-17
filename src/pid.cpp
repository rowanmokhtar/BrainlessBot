#include "pid.h"
#include "config.h"

PIDController::PIDController(float p, float i, float d) 
  : kp(p), ki(i), kd(d), prevError(0), integral(0), 
    maxOutput(MAX_SPEED), minOutput(-MAX_SPEED) {}

float PIDController::compute(float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  
  // Anti-windup
  if (integral > maxOutput) integral = maxOutput;
  if (integral < minOutput) integral = minOutput;
  
  float derivative = (error - prevError) / dt;
  prevError = error;
  
  float output = kp * error + ki * integral + kd * derivative;
  
  // Clamp output
  if (output > maxOutput) output = maxOutput;
  if (output < minOutput) output = minOutput;
  
  return output;
}

void PIDController::reset() {
  prevError = 0;
  integral = 0;
}

void PIDController::setLimits(float minOut, float maxOut) {
  minOutput = minOut;
  maxOutput = maxOut;
}

void PIDController::setGains(float p, float i, float d) {
  kp = p;
  ki = i;
  kd = d;
}
