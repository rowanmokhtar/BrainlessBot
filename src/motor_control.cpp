#include "motor_control.h"
#include "config.h"
#include <Arduino.h>

MotorControl::MotorControl() 
  : leftPID(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD),
    rightPID(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD),
    leftFilter(SPEED_FILTER_Q, SPEED_FILTER_R),
    rightFilter(SPEED_FILTER_Q, SPEED_FILTER_R),
    targetSpeed(0), targetTurn(0) {}

void MotorControl::begin() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);

  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_ENA, PWM_CHANNEL_LEFT);
  ledcAttachPin(MOTOR_ENB, PWM_CHANNEL_RIGHT);

  Serial.println("✓ Motors initialized");
}

void MotorControl::setSpeed(int speed, int turn) {
  targetSpeed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  targetTurn = constrain(turn, -MAX_SPEED, MAX_SPEED);
}

void MotorControl::update(float dt) {
  int leftSpeed = targetSpeed + targetTurn;
  int rightSpeed = targetSpeed - targetTurn;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  float filteredLeft = leftFilter.update(leftSpeed);
  float filteredRight = rightFilter.update(rightSpeed);

  float leftOut = leftPID.compute(leftSpeed, filteredLeft, dt);
  float rightOut = rightPID.compute(rightSpeed, filteredRight, dt);

  setMotorSpeed((int)leftOut, (int)rightOut);
}

void MotorControl::setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) { digitalWrite(MOTOR_IN1,HIGH); digitalWrite(MOTOR_IN2,LOW);}
  else if (leftSpeed < 0) { digitalWrite(MOTOR_IN1,LOW); digitalWrite(MOTOR_IN2,HIGH);}
  else { digitalWrite(MOTOR_IN1,LOW); digitalWrite(MOTOR_IN2,LOW); }

  if (rightSpeed > 0) { digitalWrite(MOTOR_IN3,HIGH); digitalWrite(MOTOR_IN4,LOW);}
  else if (rightSpeed < 0) { digitalWrite(MOTOR_IN3,LOW); digitalWrite(MOTOR_IN4,HIGH);}
  else { digitalWrite(MOTOR_IN3,LOW); digitalWrite(MOTOR_IN4,LOW); }

  ledcWrite(PWM_CHANNEL_LEFT, abs(leftSpeed));
  ledcWrite(PWM_CHANNEL_RIGHT, abs(rightSpeed));
}

void MotorControl::stop() {
  setSpeed(0, 0);
  update(0.02);
}
