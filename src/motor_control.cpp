#include "Motor_Control.h"
#include "config.h"
#include <Arduino.h>

MotorControl::MotorControl() 
  : targetLeftSpeed(0), targetRightSpeed(0),
    leftError(0), leftIntegral(0), leftPrevError(0),
    rightError(0), rightIntegral(0), rightPrevError(0),
    leftSpeedFilter(SPEED_Q, SPEED_R),
    rightSpeedFilter(SPEED_Q, SPEED_R),
    leftEncoder(nullptr), rightEncoder(nullptr),
    autoStopped(false),
    robotX(0), robotY(0), robotTheta(0) {}

void MotorControl::begin(Encoder* leftEnc, Encoder* rightEnc) {
  leftEncoder = leftEnc;
  rightEncoder = rightEnc;
  
  // Setup motor pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  
  // Setup PWM channels
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_LEFT_PWM, PWM_CHANNEL_LEFT);
  ledcAttachPin(MOTOR_RIGHT_PWM, PWM_CHANNEL_RIGHT);
  
  Serial.println("✓ Motor control initialized");
  
  // Test motors
  testMotors();
}

void MotorControl::testMotors() {
  Serial.println("Testing motors...");
  
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  ledcWrite(PWM_CHANNEL_LEFT, 100);
  delay(300);
  ledcWrite(PWM_CHANNEL_LEFT, 0);
  
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  ledcWrite(PWM_CHANNEL_RIGHT, 100);
  delay(300);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
  
  Serial.println("✓ Motors test complete");
}

void MotorControl::setMotorPWM(int left, int right) {
  // Left motor direction
  if (left > 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else if (left < 0) {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  }
  
  // Right motor direction
  if (right > 0) {
    digitalWrite(MOTOR_RIGHT_IN3, HIGH);
    digitalWrite(MOTOR_RIGHT_IN4, LOW);
  } else if (right < 0) {
    digitalWrite(MOTOR_RIGHT_IN3, LOW);
    digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  } else {
    digitalWrite(MOTOR_RIGHT_IN3, LOW);
    digitalWrite(MOTOR_RIGHT_IN4, LOW);
  }
  
  // Set PWM speeds
  ledcWrite(PWM_CHANNEL_LEFT, abs(left));
  ledcWrite(PWM_CHANNEL_RIGHT, abs(right));
}

float MotorControl::calculatePID(float target, float current, float &error, float &integral, float &prevError, float dt) {
  error = target - current;
  
  if (abs(error) < 0.2) {
    integral += error * dt;
  } else {
    integral = 0;
  }
  
  float maxIntegral = 50.0;
  integral = constrain(integral, -maxIntegral, maxIntegral);
  
  float derivative = (error - prevError) / dt;
  prevError = error;
  
  float output = MOTOR_KP * error + MOTOR_KI * integral + MOTOR_KD * derivative;
  
  // Feedforward term for faster response
  output += target * 300.0;  // Increased from 200.0
  
  output = constrain(output, -MAX_SPEED, MAX_SPEED);
  
  // Reduced deadband compensation for faster start
  if (abs(output) > 0 && abs(output) < 30) {  // Reduced from 50 to 30
    output = (output > 0) ? 30 : -30;
  }
  
  return output;
}

void MotorControl::updateOdometry(float dt) {
  if (!leftEncoder || !rightEncoder) return;
  
  float vLeft = leftEncoder->getVelocity();
  float vRight = rightEncoder->getVelocity();
  
  float vRobot = (vLeft + vRight) / 2.0;
  float omega = (vRight - vLeft) / WHEEL_BASE;
  
  robotTheta += omega * dt;
  robotX += vRobot * cos(robotTheta) * dt;
  robotY += vRobot * sin(robotTheta) * dt;
}

void MotorControl::update(float dt) {
  if (autoStopped) {
    setMotorPWM(0, 0);
    return;
  }
  
  if (targetLeftSpeed == 0 && targetRightSpeed == 0) {
    setMotorPWM(0, 0);
    return;
  }
  
  if (!leftEncoder || !rightEncoder) {
    // Open-loop control
    int leftPWM = (int)(targetLeftSpeed * 600);
    int rightPWM = (int)(targetRightSpeed * 600);
    setMotorPWM(leftPWM, rightPWM);
    return;
  }
  
  // Closed-loop with encoders
  float leftVelocity = leftEncoder->getVelocity();
  float rightVelocity = rightEncoder->getVelocity();
  
  float filteredLeft = leftSpeedFilter.update(leftVelocity);
  float filteredRight = rightSpeedFilter.update(rightVelocity);
  
  float leftPWM = calculatePID(targetLeftSpeed, filteredLeft, leftError, leftIntegral, leftPrevError, dt);
  float rightPWM = calculatePID(targetRightSpeed, filteredRight, rightError, rightIntegral, rightPrevError, dt);
  
  setMotorPWM((int)leftPWM, (int)rightPWM);
  updateOdometry(dt);
}

void MotorControl::stop() {
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
  setMotorPWM(0, 0);
  
  leftIntegral = 0;
  leftPrevError = 0;
  rightIntegral = 0;
  rightPrevError = 0;
  
  Serial.println("STOP");
}

void MotorControl::setTargetSpeed(float left, float right) {
  if (autoStopped && (left > 0 || right > 0)) {
    Serial.println("Cannot move forward - obstacle!");
    return;
  }
  
  targetLeftSpeed = left;
  targetRightSpeed = right;
  
  if (left < 0 || right < 0) {
    autoStopped = false;
  }
}

void MotorControl::moveForward(float speed) {
  setTargetSpeed(speed, speed);
  Serial.println("FORWARD at " + String(speed, 2) + " m/s");
}

void MotorControl::moveBackward(float speed) {
  setTargetSpeed(-speed, -speed);
  Serial.println("BACKWARD at " + String(speed, 2) + " m/s");
}

void MotorControl::turnLeft(float speed) {
  // Left wheel backward, right wheel forward (pivot turn)
  setTargetSpeed(-speed, speed);
  Serial.println("LEFT TURN at " + String(speed, 2) + " m/s");
}

void MotorControl::turnRight(float speed) {
  // Left wheel forward, right wheel backward (pivot turn)
  setTargetSpeed(speed, -speed);
  Serial.println("↷ RIGHT TURN at " + String(speed, 2) + " m/s");
}

void MotorControl::setAutoStop(bool stopped) {
  if (stopped && !autoStopped) {
    Serial.println("AUTO-STOP ACTIVATED");
    stop();
  } else if (!stopped && autoStopped) {
    Serial.println("AUTO-STOP RELEASED");
  }
  autoStopped = stopped;
}

bool MotorControl::isAutoStopped() const {
  return autoStopped;
}

MotorState MotorControl::getState() const {
  MotorState state;
  
  if (leftEncoder && rightEncoder) {
    state.leftSpeed = leftEncoder->getVelocity();
    state.rightSpeed = rightEncoder->getVelocity();
    state.leftDistance = leftEncoder->getDistance();
    state.rightDistance = rightEncoder->getDistance();
  } else {
    state.leftSpeed = 0;
    state.rightSpeed = 0;
    state.leftDistance = 0;
    state.rightDistance = 0;
  }
  
  state.leftPWM = targetLeftSpeed;
  state.rightPWM = targetRightSpeed;
  state.robotX = robotX;
  state.robotY = robotY;
  state.robotTheta = robotTheta;
  
  return state;
}

void MotorControl::resetOdometry() {
  robotX = 0;
  robotY = 0;
  robotTheta = 0;
  
  if (leftEncoder) leftEncoder->reset();
  if (rightEncoder) rightEncoder->reset();
  
}