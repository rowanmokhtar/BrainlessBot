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
  
  Serial.println("✓ Motor control initialized (H-Bridge with PID + Encoders)");
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
  integral += error * dt;
  
  // Anti-windup
  float maxIntegral = MAX_SPEED / MOTOR_KI;
  if (integral > maxIntegral) integral = maxIntegral;
  if (integral < -maxIntegral) integral = -maxIntegral;
  
  float derivative = (error - prevError) / dt;
  prevError = error;
  
  float output = MOTOR_KP * error + MOTOR_KI * integral + MOTOR_KD * derivative;
  
  // Clamp output
  if (output > MAX_SPEED) output = MAX_SPEED;
  if (output < -MAX_SPEED) output = -MAX_SPEED;
  
  return output;
}

void MotorControl::updateOdometry(float dt) {
  if (!leftEncoder || !rightEncoder) return;
  
  // Get wheel velocities
  float vLeft = leftEncoder->getVelocity();
  float vRight = rightEncoder->getVelocity();
  
  // Calculate robot linear and angular velocities
  float vRobot = (vLeft + vRight) / 2.0;
  float omega = (vRight - vLeft) / WHEEL_BASE;
  
  // Update robot pose
  robotTheta += omega * dt;
  robotX += vRobot * cos(robotTheta) * dt;
  robotY += vRobot * sin(robotTheta) * dt;
}

void MotorControl::update(float dt) {
  if (!leftEncoder || !rightEncoder) return;
  
  if (autoStopped) {
    setMotorPWM(0, 0);
    return;
  }
  
  // Get encoder feedback (actual velocities)
  float leftVelocity = leftEncoder->getVelocity();
  float rightVelocity = rightEncoder->getVelocity();
  
  // Apply Kalman filters
  float filteredLeft = leftSpeedFilter.update(leftVelocity);
  float filteredRight = rightSpeedFilter.update(rightVelocity);
  
  // Calculate PID outputs
  float leftPWM = calculatePID(targetLeftSpeed, filteredLeft, leftError, leftIntegral, leftPrevError, dt);
  float rightPWM = calculatePID(targetRightSpeed, filteredRight, rightError, rightIntegral, rightPrevError, dt);
  
  // Set motor PWM
  setMotorPWM((int)leftPWM, (int)rightPWM);
  
  // Update odometry
  updateOdometry(dt);
}

void MotorControl::stop() {
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
  setMotorPWM(0, 0);
  
  // Reset PID
  leftIntegral = 0;
  leftPrevError = 0;
  rightIntegral = 0;
  rightPrevError = 0;
  
  Serial.println("→ STOP");
}

void MotorControl::setTargetSpeed(float left, float right) {
  if (autoStopped && (left > 0 || right > 0)) {
    Serial.println("⚠️ Cannot move forward - obstacle detected!");
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
  Serial.println("→ FORWARD at " + String(speed, 2) + " m/s");
}

void MotorControl::moveBackward(float speed) {
  setTargetSpeed(-speed, -speed);
  Serial.println("→ BACKWARD at " + String(speed, 2) + " m/s");
}

void MotorControl::turnLeft(float speed) {
  setTargetSpeed(-speed, speed);
  Serial.println("→ LEFT at " + String(speed, 2) + " m/s");
}

void MotorControl::turnRight(float speed) {
  setTargetSpeed(speed, -speed);
  Serial.println("→ RIGHT at " + String(speed, 2) + " m/s");
}

void MotorControl::setAutoStop(bool stopped) {
  if (stopped && !autoStopped) {
    Serial.println("🛑 AUTO-STOP ACTIVATED");
  } else if (!stopped && autoStopped) {
    Serial.println("✅ AUTO-STOP RELEASED");
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
  
  Serial.println("📍 Odometry reset");
}