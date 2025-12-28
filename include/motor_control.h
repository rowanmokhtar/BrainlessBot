#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Kalman_filter.h"
#include "encoder.h"

struct MotorState {
  float leftSpeed;      // Current left motor speed (m/s)
  float rightSpeed;     // Current right motor speed (m/s)
  float leftPWM;        // Left motor PWM value
  float rightPWM;       // Right motor PWM value
  float leftDistance;   // Distance traveled by left wheel (m)
  float rightDistance;  // Distance traveled by right wheel (m)
  float robotX;         // Robot X position (m)
  float robotY;         // Robot Y position (m)
  float robotTheta;     // Robot orientation (radians)
};

class MotorControl {
private:
  // Target speeds (m/s)
  float targetLeftSpeed;
  float targetRightSpeed;
  
  // PID variables for closed-loop control
  float leftError, leftIntegral, leftPrevError;
  float rightError, rightIntegral, rightPrevError;
  
  // Kalman filters for speed smoothing
  KalmanFilter leftSpeedFilter;
  KalmanFilter rightSpeedFilter;
  
  // Encoders for feedback
  Encoder* leftEncoder;
  Encoder* rightEncoder;
  
  // Auto-stop state
  bool autoStopped;
  
  // Odometry variables
  float robotX, robotY, robotTheta;
  
  // Helper methods
  void setMotorPWM(int left, int right);
  float calculatePID(float target, float current, float &error, float &integral, float &prevError, float dt);
  void updateOdometry(float dt);

public:
  MotorControl();
  void begin(Encoder* leftEnc, Encoder* rightEnc);
  void update(float dt);
  void stop();
  
  // Movement commands (speed in m/s)
  void setTargetSpeed(float left, float right);
  void moveForward(float speed);
  void moveBackward(float speed);
  void turnLeft(float speed);
  void turnRight(float speed);
  
  // Auto-stop control
  void setAutoStop(bool stopped);
  bool isAutoStopped() const;
  
  // Get motor state
  MotorState getState() const;
  
  // Reset odometry
  void resetOdometry();
   void testMotors();
};

#endif