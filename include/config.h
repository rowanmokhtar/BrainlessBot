#ifndef CONFIG_H
#define CONFIG_H

// WiFi Settings
#define AP_SSID        "brainless"
#define AP_PASSWORD    "12345678"

// Motor Pins 
#define MOTOR_LEFT_PWM   5    // ENA 
#define MOTOR_LEFT_IN1   18   // IN1 
#define MOTOR_LEFT_IN2   19   // IN2 
#define MOTOR_RIGHT_PWM  4    // ENB 
#define MOTOR_RIGHT_IN3  22   // IN3
#define MOTOR_RIGHT_IN4  23   // IN4 

// Encoder Pins 
#define ENCODER_LEFT_A   34   //  R channel A
#define ENCODER_LEFT_B   35   //  L channel B
#define ENCODER_RIGHT_A  32   // R channel A
#define ENCODER_RIGHT_B  33   // L channel B

// MPU6050 I2C Pins
#define MPU_SDA_PIN  26
#define MPU_SCL_PIN  27

// Servo Pin
#define SERVO_PIN 21

// Ultrasonic Pins
#define TRIG_PIN 13
#define ECHO_PIN 12

// PWM Settings
#define PWM_FREQ 20000        // 20kHz
#define PWM_RES 8             // 8-bit resolution (0-255)
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1

// Motor Speeds
#define MAX_SPEED 255
#define FORWARD_SPEED 200   // Target speed for forward movement
#define TURN_SPEED 150       // Target speed for turning

// Encoder Settings
#define ENCODER_PPR 280      // Pulses per revolution
#define WHEEL_DIAMETER 0.065  //65mm
#define WHEEL_BASE 0.16     // Distance between wheels in meters (160mm)

// PID Parameters for Motor Speed Control (Closed Loop)
#define MOTOR_KP 2.5
#define MOTOR_KI 0.8
#define MOTOR_KD 0.15

// PID Parameters for Balance Control 
#define BALANCE_KP 15.0
#define BALANCE_KI 0.5
#define BALANCE_KD 1.2

// Kalman Filter Parameters
#define DISTANCE_Q 0.1        // Process noise for distance
#define DISTANCE_R 5.0        // Measurement noise for distance
#define SPEED_Q 0.01          // Process noise for motor speed
#define SPEED_R 0.1           // Measurement noise for motor speed
#define ANGLE_Q 0.001         // Process noise for angle (MPU)
#define ANGLE_R 0.03          // Measurement noise for angle (MPU)

// Complementary Filter for MPU
#define COMPLEMENTARY_ALPHA 0.98  // 98% gyro, 2% accelerometer

// Servo Settings
#define SERVO_MIN 0
#define SERVO_MAX 180
#define SERVO_INITIAL 90
#define SERVO_STEP 60

// Safety Settings
#define STOP_DISTANCE 15.0
#define RESUME_DISTANCE 30.0
#define CHECK_INTERVAL 50

// Update Intervals (ms)
#define PID_UPDATE_INTERVAL 10     // 100Hz for motor PID
#define MPU_UPDATE_INTERVAL 10     // 100Hz for IMU
#define ENCODER_UPDATE_INTERVAL 20 // 50Hz for encoder readings
#define DISTANCE_CHECK_INTERVAL 50 // 20Hz for ultrasonic
#define WEB_UPDATE_INTERVAL 100    // 10Hz for web updates

// Motion Tracking
#define ENABLE_ODOMETRY true
#define ENABLE_MOTION_TRACKING true

#endif