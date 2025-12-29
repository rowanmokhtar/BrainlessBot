#ifndef CONFIG_H
#define CONFIG_H

// WiFi Settings
#define AP_SSID        "brainless"
#define AP_PASSWORD    "12345678"

// Motor Pins 
#define MOTOR_LEFT_PWM   14  // ENA 
#define MOTOR_LEFT_IN1   18  // IN1 
#define MOTOR_LEFT_IN2   19  // IN2 
#define MOTOR_RIGHT_PWM  4   // ENB 
#define MOTOR_RIGHT_IN3  25  // IN3
#define MOTOR_RIGHT_IN4  23  // IN4 

// Encoder Pins - POLLING MODE 
#define ENCODER_LEFT_A   34  // Channel A
#define ENCODER_LEFT_B   35  // Channel B
#define ENCODER_RIGHT_A  32  // Channel A
#define ENCODER_RIGHT_B  33  // Channel B

// MPU6050 I2C Pins
#define MPU_SDA_PIN  26
#define MPU_SCL_PIN  27

// Servo Pin
#define SERVO_PIN 21

// Ultrasonic Pins
#define TRIG_PIN 13
#define ECHO_PIN 12

// PWM Settings
#define PWM_FREQ 1000
#define PWM_RES 8
#define PWM_CHANNEL_LEFT 4
#define PWM_CHANNEL_RIGHT 5

// Motor Speeds
#define MAX_SPEED 255
#define FORWARD_SPEED 200
#define TURN_SPEED 150

// Encoder Settings
#define ENCODER_PPR 200 // PPR 
#define WHEEL_DIAMETER 0.065
#define WHEEL_BASE 0.16

// PID Parameters
#define MOTOR_KP 3.0   // kp
#define MOTOR_KI 0.5   // ki
#define MOTOR_KD 0.1   // kd

// Kalman Filter Parameters
#define DISTANCE_Q 0.2
#define DISTANCE_R 3.0
#define SPEED_Q 0.01
#define SPEED_R 0.1
#define ANGLE_Q 0.001
#define ANGLE_R 0.03

// Complementary Filter
#define COMPLEMENTARY_ALPHA 0.98

// Servo Settings
#define SERVO_MIN 0
#define SERVO_MAX 180
#define SERVO_INITIAL 90
#define SERVO_STEP 90

// Safety Settings
#define STOP_DISTANCE 30.0
#define RESUME_DISTANCE 40.0
#define CHECK_INTERVAL 50

// Update Intervals
#define PID_UPDATE_INTERVAL 20
#define MPU_UPDATE_INTERVAL 20
#define ENCODER_UPDATE_INTERVAL 1      // Poll as fast as possible
#define DISTANCE_CHECK_INTERVAL 100
#define WEB_UPDATE_INTERVAL 200

// Motion Tracking
#define ENABLE_ODOMETRY true
#define ENABLE_MOTION_TRACKING true

#endif