#ifndef CONFIG_H
#define CONFIG_H


// ===== Access Point Settings =====
#define AP_SSID        "brainless"
#define AP_PASSWORD    "12345678"


// ===== L298N Motor Driver Pins =====
#define MOTOR_ENA 5
#define MOTOR_IN1 18
#define MOTOR_IN2 19
#define MOTOR_ENB 4
#define MOTOR_IN3 22
#define MOTOR_IN4 23

// ===== Servo Pin =====
#define SERVO_PIN 21

// ===== Ultrasonic Sensor Pins =====
#define TRIG_PIN 13
#define ECHO_PIN 12

// ===== MPU6050 I2C Pins =====
#define SDA_PIN 26
#define SCL_PIN 27

// ===== PWM Settings =====
#define PWM_FREQ 5000
#define PWM_RES 8
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1

// ===== Motor Speed Settings =====
#define MAX_SPEED 255
#define DEFAULT_SPEED 200
#define DEFAULT_TURN_SPEED 150

// ===== PID Parameters =====
#define MOTOR_PID_KP 2.0
#define MOTOR_PID_KI 0.5
#define MOTOR_PID_KD 0.1

#define BALANCE_PID_KP 3.0
#define BALANCE_PID_KI 0.8
#define BALANCE_PID_KD 0.2

// ===== Kalman Filter Parameters =====
#define SPEED_FILTER_Q 0.1
#define SPEED_FILTER_R 0.5

#define ANGLE_FILTER_Q 0.01
#define ANGLE_FILTER_R 0.1

// ===== Servo Settings =====
#define SERVO_MIN 0
#define SERVO_MAX 180
#define SERVO_INITIAL 90
#define SERVO_STEP 10

// ===== Sensor Update Rates =====
#define SENSOR_UPDATE_RATE 20  // 50Hz (20ms)
#define MOTOR_UPDATE_RATE 20   // 50Hz (20ms)

// ===== Distance Sensor Settings =====
#define MAX_DISTANCE 400  // Maximum measurable distance in cm
#define WARNING_DISTANCE 15  // Show warning below this distance

// ===== Auto-Stop Safety Settings ===== 
#define ENABLE_AUTO_STOP true        // Set to false to disable auto-stop
#define AUTO_STOP_DISTANCE 10.0      // Distance in cm to trigger auto-stop
#define AUTO_STOP_CHECK_INTERVAL 50  // Check every 50ms

#endif