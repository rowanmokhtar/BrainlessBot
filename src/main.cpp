#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "Kalman_Filter.h"
#include "MPU.h"
#include "Encoder.h"
#include "Motor_Control.h"
#include "Ultrasonic.h"
#include "ServoControl.h"
#include "Web_Interface.h"

// Global objects
MPU6050Sensor imu;
Encoder leftEncoder(ENCODER_LEFT_A, ENCODER_LEFT_B, WHEEL_DIAMETER, ENCODER_PPR);
Encoder rightEncoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B, WHEEL_DIAMETER, ENCODER_PPR);
MotorControl motors;
UltrasonicSensor ultrasonic;
ServoControl servoArm;
WebInterface webInterface;

// Timing variables
unsigned long lastPIDUpdate = 0;
unsigned long lastMPUUpdate = 0;
unsigned long lastEncoderUpdate = 0;
unsigned long lastDistanceCheck = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║   🤖 ADVANCED ROBOT SYSTEM 🤖  ║");
  Serial.println("╚════════════════════════════════╝");
  
  // Setup WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.println("\n✓ WiFi AP Started");
  Serial.print("📡 SSID: ");
  Serial.println(AP_SSID);
  Serial.print("🌐 IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  // Initialize IMU (MPU6050)
  Serial.println("\n⚙️ Initializing sensors...");
  if (!imu.begin()) {
    Serial.println("✗ Failed to initialize MPU6050!");
    Serial.println("⚠️ Check I2C connections (SDA=26, SCL=27)");
  }
  
  // Initialize encoders with interrupts
  Encoder::leftEncoder = &leftEncoder;
  Encoder::rightEncoder = &rightEncoder;
  leftEncoder.begin();
  rightEncoder.begin();
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), Encoder::isrA_Left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), Encoder::isrA_Right, CHANGE);
  Serial.println("✓ Encoders initialized with interrupts");
  
  // Initialize other components
  motors.begin(&leftEncoder, &rightEncoder);
  ultrasonic.begin();
  servoArm.begin();
  webInterface.begin(&motors, &ultrasonic, &servoArm, &imu, &leftEncoder, &rightEncoder);
  
  // Print system features
  Serial.println("\n🎯 Advanced Features Enabled:");
  Serial.println("  ✓ MPU6050 Sensor Fusion (Complementary + Kalman)");
  Serial.println("  ✓ Encoder Feedback (Quadrature)");
  Serial.println("  ✓ PID Closed-Loop Motor Control");
  Serial.println("  ✓ H-Bridge Motor Driving (L298N)");
  Serial.println("  ✓ Motion Tracking & Odometry");
  Serial.println("  ✓ Kalman Filters (Distance + Speed + Angles)");
  Serial.println("  ✓ Auto-Stop Safety System");
  
  // Print configuration
  Serial.println("\n⚙️ Configuration:");
  Serial.println("  PID: Kp=" + String(MOTOR_KP) + ", Ki=" + String(MOTOR_KI) + ", Kd=" + String(MOTOR_KD));
  Serial.println("  Distance Filter: Q=" + String(DISTANCE_Q) + ", R=" + String(DISTANCE_R));
  Serial.println("  Speed Filter: Q=" + String(SPEED_Q) + ", R=" + String(SPEED_R));
  Serial.println("  Angle Filter: Q=" + String(ANGLE_Q) + ", R=" + String(ANGLE_R));
  
  Serial.println("\n🚀 System Ready!");
  Serial.println("⚠️ Auto-stop at " + String(STOP_DISTANCE) + "cm ENABLED");
  Serial.println("\n📱 Connect to WiFi and open: http://" + WiFi.softAPIP().toString());
  Serial.println("════════════════════════════════════\n");
  
  // Initialize timing
  lastPIDUpdate = millis();
  lastMPUUpdate = millis();
  lastEncoderUpdate = millis();
  lastDistanceCheck = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Handle web requests (highest priority)
  webInterface.handleClient();
  
  // Update MPU6050 (100Hz for smooth sensor fusion)
  if (now - lastMPUUpdate >= MPU_UPDATE_INTERVAL) {
    imu.update();
    lastMPUUpdate = now;
  }
  
  // Update encoders (50Hz)
  if (now - lastEncoderUpdate >= ENCODER_UPDATE_INTERVAL) {
    leftEncoder.update();
    rightEncoder.update();
    lastEncoderUpdate = now;
  }
  
  // Check distance for auto-stop safety (20Hz)
  if (now - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    float distance = ultrasonic.getDistance();
    
    // Auto-stop logic with hysteresis
    if (distance < STOP_DISTANCE) {
      motors.setAutoStop(true);
    } else if (distance > RESUME_DISTANCE) {
      motors.setAutoStop(false);
    }
    
    lastDistanceCheck = now;
  }
  
  // Update motors with PID control (100Hz for responsive control)
  if (now - lastPIDUpdate >= PID_UPDATE_INTERVAL) {
    float dt = (now - lastPIDUpdate) / 1000.0;
    motors.update(dt);
    lastPIDUpdate = now;
  }
  
  // Small delay to prevent watchdog timer issues
  delay(1);
}