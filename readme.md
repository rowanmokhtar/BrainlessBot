# Brainless Bot

Brainless Bot is a simple humanoid robot that kicks balls. 
It’s not smart, but it tries its best! ⚽🤖

## Features
- WiFi web interface for remote control
- L298N motor driver with PID control
- MPU6050 IMU for orientation tracking
- HC-SR04 ultrasonic distance sensor
- Servo arm control
- Kalman filtering for smooth sensor data
- Real-time sensor dashboard

## Hardware Requirements
- ESP32 Development Board
- L298N Motor Driver
- MPU6050 IMU (I2C)
- HC-SR04 Ultrasonic Sensor
- Servo Motor (for arm)
- 2x DC Motors
- Power Supply (12V for motors)

## Wiring Diagram

### L298N Motor Driver
- ENA → GPIO 5(Left motor PWM)
- IN1 → GPIO 18 (Left motor direction 1)
- IN2 → GPIO 19 (Left motor direction 2)
- ENB → GPIO 4 (Right motor PWM)
- IN3 → GPIO 22 (Right motor direction 1)
- IN4 → GPIO 23 (Right motor direction 2)

### HC-SR04 Ultrasonic
- TRIG → GPIO 13
- ECHO → GPIO 12
- VCC → 5V
- GND → GND

### MPU6050 IMU
- SDA → GPIO 26
- SCL → GPIO 27
- VCC → 3.3V
- GND → GND

### Servo Motor
- Signal → GPIO 21
- VCC → 5V
- GND → GND

## Setup Instructions

1. Install PlatformIO in VS Code
2. Clone/download this project
3. Edit `include/config.h` with your WiFi credentials
4. Connect ESP32 via USB
5. Click "Upload" in PlatformIO
6. Open Serial Monitor to see IP address
7. Open the IP address in your browser

## Usage

### Web Interface Controls
- ▲ Forward
- ▼ Backward
- ◀ Left
- ▶ Right
- ■ Stop
- ⬆ Arm Up
- ⬇ Arm Down

### Real-time Data Display
- Distance measurement (cm)
- IMU orientation (pitch, roll, yaw)
- Arm position (degrees)

## Configuration

Edit `include/config.h` to change:
- WiFi credentials
- PID parameters
- Kalman filter settings
- Motor speed limits
- Pin assignments

## Troubleshooting

**WiFi won't connect:**
- Check SSID and password in config.h
- Ensure 2.4GHz WiFi (ESP32 doesn't support 5GHz)

**Motors not moving:**
- Check L298N power supply (7-12V)
- Verify all motor connections
- Check enable jumpers on L298N

**IMU not detected:**
- Verify I2C connections (SDA/SCL)
- Check MPU6050 power (3.3V)
- Try different I2C address if needed

**Sensors not updating:**
- Check Serial Monitor for errors
- Verify sensor connections
- Ensure proper power supply

