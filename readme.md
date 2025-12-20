# 🤖 BrainlessBot

**ESP32-based mobile robot firmware focused on control, stability, and real-time WiFi operation.**

BrainlessBot is a modular robotics firmware built using ESP32 and PlatformIO.  
It combines **IMU-based orientation tracking**, **Kalman filtering**, and **PID motor control** to achieve stable and responsive robot movement, all controlled wirelessly over WiFi.

## ✨ Key Features
- 📶 WiFi-based remote control
- 🧭 Real-time IMU orientation (roll, pitch, yaw)
- 📊 Kalman filter for noise reduction & sensor fusion
- 🤖 PID control for precise motor movement
- ⚙️ Modular C++ firmware (PlatformIO + Arduino)

## 🛠️ Tech Stack
- **ESP32**
- **C++**
- **PlatformIO**
- **Arduino Framework**
- **WiFi**
- **Kalman Filter & PID Control**

## 🚀 Getting Started
```bash
git clone https://github.com/rowanmokhtar/BrainlessBot.git
cd BrainlessBot
pio run --target upload
