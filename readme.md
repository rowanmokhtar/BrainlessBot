# 🤖 BrainlessBot

**ESP32-based mobile robot firmware focused on control, stability, and real-time WiFi operation.**

BrainlessBot is a modular robotics firmware built using ESP32 and PlatformIO.  
It combines **IMU-based orientation tracking**, **Kalman filtering**, and **PID motor control** to achieve stable and responsive robot movement, all controlled wirelessly over WiFi.

**What’s inside**

📶 WiFi remote control

🧭 IMU orientation (roll, pitch, yaw)

📊 Kalman filter to calm noisy sensors

🤖 PID motor control

⚙️ Modular C++ code (PlatformIO)

🛠️ Built with

ESP32 &
C++ &
PlatformIO &
Arduino framework &
WiFi

🚀 **Run it**
```bash
git clone https://github.com/rowanmokhtar/BrainlessBot.git
cd BrainlessBot
pio run --target upload

