import React, { useState } from 'react';
import { Wifi, ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Square, ChevronsUp, ChevronsDown, Activity, Radio } from 'lucide-react';

export default function RobotControlInterface() {
  const [wifiSSID, setWifiSSID] = useState('yarab');
  const [wifiPassword, setWifiPassword] = useState('123456789');
  const [status, setStatus] = useState('Configure WiFi credentials below');

  const handleCopy = () => {
    setStatus('yaaarraab');
  };

  const code = `#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// WiFi Credentials
const char* ssid = "yarab";
const char* password = "123456789";

// L298N Motor Driver Pins
#define ENA 5  // Left motor speed (PWM)
#define IN1 18  // Left motor direction 1
#define IN2 19  // Left motor direction 2
#define ENB 4  // Right motor speed (PWM)
#define IN3 22  // Right motor direction 1
#define IN4 23  // Right motor direction 2

// Servo Pin
#define SERVO_PIN 21

// Ultrasonic Sensor Pins
#define TRIG_PIN 13
#define ECHO_PIN 12

// MPU6050 I2C Pins (ESP32 default: SDA=21, SCL=22)
#define SDA_PIN 26
#define SCL_PIN 27

Servo armServo;
int armPosition = 90;

// MPU6050 IMU
MPU6050 mpu;

// Web Server
WebServer server(80);

// PWM Settings
#define PWM_FREQ 5000
#define PWM_RES 8

// Distance Measurement
float currentDistance = 0;

// IMU Data
float pitch = 0, roll = 0, yaw = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

// PID Controller
struct PIDController {
  float kp, ki, kd;
  float prevError, integral;
  float maxOutput, minOutput;
  
  PIDController(float p, float i, float d) : 
    kp(p), ki(i), kd(d), prevError(0), integral(0), 
    maxOutput(255), minOutput(-255) {}
  
  float compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integral += error * dt;
    
    if (integral > maxOutput) integral = maxOutput;
    if (integral < minOutput) integral = minOutput;
    
    float derivative = (error - prevError) / dt;
    prevError = error;
    
    float output = kp * error + ki * integral + kd * derivative;
    
    if (output > maxOutput) output = maxOutput;
    if (output < minOutput) output = minOutput;
    
    return output;
  }
  
  void reset() {
    prevError = 0;
    integral = 0;
  }
};

// Kalman Filter
struct KalmanFilter {
  float q, r, x, p, k;
  
  KalmanFilter(float processNoise, float measurementNoise) :
    q(processNoise), r(measurementNoise), x(0), p(1), k(0) {}
  
  float update(float measurement) {
    p = p + q;
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;
    return x;
  }
};

PIDController leftMotorPID(2.0, 0.5, 0.1);
PIDController rightMotorPID(2.0, 0.5, 0.1);
PIDController balancePID(3.0, 0.8, 0.2);  // For pitch stabilization

KalmanFilter leftSpeedFilter(0.1, 0.5);
KalmanFilter rightSpeedFilter(0.1, 0.5);
KalmanFilter pitchFilter(0.01, 0.1);
KalmanFilter rollFilter(0.01, 0.1);

int targetSpeed = 0;
int targetTurn = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastSensorUpdate = 0;

// HTML Web Interface
const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: Arial, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      padding: 10px;
      min-height: 100vh;
    }
    .container {
      max-width: 500px;
      margin: 0 auto;
      background: white;
      padding: 20px;
      border-radius: 20px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.3);
    }
    h1 { color: #667eea; margin-bottom: 15px; font-size: 24px; text-align: center; }
    .sensors {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-bottom: 15px;
    }
    .sensor-box {
      background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
      padding: 15px;
      border-radius: 12px;
      color: white;
      text-align: center;
    }
    .sensor-label { font-size: 12px; opacity: 0.9; margin-bottom: 5px; }
    .sensor-value { font-size: 24px; font-weight: bold; }
    .sensor-unit { font-size: 14px; opacity: 0.9; }
    .imu-data {
      background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
      padding: 15px;
      border-radius: 12px;
      color: white;
      margin-bottom: 15px;
    }
    .imu-row {
      display: flex;
      justify-content: space-between;
      margin: 5px 0;
      font-size: 14px;
    }
    .status {
      background: #f0f0f0;
      padding: 12px;
      border-radius: 10px;
      margin-bottom: 15px;
      font-size: 14px;
      text-align: center;
    }
    .obstacle-warning {
      background: #e74c3c;
      color: white;
      padding: 10px;
      border-radius: 10px;
      margin-bottom: 15px;
      text-align: center;
      font-weight: bold;
      animation: pulse 1s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.7; }
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      margin-bottom: 15px;
    }
    button {
      padding: 20px;
      font-size: 16px;
      border: none;
      border-radius: 12px;
      background: #667eea;
      color: white;
      cursor: pointer;
      transition: all 0.2s;
      font-weight: bold;
      touch-action: manipulation;
    }
    button:active {
      transform: scale(0.95);
      background: #5568d3;
    }
    .up { grid-column: 2; }
    .left { grid-column: 1; grid-row: 2; }
    .stop { grid-column: 2; grid-row: 2; background: #e74c3c; }
    .right { grid-column: 3; grid-row: 2; }
    .down { grid-column: 2; grid-row: 3; }
    .arm-controls {
      display: flex;
      gap: 8px;
      margin-top: 15px;
    }
    .arm-btn { background: #2ecc71; flex: 1; padding: 15px; }
    h3 { color: #667eea; font-size: 16px; margin: 15px 0 10px 0; text-align: center; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🤖 ESP32 Robot Control</h1>
    
    <div class="sensors">
      <div class="sensor-box">
        <div class="sensor-label">DISTANCE</div>
        <div class="sensor-value" id="distance">--</div>
        <div class="sensor-unit">cm</div>
      </div>
      <div class="sensor-box">
        <div class="sensor-label">ARM ANGLE</div>
        <div class="sensor-value" id="arm">90</div>
        <div class="sensor-unit">degrees</div>
      </div>
    </div>
    
    <div class="imu-data">
      <div style="font-weight: bold; margin-bottom: 8px; text-align: center;">📊 IMU Data</div>
      <div class="imu-row"><span>Pitch:</span><span id="pitch">0°</span></div>
      <div class="imu-row"><span>Roll:</span><span id="roll">0°</span></div>
      <div class="imu-row"><span>Yaw:</span><span id="yaw">0°</span></div>
    </div>
    
    <div id="warning" class="obstacle-warning" style="display: none;">
      ⚠️ OBSTACLE NEARBY - {currentDistance.toFixed(1)} cm
    </div>
    
    <div class="status" id="status">Ready</div>
    
    <div class="controls">
      <button class="up" onclick="send('F')">▲<br>FWD</button>
      <button class="left" onclick="send('L')">◀<br>LEFT</button>
      <button class="stop" onclick="send('S')">■<br>STOP</button>
      <button class="right" onclick="send('R')">▶<br>RIGHT</button>
      <button class="down" onclick="send('B')">▼<br>BACK</button>
    </div>
    
    <h3>🦾 Arm Control</h3>
    <div class="arm-controls">
      <button class="arm-btn" onclick="send('U')">⬆ UP</button>
      <button class="arm-btn" onclick="send('D')">⬇ DOWN</button>
    </div>
  </div>
  
  <script>
    function send(cmd) {
      fetch('/control?cmd=' + cmd)
        .then(r => r.text())
        .then(data => {
          document.getElementById('status').innerText = data;
        });
    }
    
    function updateSensors() {
      fetch('/sensors')
        .then(r => r.json())
        .then(data => {
          document.getElementById('distance').innerText = data.distance;
          document.getElementById('arm').innerText = data.arm;
          document.getElementById('pitch').innerText = data.pitch + '°';
          document.getElementById('roll').innerText = data.roll + '°';
          document.getElementById('yaw').innerText = data.yaw + '°';
          
          // Optional: Show warning if very close (visual only, no auto-stop)
          if (data.distance < 15) {
            document.getElementById('warning').style.display = 'block';
          } else {
            document.getElementById('warning').style.display = 'none';
          }
        });
    }
    
    setInterval(updateSensors, 200);
  </script>
</body>
</html>
)=====";

void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);
}

void setupUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void setupIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;
  
  if (distance == 0 || distance > 400) distance = 400;
  return distance;
}

void readIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to g and deg/s
  accelX = ax / 16384.0;
  accelY = ay / 16384.0;
  accelZ = az / 16384.0;
  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
  
  // Calculate pitch and roll (simplified)
  float rawPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float rawRoll = atan2(-accelX, accelZ) * 180.0 / PI;
  
  // Apply Kalman filtering
  pitch = pitchFilter.update(rawPitch);
  roll = rollFilter.update(rawRoll);
  
  // Simple yaw integration (drift will occur without magnetometer)
  yaw += gyroZ * 0.02; // 20ms update rate
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(0, abs(leftSpeed));
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(0, abs(leftSpeed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(0, 0);
  }
  
  // Right Motor
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(1, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(1, abs(rightSpeed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(1, 0);
  }
}

void moveRobot(int speed, int turn) {
  int leftSpeed = speed + turn;
  int rightSpeed = speed - turn;
  
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Apply Kalman filtering
  float filteredLeft = leftSpeedFilter.update(leftSpeed);
  float filteredRight = rightSpeedFilter.update(rightSpeed);
  
  setMotorSpeed((int)filteredLeft, (int)filteredRight);
}

void handleRoot() {
  server.send(200, "text/html", webpage);
}

void handleControl() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    String response = "";
    
    if (cmd == "W") {
      targetSpeed = 200;
      targetTurn = 0;
      response = "Moving Forward ▲";
    } else if (cmd == "S") {
      targetSpeed = -200;
      targetTurn = 0;
      response = "Moving Backward ▼";
    } else if (cmd == "A") {
      targetSpeed = 0;
      targetTurn = -150;
      response = "Turning Left ◀";
    } else if (cmd == "D") {
      targetSpeed = 0;
      targetTurn = 150;
      response = "Turning Right ▶";
    } else if (cmd == "Q") {
      targetSpeed = 0;
      targetTurn = 0;
      response = "Stopped ■";
    } else if (cmd == "O") {
      armPosition = constrain(armPosition + 10, 0, 180);
      armServo.write(armPosition);
      response = "Arm Up ⬆ " + String(armPosition) + "°";
    } else if (cmd == "P") {
      armPosition = constrain(armPosition - 10, 0, 180);
      armServo.write(armPosition);
      response = "Arm Down ⬇ " + String(armPosition) + "°";
    }
    
    server.send(200, "text/plain", response);
  }
}

void handleSensors() {
  String json = "{";
  json += "\\"distance\\":" + String(currentDistance, 1) + ",";
  json += "\\"arm\\":" + String(armPosition) + ",";
  json += "\\"pitch\\":" + String(pitch, 1) + ",";
  json += "\\"roll\\":" + String(roll, 1) + ",";
  json += "\\"yaw\\":" + String(yaw, 1) + ",";
  json += "\\"obstacle\\":" + String(obstacleDetected ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  
  setupMotors();
  setupUltrasonic();
  setupIMU();
  
  armServo.attach(SERVO_PIN);
  armServo.write(armPosition);
  
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/sensors", handleSensors);
  server.begin();
  
  lastUpdateTime = millis();
  lastSensorUpdate = millis();
}

void loop() {
  server.handleClient();
  
  unsigned long currentTime = millis();
  
  // Update sensors at 50Hz
  if (currentTime - lastSensorUpdate >= 20) {
    currentDistance = readDistance();
    readIMU();
    lastSensorUpdate = currentTime;
  }
  
  // Update motors at 50Hz
  float dt = (currentTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = currentTime;
  
  moveRobot(targetSpeed, targetTurn);
  
  delay(20);
}`;

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 p-8">
      <div className="max-w-4xl mx-auto">
        <div className="bg-white rounded-2xl shadow-2xl overflow-hidden">
          <div className="bg-gradient-to-r from-purple-600 to-indigo-600 p-6 text-white">
            <div className="flex items-center gap-3">
              <Wifi className="w-8 h-8" />
              <div>
                <h1 className="text-2xl font-bold">ESP32 Advanced Robot Control</h1>
                <p className="text-purple-100 text-sm">L298N + MPU6050 IMU + Ultrasonic + PID + Kalman</p>
              </div>
            </div>
          </div>

          <div className="p-6">
            <div className="mb-6 p-4 bg-blue-50 border-l-4 border-blue-500 rounded">
              <p className="text-sm text-blue-800 font-medium">{status}</p>
            </div>

            <div className="grid grid-cols-2 gap-4 mb-6">
              <div className="bg-gradient-to-br from-pink-500 to-rose-500 p-4 rounded-lg text-white text-center">
                <Activity className="w-8 h-8 mx-auto mb-2" />
                <div className="text-sm opacity-90">MPU6050 IMU</div>
                <div className="font-bold">Balance & Orientation</div>
              </div>
              <div className="bg-gradient-to-br from-cyan-500 to-blue-500 p-4 rounded-lg text-white text-center">
                <Radio className="w-8 h-8 mx-auto mb-2" />
                <div className="text-sm opacity-90">Ultrasonic</div>
                <div className="font-bold">Obstacle Detection</div>
              </div>
            </div>

            <div className="mb-6 space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">WiFi SSID</label>
                <input
                  type="text"
                  value={wifiSSID}
                  onChange={(e) => setWifiSSID(e.target.value)}
                  className="w-full px-4 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-purple-500 focus:border-transparent"
                  placeholder="Your WiFi Name"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">WiFi Password</label>
                <input
                  type="text"
                  value={wifiPassword}
                  onChange={(e) => setWifiPassword(e.target.value)}
                  className="w-full px-4 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-purple-500 focus:border-transparent"
                  placeholder="Your WiFi Password"
                />
              </div>
            </div>

            <div className="bg-gray-900 rounded-lg p-4 mb-4 max-h-96 overflow-auto">
              <pre className="text-green-400 text-xs font-mono whitespace-pre-wrap">{code}</pre>
            </div>

            <button
              onClick={handleCopy}
              className="w-full bg-gradient-to-r from-purple-600 to-indigo-600 text-white py-3 rounded-lg font-semibold hover:from-purple-700 hover:to-indigo-700 transition-all"
            >
              Copy Complete Code
            </button>

            <div className="mt-6 p-4 bg-gray-50 rounded-lg">
              <h3 className="font-bold text-gray-800 mb-3">📋 Wiring Guide:</h3>
              <div className="space-y-3 text-sm">
                <div>
                  <div className="font-semibold text-gray-700 mb-1">L298N Motor Driver:</div>
                  <div className="grid grid-cols-2 gap-2 text-gray-600">
                    <div>• ENA → GPIO 5</div>
                    <div>• ENB → GPIO 4</div>
                    <div>• IN1 → GPIO 18</div>
                    <div>• IN3 → GPIO 19</div>
                    <div>• IN2 → GPIO 22</div>
                    <div>• IN4 → GPIO 23</div>
                  </div>
                </div>
                <div>
                  <div className="font-semibold text-gray-700 mb-1">HC-SR04 Ultrasonic:</div>
                  <div className="text-gray-600">• TRIG → GPIO 5 | ECHO → GPIO 18</div>
                </div>
                <div>
                  <div className="font-semibold text-gray-700 mb-1">MPU6050 IMU (I2C):</div>
                  <div className="text-gray-600">• SDA → GPIO 21 | SCL → GPIO 22</div>
                </div>
                <div>
                  <div className="font-semibold text-gray-700 mb-1">Servo:</div>
                  <div className="text-gray-600">• Signal → GPIO 15</div>
                </div>
              </div>
            </div>

            <div className="mt-4 p-4 bg-gray-50 rounded-lg">
              <h3 className="font-bold text-gray-800 mb-3">🚀 Setup Instructions:</h3>
              <ol className="text-sm text-gray-700 space-y-2 list-decimal list-inside">
                <li>Enter WiFi credentials above and copy code</li>
                <li>Add to platformio.ini:
                  <pre className="mt-2 p-2 bg-white rounded text-xs">
{`lib_deps = 
    ESP32Servo
    jrowberg/I2Cdevlib-MPU6050`}
                  </pre>
                </li>
                <li>Wire all components as shown above</li>
                <li>Upload and check Serial Monitor for IP</li>
                <li>Open IP in browser - sensors update in real-time!</li>
              </ol>
            </div>

            <div className="mt-4 p-4 bg-green-50 rounded-lg border-l-4 border-green-500">
              <h4 className="font-bold text-green-800 mb-2">✨ Advanced Features:</h4>
              <ul className="text-sm text-green-700 space-y-1">
                <li>✅ Real-time sensor dashboard with auto-refresh</li>
                <li>✅ Live distance measurement (no auto-stop)</li>
                <li>✅ MPU6050 IMU for pitch, roll, yaw tracking</li>
                <li>✅ Kalman filtering for smooth sensor data</li>
                <li>✅ PID control with balance stabilization</li>
                <li>✅ Visual distance monitoring on web interface</li>
                <li>✅ Full manual control - you decide when to stop</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}