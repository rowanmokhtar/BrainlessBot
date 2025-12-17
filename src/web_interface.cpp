#include "web_interface.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include <Arduino.h>

extern MotorControl motors;
extern Sensors sensors;

const char WebInterface::webpage[] PROGMEM = R"=====(
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
    .warning {
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
    <h1>🤖 ESP32 Robot</h1>
    
    <div class="sensors">
      <div class="sensor-box">
        <div class="sensor-label">DISTANCE</div>
        <div class="sensor-value" id="distance">--</div>
        <div class="sensor-unit">cm</div>
      </div>
      <div class="sensor-box">
        <div class="sensor-label">ARM</div>
        <div class="sensor-value" id="arm">90</div>
        <div class="sensor-unit">°</div>
      </div>
    </div>
    
    <div class="imu-data">
      <div style="font-weight: bold; margin-bottom: 8px; text-align: center;">📊 IMU</div>
      <div class="imu-row"><span>Pitch:</span><span id="pitch">0°</span></div>
      <div class="imu-row"><span>Roll:</span><span id="roll">0°</span></div>
      <div class="imu-row"><span>Yaw:</span><span id="yaw">0°</span></div>
    </div>
    
    <div id="warning" class="warning" style="display: none;">
      ⚠️ OBSTACLE NEARBY
    </div>
    
    <div class="status" id="status">Ready</div>
    
    <div class="controls">
      <button class="up" onclick="send('F')">▲<br>FWD</button>
      <button class="left" onclick="send('L')">◀<br>LEFT</button>
      <button class="stop" onclick="send('S')">■<br>STOP</button>
      <button class="right" onclick="send('R')">▶<br>RIGHT</button>
      <button class="down" onclick="send('B')">▼<br>BACK</button>
    </div>
    
    <h3>🦾 Arm</h3>
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

WebInterface::WebInterface() : server(80), armPosition(SERVO_INITIAL) {}

void WebInterface::begin() {
  armServo.attach(SERVO_PIN);
  armServo.write(armPosition);
  
  server.on("/", [this]() { this->handleRoot(); });
  server.on("/control", [this]() { this->handleControl(); });
  server.on("/sensors", [this]() { this->handleSensors(); });
  
  server.begin();
  Serial.println("✓ Web server started");
}

void WebInterface::handleClient() {
  server.handleClient();
}

void WebInterface::handleRoot() {
  server.send(200, "text/html", webpage);
}

void WebInterface::handleControl() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    String response = "";
    
    if (cmd == "") {
      motors.setSpeed(DEFAULT_SPEED, 0);
      response = "Moving Forward ▲";
    } else if (cmd == "s") {
      motors.setSpeed(-DEFAULT_SPEED, 0);
      response = "Moving Backward ▼";
    } else if (cmd == "a") {
      motors.setSpeed(0, -DEFAULT_TURN_SPEED);
      response = "Turning Left ◀";
    } else if (cmd == "d") {
      motors.setSpeed(0, DEFAULT_TURN_SPEED);
      response = "Turning Right ▶";
    } else if (cmd == "q") {
      motors.stop();
      response = "Stopped ■";
    } else if (cmd == "o") {
      armPosition = constrain(armPosition + SERVO_STEP, SERVO_MIN, SERVO_MAX);
      armServo.write(armPosition);
      response = "Arm Up ⬆ " + String(armPosition) + "°";
    } else if (cmd == "p") {
      armPosition = constrain(armPosition - SERVO_STEP, SERVO_MIN, SERVO_MAX);
      armServo.write(armPosition);
      response = "Arm Down ⬇ " + String(armPosition) + "°";
    }
    
    server.send(200, "text/plain", response);
  }
}

void WebInterface::handleSensors() {
  String json = "{";
  json += "\"distance\":" + String(sensors.getDistance(), 1) + ",";
  json += "\"arm\":" + String(armPosition) + ",";
  json += "\"pitch\":" + String(sensors.getPitch(), 1) + ",";
  json += "\"roll\":" + String(sensors.getRoll(), 1) + ",";
  json += "\"yaw\":" + String(sensors.getYaw(), 1);
  json += "}";
  
  server.send(200, "application/json", json);
}