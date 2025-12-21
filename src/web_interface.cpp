#include "web_interface.h"
#include "motor_control.h"
#include "sensors.h"
#include "config.h"

extern MotorControl motors;
extern Sensors sensors;  

const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body {
  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
  text-align: center;
  background: linear-gradient(135deg, #1e3c72 0%, #2a5298 50%, #7e22ce 100%);
  color: white;
  margin: 0;
  padding: 20px;
  min-height: 100vh;
}

.container {
  max-width: 500px;
  margin: 0 auto;
  background: rgba(255,255,255,0.08);
  backdrop-filter: blur(15px);
  border-radius: 25px;
  padding: 30px;
  box-shadow: 0 8px 32px rgba(0,0,0,0.4);
  border: 1px solid rgba(255,255,255,0.1);
}

h2 {
  font-size: 2.2em;
  margin-bottom: 25px;
  text-shadow: 2px 2px 8px rgba(0,0,0,0.4);
  letter-spacing: 2px;
}

.sensor-panel {
  background: rgba(0,0,0,0.3);
  border-radius: 20px;
  padding: 25px;
  margin: 25px 0;
  border: 2px solid rgba(255,255,255,0.1);
  position: relative;
  overflow: hidden;
}

.sensor-panel::before {
  content: '';
  position: absolute;
  top: 0;
  left: -100%;
  width: 100%;
  height: 100%;
  background: linear-gradient(90deg, transparent, rgba(255,255,255,0.1), transparent);
  animation: scan 2s infinite;
}

@keyframes scan {
  0% { left: -100%; }
  100% { left: 100%; }
}

.distance-header {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 10px;
  margin-bottom: 15px;
}

.sensor-icon {
  font-size: 1.8em;
  animation: radar 2s infinite;
}

@keyframes radar {
  0%, 100% { transform: scale(1); opacity: 1; }
  50% { transform: scale(1.2); opacity: 0.7; }
}

.distance-label {
  font-size: 1.3em;
  font-weight: 600;
  letter-spacing: 1px;
}

.distance-display {
  font-size: 4em;
  font-weight: bold;
  margin: 20px 0;
  text-shadow: 0 0 30px currentColor;
  transition: all 0.3s ease;
  letter-spacing: 3px;
}

.safe {
  color: #4ade80;
  text-shadow: 0 0 20px #4ade80, 0 0 40px #22c55e;
}

.warning {
  color: #fbbf24;
  text-shadow: 0 0 20px #fbbf24, 0 0 40px #f59e0b;
  animation: pulse-warning 1s infinite;
}

.danger {
  color: #ef4444;
  text-shadow: 0 0 20px #ef4444, 0 0 40px #dc2626;
  animation: pulse-danger 0.5s infinite;
}

@keyframes pulse-warning {
  0%, 100% { opacity: 1; transform: scale(1); }
  50% { opacity: 0.7; transform: scale(1.05); }
}

@keyframes pulse-danger {
  0%, 100% { opacity: 1; transform: scale(1); }
  50% { opacity: 0.5; transform: scale(1.1); }
}

.status-message {
  font-size: 1.3em;
  font-weight: 600;
  padding: 12px 20px;
  border-radius: 12px;
  margin-top: 15px;
  transition: all 0.3s ease;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.status-safe {
  background: rgba(74, 222, 128, 0.2);
  border: 2px solid #4ade80;
}

.status-warning {
  background: rgba(251, 191, 36, 0.2);
  border: 2px solid #fbbf24;
  animation: border-pulse-warning 1s infinite;
}

.status-danger {
  background: rgba(239, 68, 68, 0.3);
  border: 2px solid #ef4444;
  animation: border-pulse-danger 0.5s infinite;
}

@keyframes border-pulse-warning {
  0%, 100% { border-color: #fbbf24; box-shadow: 0 0 10px rgba(251, 191, 36, 0.3); }
  50% { border-color: #f59e0b; box-shadow: 0 0 20px rgba(251, 191, 36, 0.5); }
}

@keyframes border-pulse-danger {
  0%, 100% { border-color: #ef4444; box-shadow: 0 0 15px rgba(239, 68, 68, 0.5); }
  50% { border-color: #dc2626; box-shadow: 0 0 30px rgba(239, 68, 68, 0.8); }
}

.distance-bar-container {
  width: 100%;
  height: 30px;
  background: rgba(0,0,0,0.3);
  border-radius: 15px;
  overflow: hidden;
  margin-top: 20px;
  border: 1px solid rgba(255,255,255,0.1);
}

.distance-bar {
  height: 100%;
  transition: width 0.3s ease, background-color 0.3s ease;
  box-shadow: 0 0 10px currentColor;
}

.zone-indicators {
  display: flex;
  justify-content: space-between;
  margin-top: 10px;
  font-size: 0.9em;
  opacity: 0.8;
}

.zone {
  padding: 5px 10px;
  border-radius: 8px;
  font-weight: 600;
}

.zone-danger { background: rgba(239, 68, 68, 0.2); color: #ef4444; }
.zone-warning { background: rgba(251, 191, 36, 0.2); color: #fbbf24; }
.zone-safe { background: rgba(74, 222, 128, 0.2); color: #4ade80; }

.control-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 12px;
  margin: 25px 0;
}

button {
  background: rgba(255,255,255,0.15);
  border: 2px solid rgba(255,255,255,0.25);
  color: white;
  padding: 22px;
  font-size: 1.1em;
  border-radius: 15px;
  cursor: pointer;
  transition: all 0.2s;
  font-weight: bold;
  text-shadow: 1px 1px 3px rgba(0,0,0,0.4);
}

button:hover {
  background: rgba(255,255,255,0.25);
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(0,0,0,0.3);
}

button:active {
  transform: translateY(0) scale(0.97);
}

.btn-forward { grid-column: 2; }
.btn-left { grid-column: 1; grid-row: 2; }
.btn-right { grid-column: 3; grid-row: 2; }
.btn-backward { grid-column: 2; grid-row: 2; }

.btn-stop {
  background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
  border-color: #dc2626;
  grid-column: 1 / 4;
  margin-top: 15px;
  font-size: 1.3em;
}

.arm-controls {
  display: flex;
  gap: 12px;
  margin: 20px 0;
}

.arm-controls button {
  flex: 1;
  background: rgba(139, 92, 246, 0.25);
  border-color: rgba(139, 92, 246, 0.4);
}

.connection-status {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  margin-top: 20px;
  font-size: 0.95em;
  color: rgba(255,255,255,0.7);
}

.status-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  animation: blink 1s infinite;
}

.status-online { background: #4ade80; }
.status-offline { background: #ef4444; }

@keyframes blink {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.3; }
}

.update-time {
  background: rgba(0,0,0,0.3);
  padding: 8px 15px;
  border-radius: 8px;
  font-size: 0.85em;
  margin-top: 10px;
}
</style>
</head>
<body>
<div class="container">
  <h2>🤖 NO BRAIN, JUST KICK</h2>
  
  <div class="sensor-panel">
    <div class="distance-header">
      <span class="sensor-icon">📡</span>
      <span class="distance-label">ULTRASONIC SENSOR</span>
    </div>
    
    <div class="distance-display safe" id="distance">--</div>
    
    <div class="status-message status-safe" id="statusMsg">
      Initializing...
    </div>
    
    <div class="distance-bar-container">
      <div class="distance-bar safe" id="distanceBar" style="width: 0%;"></div>
    </div>
    
    <div class="zone-indicators">
      <span class="zone zone-danger">0-15cm</span>
      <span class="zone zone-warning">15-30cm</span>
      <span class="zone zone-safe">30+ cm</span>
    </div>
  </div>

  <div class="control-grid">
    <button class="btn-forward" onclick="sendCmd('w')">⬆️<br>FORWARD</button>
    <button class="btn-left" onclick="sendCmd('a')">⬅️<br>LEFT</button>
    <button class="btn-backward" onclick="sendCmd('s')">⬇️<br>BACK</button>
    <button class="btn-right" onclick="sendCmd('d')">➡️<br>RIGHT</button>
  </div>

  <div class="arm-controls">
    <button onclick="sendCmd('o')">🔼 ARM UP</button>
    <button onclick="sendCmd('p')">🔽 ARM DOWN</button>
  </div>

  <button class="btn-stop" onclick="sendCmd('q')">🛑 EMERGENCY STOP</button>

  <div class="connection-status">
    <span class="status-dot status-online" id="statusDot"></span>
    <span id="connectionText">Connected</span>
  </div>
  
  <div class="update-time">
    Last update: <span id="lastUpdate">--</span>
  </div>
</div>

<script>
const MAX_DISTANCE = 400;

function sendCmd(cmd){
  fetch('/control?cmd='+cmd)
    .catch(err => {
      console.error('Command failed:', err);
      showConnectionError();
    });
}

function updateDistanceDisplay(distance) {
  const distEl = document.getElementById('distance');
  const statusEl = document.getElementById('statusMsg');
  const barEl = document.getElementById('distanceBar');
  
  distEl.textContent = distance.toFixed(1) + ' cm';
  
  let percentage = Math.max(0, Math.min(100, (MAX_DISTANCE - distance) / MAX_DISTANCE * 100));
  barEl.style.width = percentage + '%';
  
  distEl.className = 'distance-display';
  statusEl.className = 'status-message';
  barEl.className = 'distance-bar';
  
  if(distance < 15) {
    distEl.classList.add('danger');
    statusEl.classList.add('status-danger');
    barEl.classList.add('danger');
    statusEl.innerHTML = '🚨 OBSTACLE TOO CLOSE! 🚨';
  } else if(distance < 30) {
    distEl.classList.add('warning');
    statusEl.classList.add('status-warning');
    barEl.classList.add('warning');
    statusEl.innerHTML = '⚠️ OBSTACLE DETECTED';
  } else if(distance >= MAX_DISTANCE) {
    distEl.classList.add('safe');
    statusEl.classList.add('status-safe');
    barEl.classList.add('safe');
    statusEl.innerHTML = '✅ NO OBSTACLE IN RANGE';
  } else {
    distEl.classList.add('safe');
    statusEl.classList.add('status-safe');
    barEl.classList.add('safe');
    statusEl.innerHTML = '✅ PATH CLEAR';
  }
}

function showConnectionError() {
  document.getElementById('statusDot').className = 'status-dot status-offline';
  document.getElementById('connectionText').textContent = 'Connection Lost';
  document.getElementById('statusMsg').textContent = '❌ Sensor Offline';
}

function showConnectionSuccess() {
  document.getElementById('statusDot').className = 'status-dot status-online';
  document.getElementById('connectionText').textContent = 'Connected';
}

function updateSensorData() {
  fetch('/sensordata')
    .then(r => {
      if(!r.ok) throw new Error('Network response failed');
      return r.json();
    })
    .then(data => {
      updateDistanceDisplay(data.distance);
      showConnectionSuccess();
      
      //if auto-stopped, show message
      if(data.autoStopped) {
        const statusEl = document.getElementById('statusMsg');
        statusEl.innerHTML = 'AUTO-STOPPED! Obstacle at ' + data.distance.toFixed(1) + ' cm';
        statusEl.className = 'status-message status-danger';
      }
      
      const now = new Date();
      document.getElementById('lastUpdate').textContent = 
        now.toLocaleTimeString('en-US', { hour12: false });
    })
    .catch(err => {
      console.error('Sensor update failed:', err);
      showConnectionError();
    });
}

setInterval(updateSensorData, 200);
updateSensorData();

document.addEventListener('keydown', e => {
  const keyMap = {
    'w': 'w', 'W': 'w', 'ArrowUp': 'w',
    's': 's', 'S': 's', 'ArrowDown': 's',
    'a': 'a', 'A': 'a', 'ArrowLeft': 'a',
    'd': 'd', 'D': 'd', 'ArrowRight': 'd',
    'q': 'q', 'Q': 'q', ' ': 'q',
    'o': 'o', 'O': 'o',
    'p': 'p', 'P': 'p'
  };
  if(keyMap[e.key]) {
    e.preventDefault();
    sendCmd(keyMap[e.key]);
  }
});
</script>
</body>
</html>
)=====";

WebInterface::WebInterface() : server(80), armPosition(90) {}

void WebInterface::begin() {
  armServo.attach(SERVO_PIN);
  armServo.write(armPosition);

  server.on("/", [this]() { handleRoot(); });
  server.on("/control", [this]() { handleControl(); });
  server.on("/sensordata", [this]() { handleSensorData(); });  //ultrasonic sensor data endpoint

  server.begin();
  Serial.println("✓ Web server started");
}

void WebInterface::handleClient() {
  server.handleClient();
}

void WebInterface::handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void WebInterface::handleControl() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing cmd parameter");
    return;
  }

  String cmd = server.arg("cmd");

  if (cmd == "w") {
    motors.setSpeed(DEFAULT_SPEED, 0);
  }
  else if (cmd == "s") {
    motors.setSpeed(-DEFAULT_SPEED, 0);
  }
  else if (cmd == "a") {
    motors.setSpeed(0, -DEFAULT_TURN_SPEED);
  }
  else if (cmd == "d") {
    motors.setSpeed(0, DEFAULT_TURN_SPEED);
  }
  else if (cmd == "q") {
    motors.stop();
  }
  else if (cmd == "o") {
    armPosition = constrain(armPosition + SERVO_STEP, SERVO_MIN, SERVO_MAX);
    armServo.write(armPosition);
  }
  else if (cmd == "p") {
    armPosition = constrain(armPosition - SERVO_STEP, SERVO_MIN, SERVO_MAX);
    armServo.write(armPosition);
  }

  server.send(200, "text/plain", "OK");
}

//ultrasonic sensor data handler
void WebInterface::handleSensorData() {
  float distance = sensors.getDistance();
  
  String json = "{";
  json += "\"distance\":" + String(distance, 1);
  json += ",\"pitch\":" + String(sensors.getPitch(), 2);
  json += ",\"roll\":" + String(sensors.getRoll(), 2);
  json += ",\"armPos\":" + String(armPosition);
  json += ",\"autoStopped\":" + String(motors.isAutoStopActive() ? "true" : "false");  
  json += "}";
  
  server.send(200, "application/json", json);
}