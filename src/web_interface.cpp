#include "Web_Interface.h"
#include "Motor_Control.h"
#include "Ultrasonic.h"
#include "ServoControl.h"
#include "mpu.h"
#include "Encoder.h"
#include "config.h"
#include <Arduino.h>
const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:linear-gradient(135deg,#1e3a8a 0%,#7e22ce 100%);color:#fff;padding:15px}
.container{max-width:900px;margin:0 auto}
.card{background:rgba(255,255,255,0.1);backdrop-filter:blur(10px);border-radius:15px;padding:20px;margin:15px 0;border:1px solid rgba(255,255,255,0.2)}
h1{font-size:2em;text-align:center;margin-bottom:20px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:15px}
.sensor-value{font-size:2.5em;font-weight:bold;text-align:center;margin:10px 0}
.safe{color:#4ade80}
.warning{color:#fbbf24}
.danger{color:#ef4444;animation:pulse 0.5s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}
.label{font-size:0.9em;opacity:0.8;text-align:center}
.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:15px 0}
button{background:rgba(255,255,255,0.2);border:2px solid rgba(255,255,255,0.3);color:#fff;padding:15px;font-size:1em;border-radius:10px;cursor:pointer;transition:all 0.2s;font-weight:bold}
button:hover{background:rgba(255,255,255,0.3);transform:scale(1.05)}
button:active{transform:scale(0.95)}
.btn-forward{grid-column:2}
.btn-left{grid-column:1;grid-row:2}
.btn-right{grid-column:3;grid-row:2}
.btn-backward{grid-column:2;grid-row:2}
.btn-stop{background:linear-gradient(135deg,#ef4444,#dc2626);grid-column:1/4;margin-top:10px}
.servo-controls{display:flex;gap:10px}
.servo-controls button{flex:1}
.data-row{display:flex;justify-content:space-between;padding:8px;background:rgba(0,0,0,0.2);border-radius:8px;margin:5px 0}
.badge{display:inline-block;background:rgba(74,222,128,0.3);border:1px solid #4ade80;padding:3px 10px;border-radius:10px;font-size:0.75em;margin:5px 0}
</style>
</head>
<body>
<div class="container">
<h1>🤖 Advanced Robot Control</h1>
<div class="grid">
<div class="card">
<h2>📡 Distance</h2>
<div class="sensor-value safe" id="distance">--</div>
<div class="label">Kalman Filtered</div>
<div class="badge">✓ Auto-Stop Active</div>
</div>
<div class="card">
<h2>⚖️ IMU (MPU6050)</h2>
<div class="data-row"><span>Pitch:</span><span id="pitch">--°</span></div>
<div class="data-row"><span>Roll:</span><span id="roll">--°</span></div>
<div class="data-row"><span>Yaw:</span><span id="yaw">--°</span></div>
<div class="badge">🎯 Sensor Fusion</div>
</div>
</div>
<div class="card">
<h2>🎯 Encoders & Odometry</h2>
<div class="grid">
<div>
<div class="data-row"><span>Left Speed:</span><span id="leftSpeed">-- m/s</span></div>
<div class="data-row"><span>Right Speed:</span><span id="rightSpeed">-- m/s</span></div>
</div>
<div>
<div class="data-row"><span>Position X:</span><span id="posX">-- m</span></div>
<div class="data-row"><span>Position Y:</span><span id="posY">-- m</span></div>
<div class="data-row"><span>Heading:</span><span id="heading">--°</span></div>
</div>
</div>
<div class="badge">📍 Motion Tracking</div>
</div>
<div class="card">
<h2>🎮 Controls</h2>
<div class="controls">
<button class="btn-forward" onclick="cmd('w')">⬆️<br>FORWARD</button>
<button class="btn-left" onclick="cmd('a')">⬅️<br>LEFT</button>
<button class="btn-backward" onclick="cmd('s')">⬇️<br>BACK</button>
<button class="btn-right" onclick="cmd('d')">➡️<br>RIGHT</button>
</div>
<div class="servo-controls">
<button onclick="cmd('o')">🔼 ARM UP</button>
<button onclick="cmd('p')">🔽 ARM DOWN</button>
</div>
<button class="btn-stop" onclick="cmd('q')">🛑 STOP</button>
<button onclick="cmd('r')" style="margin-top:10px;grid-column:1/4">🔄 Reset Odometry</button>
</div>
</div>
<script>
function cmd(c){fetch('/cmd?c='+c).catch(e=>console.error(e))}
function update(){
fetch('/data').then(r=>r.json()).then(d=>{
document.getElementById('distance').textContent=d.dist.toFixed(1)+' cm';
document.getElementById('pitch').textContent=d.pitch.toFixed(1)+'°';
document.getElementById('roll').textContent=d.roll.toFixed(1)+'°';
document.getElementById('yaw').textContent=d.yaw.toFixed(1)+'°';
document.getElementById('leftSpeed').textContent=d.leftSpeed.toFixed(3)+' m/s';
document.getElementById('rightSpeed').textContent=d.rightSpeed.toFixed(3)+' m/s';
document.getElementById('posX').textContent=d.posX.toFixed(2)+' m';
document.getElementById('posY').textContent=d.posY.toFixed(2)+' m';
document.getElementById('heading').textContent=d.heading.toFixed(1)+'°';
let distEl=document.getElementById('distance');
if(d.dist<15){distEl.className='sensor-value danger'}
else if(d.dist<30){distEl.className='sensor-value warning'}
else{distEl.className='sensor-value safe'}
}).catch(e=>console.error(e))}
setInterval(update,100);update();
document.addEventListener('keydown',e=>{
const keys={'w':'w','W':'w','ArrowUp':'w','s':'s','S':'s','ArrowDown':'s',
'a':'a','A':'a','ArrowLeft':'a','d':'d','D':'d','ArrowRight':'d',
'q':'q','Q':'q',' ':'q','o':'o','O':'o','p':'p','P':'p','r':'r','R':'r'};
if(keys[e.key]){e.preventDefault();cmd(keys[e.key])}});
</script>
</body>
</html>
)=====";

WebInterface::WebInterface() : server(80) {}

void WebInterface::begin(MotorControl* m, UltrasonicSensor* s, ServoControl* srv,
                         MPU6050Sensor* i, Encoder* left, Encoder* right) {
  motors = m;
  sensor = s;
  servo = srv;
  imu = i;
  leftEnc = left;
  rightEnc = right;
  
  server.on("/", [this]() { handleRoot(); });
  server.on("/cmd", [this]() { handleCommand(); });
  server.on("/data", [this]() { handleData(); });
  
  server.begin();
  Serial.println("✓ Web server started");
}

void WebInterface::handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void WebInterface::handleCommand() {
  if (!server.hasArg("c")) {
    server.send(400, "text/plain", "Missing command");
    return;
  }
  
  String c = server.arg("c");
  
  // Convert PWM speeds to m/s (approximate)
  float speedMS = 0.3; // ~0.3 m/s at medium speed
  
  if (c == "w") motors->moveForward(speedMS);
  else if (c == "s") motors->moveBackward(speedMS);
  else if (c == "a") motors->turnLeft(speedMS * 0.7);
  else if (c == "d") motors->turnRight(speedMS * 0.7);
  else if (c == "q") motors->stop();
  else if (c == "o") servo->moveUp();
  else if (c == "p") servo->moveDown();
  else if (c == "r") motors->resetOdometry();
  
  server.send(200, "text/plain", "OK");
}

void WebInterface::handleData() {
  MotorState state = motors->getState();
  IMUData imuData = imu->getData();
  
  String json = "{";
  json += "\"dist\":" + String(sensor->getFilteredDistance(), 1);
  json += ",\"pitch\":" + String(imuData.pitch, 2);
  json += ",\"roll\":" + String(imuData.roll, 2);
  json += ",\"yaw\":" + String(imuData.yaw, 2);
  json += ",\"leftSpeed\":" + String(state.leftSpeed, 3);
  json += ",\"rightSpeed\":" + String(state.rightSpeed, 3);
  json += ",\"posX\":" + String(state.robotX, 3);
  json += ",\"posY\":" + String(state.robotY, 3);
  json += ",\"heading\":" + String(state.robotTheta * 180.0 / PI, 2);
  json += ",\"servo\":" + String(servo->getPosition());
  json += ",\"stopped\":" + String(motors->isAutoStopped() ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

void WebInterface::handleClient() {
  server.handleClient();
}