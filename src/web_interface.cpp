#include "web_interface.h"
#include "motor_control.h"
#include "config.h"

extern MotorControl motors;

const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<body style="text-align:center;font-size:30px;">
<h2>no brain , just kick</h2>

<button onclick="sendCmd('w')">FORWARD</button><br><br>
<button onclick="sendCmd('a')">LEFT</button>
<button onclick="sendCmd('d')">RIGHT</button><br><br>
<button onclick="sendCmd('s')">BACKWARD</button><br><br>
<button onclick="sendCmd('o')">ARM UP</button>
<button onclick="sendCmd('p')">ARM DOWN</button><br><br>
<button onclick="sendCmd('q')" style="background:red;color:white;">STOP</button>

<script>
function sendCmd(cmd){
  fetch('/control?cmd='+cmd);
}
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

  server.begin();
  Serial.println("Web server started");
}

void WebInterface::handleClient() {
  server.handleClient();
}

void WebInterface::handleRoot() {
  server.send_P(200, "text/html", webpage);
}
void WebInterface::handleControl() {
  if (!server.hasArg("cmd")) return;

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
