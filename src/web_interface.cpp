#include "web_interface.h"
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include <Arduino.h>

extern MotorControl motors;
extern Sensors sensors;

const char WebInterface::webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html> ... </html>
)=====";

WebInterface::WebInterface() : server(80), armPosition(SERVO_INITIAL) {}

void WebInterface::begin() {
  armServo.attach(SERVO_PIN);
  armServo.write(armPosition);

  server.on("/", [this](){ this->handleRoot(); });
  server.on("/control", [this](){ this->handleControl(); });
  server.on("/sensors", [this](){ this->handleSensors(); });

  server.begin();
  Serial.println("✓ Web server started");
}

void WebInterface::handleClient() { server.handleClient(); }

void WebInterface::handleRoot() { server.send_P(200,"text/html",webpage); }

void WebInterface::handleControl() {
  if(server.hasArg("cmd")){
    String cmd = server.arg("cmd");
    String resp="";

    if(cmd=="w") { motors.setSpeed(DEFAULT_SPEED,0); resp="Forward"; }
    else if(cmd=="s") { motors.setSpeed(-DEFAULT_SPEED,0); resp="Backward"; }
    else if(cmd=="a") { motors.setSpeed(0,-DEFAULT_TURN_SPEED); resp="Left"; }
    else if(cmd=="d") { motors.setSpeed(0,DEFAULT_TURN_SPEED); resp="Right"; }
    else if(cmd=="q") { motors.stop(); resp="Stopped"; }
    else if(cmd=="o") { armPosition=constrain(armPosition+SERVO_STEP,SERVO_MIN,SERVO_MAX); armServo.write(armPosition); resp="Arm Up"; }
    else if(cmd=="p") { armPosition=constrain(armPosition-SERVO_STEP,SERVO_MIN,SERVO_MAX); armServo.write(armPosition); resp="Arm Down"; }

    server.send(200,"text/plain",resp);
  }
}

void WebInterface::handleSensors() {
  String json="{";
  json += "\"distance\":"+String(sensors.getDistance(),1)+",";
  json += "\"arm\":"+String(armPosition)+",";
  json += "\"pitch\":"+String(sensors.getPitch(),1)+",";
  json += "\"roll\":"+String(sensors.getRoll(),1)+",";
  json += "\"yaw\":"+String(sensors.getYaw(),1);
  json += "}";
  server.send(200,"application/json",json);
}
