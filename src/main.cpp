#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "web_interface.h"

MotorControl motors;
Sensors sensors;
WebInterface webInterface;

unsigned long lastMotorUpdate=0;
unsigned long lastSensorUpdate=0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Robot ===");

  motors.begin();
  sensors.begin();
  webInterface.begin();

  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while(WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println("\n✓ WiFi connected");
  Serial.println(WiFi.localIP());

  lastMotorUpdate=millis();
  lastSensorUpdate=millis();
}

void loop() {
  unsigned long now=millis();

  webInterface.handleClient();

  if(now-lastSensorUpdate >= SENSOR_UPDATE_RATE){
    sensors.updateIMU();
    sensors.updateDistance();
    lastSensorUpdate=now;
  }

  if(now-lastMotorUpdate >= MOTOR_UPDATE_RATE){
    float dt=(now-lastMotorUpdate)/1000.0;
    motors.update(dt);
    lastMotorUpdate=now;
  }

  delay(1);
}
