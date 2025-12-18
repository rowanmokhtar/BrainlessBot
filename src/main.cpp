#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "web_interface.h"

// ===== Objects =====
MotorControl motors;
Sensors sensors;
WebInterface webInterface;

// ===== Timers =====
unsigned long lastMotorUpdate = 0;
unsigned long lastSensorUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
Serial.println("Hello ESP"); 
  WiFi.mode(WIFI_AP);
  WiFi.softAP("brainless", "yarab");

  Serial.println("✓ ESP32 Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  motors.begin();
  sensors.begin();
  webInterface.begin();

  lastMotorUpdate = millis();
  lastSensorUpdate = millis();
}


void loop() {
  unsigned long now = millis();

  // Handle Web Requests
  webInterface.handleClient();

  // Update Sensors
  if (now - lastSensorUpdate >= SENSOR_UPDATE_RATE) {
    sensors.updateIMU();
    sensors.updateDistance();
    lastSensorUpdate = now;
  }

  // Update Motors
  if (now - lastMotorUpdate >= MOTOR_UPDATE_RATE) {
    float dt = (now - lastMotorUpdate) / 1000.0;
    motors.update(dt);
    lastMotorUpdate = now;
  }

  delay(1);
}
