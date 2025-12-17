#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "motor_control.h"
#include "sensors.h"
#include "web_interface.h"

MotorControl motors;
Sensors sensors;
WebInterface webInterface;

unsigned long lastMotorUpdate = 0;
unsigned long lastSensorUpdate = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP32 Robot Control System ===");
  
  // Initialize components
  motors.begin();
  sensors.begin();
  webInterface.begin();
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n✓ WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Open this IP in your browser to control the robot\n");
  
  lastMotorUpdate = millis();
  lastSensorUpdate = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle web requests
  webInterface.handleClient();
  
  // Update sensors at configured rate
  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_RATE) {
    sensors.updateIMU();
    sensors.updateDistance();
    lastSensorUpdate = currentTime;
  }
  
  // Update motors at configured rate
  if (currentTime - lastMotorUpdate >= MOTOR_UPDATE_RATE) {
    float dt = (currentTime - lastMotorUpdate) / 1000.0;
    motors.update(dt);
    lastMotorUpdate = currentTime;
  }
  
  delay(1);
}
