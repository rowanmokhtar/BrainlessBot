#include "Ultrasonic.h"
#include "config.h"
#include <Arduino.h>

UltrasonicSensor::UltrasonicSensor() 
  : filter(DISTANCE_Q, DISTANCE_R), lastDistance(400.0) {}

void UltrasonicSensor::begin() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Ultrasonic sensor initialized");
}

float UltrasonicSensor::readRawDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float dist = duration * 0.034 / 2.0;
  
  if (dist == 0 || dist > 400) dist = 400;
  return dist;
}

float UltrasonicSensor::getDistance() {
  float raw = readRawDistance();
  lastDistance = filter.update(raw);
  return lastDistance;
}

float UltrasonicSensor::getFilteredDistance() {
  return lastDistance;
}