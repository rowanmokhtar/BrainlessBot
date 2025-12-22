#include "ServoControl.h"
#include "config.h"
#include <Arduino.h>

ServoControl::ServoControl() : position(SERVO_INITIAL) {}

void ServoControl::begin() {
  servo.attach(SERVO_PIN);
  servo.write(position);
  Serial.println("Servo initialized at " + String(position) + "°");
}

void ServoControl::moveUp() {
  position = min(position + SERVO_STEP, SERVO_MAX);
  servo.write(position);
  Serial.println("ARM UP: " + String(position) + "°");
}

void ServoControl::moveDown() {
  position = max(position - SERVO_STEP, SERVO_MIN);
  servo.write(position);
  Serial.println("ARM DOWN: " + String(position) + "°");
}

void ServoControl::setPosition(int pos) {
  position = constrain(pos, SERVO_MIN, SERVO_MAX);
  servo.write(position);
}

int ServoControl::getPosition() const {
  return position;
}