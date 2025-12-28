#include "ServoControl.h"
#include "config.h"
#include <Arduino.h>

ServoControl::ServoControl() : position(SERVO_INITIAL) {}

void ServoControl::begin() {

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
   servo.setPeriodHertz(50);  // Standard servo frequency 
  
  // Attach with wider pulse range for better compatibility
  if (!servo.attach(SERVO_PIN, 500, 2500)) {   // 1000, 2000 

    return;
  }
  
  // Initialize to center position
  delay(100);
  servo.write(position);
  delay(300); // Allow servo to reach position
  
}

void ServoControl::moveUp() {
  int newPos = min(position + SERVO_STEP, SERVO_MAX);
  if (newPos != position) {
    position = newPos;
    servo.write(position);
    Serial.println("ARM UP → " + String(position) + "°");
  } else {
    Serial.println("Already at maximum position");
  }
}

void ServoControl::moveDown() {
  int newPos = max(position - SERVO_STEP, SERVO_MIN);
  if (newPos != position) {
    position = newPos;
    servo.write(position);
    Serial.println("ARM DOWN → " + String(position) + "°");
  } else {
    Serial.println("Already at minimum position");
  }
}

void ServoControl::setPosition(int pos) {
  int newPos = constrain(pos, SERVO_MIN, SERVO_MAX);
  if (newPos != position) {
    position = newPos;
    servo.write(position);
    Serial.println("SERVO → " + String(position) + "°");
  }
}

int ServoControl::getPosition() const {
  return position;
}