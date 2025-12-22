#include "encoder.h"
#include "config.h"

// Static member initialization
Encoder* Encoder::leftEncoder = nullptr;
Encoder* Encoder::rightEncoder = nullptr;

Encoder::Encoder(uint8_t pinA, uint8_t pinB, float wheelDiameter, int ppr)
  : count(0), pinA(pinA), pinB(pinB), lastTime(0), velocity(0), distance(0) {
  
  // Calculate pulses per meter
  float wheelCircumference = PI * wheelDiameter;
  pulsesPerMeter = ppr / wheelCircumference;
}

void Encoder::begin() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  
  lastTime = micros();
}

void IRAM_ATTR Encoder::isrA_Left() {
  if (leftEncoder) leftEncoder->handleInterrupt();
}

void IRAM_ATTR Encoder::isrA_Right() {
  if (rightEncoder) rightEncoder->handleInterrupt();
}

void Encoder::handleInterrupt() {

  bool a = digitalRead(pinA);
  bool b = digitalRead(pinB);
  
  // Determine direction and increment/decrement count
  if (a == b) {
    count++;
  } else {
    count--;
  }
}

void Encoder::update() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
  
  if (dt >= 0.02) { // Update every 20ms
    // Calculate distance traveled
    float currentDistance = count / pulsesPerMeter;
    
    // Calculate velocity
    velocity = (currentDistance - distance) / dt;
    
    distance = currentDistance;
    lastTime = currentTime;
  }
}

void Encoder::reset() {
  count = 0;
  distance = 0;
  velocity = 0;
  lastTime = micros();
}

long Encoder::getCount() const {
  return count;
}

float Encoder::getVelocity() const {
  return velocity;
}

float Encoder::getDistance() const {
  return distance;
}

float Encoder::getRPM() const {
  // RPM = (velocity / wheel_circumference) * 60
  float wheelCircumference = PI * WHEEL_DIAMETER;
  return (velocity / wheelCircumference) * 60.0;
}