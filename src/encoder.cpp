#include "encoder.h"
#include "config.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, float wheelDiameter, int ppr)
  : count(0), pinA(pinA), pinB(pinB), lastTime(0), velocity(0), distance(0),
    lastStateA(false), lastStateB(false) {
  
  // Calculate pulses per meter
  float wheelCircumference = PI * wheelDiameter;
  pulsesPerMeter = ppr / wheelCircumference;
  
  Serial.println("Encoder created (POLLING MODE):");
  Serial.println("  Pin A: GPIO" + String(pinA));
  Serial.println("  Pin B: GPIO" + String(pinB));
  Serial.println("  PPR: " + String(ppr));
  Serial.println("  Pulses/meter: " + String(pulsesPerMeter, 2));
}

void Encoder::begin() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  
  // Small delay for pin stabilization
  delay(10);
  
  // Read initial states
  lastStateA = digitalRead(pinA);
  lastStateB = digitalRead(pinB);
  
  Serial.println("Encoder initialized:");
  Serial.println("  Pin A (GPIO" + String(pinA) + "): " + String(lastStateA ? "HIGH" : "LOW"));
  Serial.println("  Pin B (GPIO" + String(pinB) + "): " + String(lastStateB ? "HIGH" : "LOW"));
  
  lastTime = micros();
}

void Encoder::poll() {
  // Read current states
  bool currentA = digitalRead(pinA);
  bool currentB = digitalRead(pinB);
  
  // Check if Channel A changed
  if (currentA != lastStateA) {
    // On rising edge of A, check B for direction
    if (currentA == HIGH) {
      if (currentB == LOW) {
        count++;  // Forward (A leads B)
      } else {
        count--;  // Backward (B leads A)
      }
    }
    // On falling edge of A, check B for direction
    else {
      if (currentB == HIGH) {
        count++;  // Forward
      } else {
        count--;  // Backward
      }
    }
    lastStateA = currentA;
  }
  
  // Check if Channel B changed
  if (currentB != lastStateB) {
    // On rising edge of B, check A for direction
    if (currentB == HIGH) {
      if (currentA == HIGH) {
        count++;  // Forward (B follows A)
      } else {
        count--;  // Backward
      }
    }
    // On falling edge of B, check A for direction
    else {
      if (currentA == LOW) {
        count++;  // Forward
      } else {
        count--;  // Backward
      }
    }
    lastStateB = currentB;
  }
}

void Encoder::update() {
  // Poll encoder state
  poll();
  
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
  
  // Update velocity calculation every 50ms (not too fast, not too slow)
  if (dt >= 0.05) {
    // Calculate distance traveled
    float currentDistance = count / pulsesPerMeter;
    
    // Calculate velocity (m/s)
    float deltaDistance = currentDistance - distance;
    velocity = deltaDistance / dt;
    
    // Update stored values
    distance = currentDistance;
    lastTime = currentTime;
  }
}

void Encoder::reset() {
  count = 0;
  distance = 0;
  velocity = 0;
  lastTime = micros();
  
  Serial.println("Encoder reset (Pin A: GPIO" + String(pinA) + ")");
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