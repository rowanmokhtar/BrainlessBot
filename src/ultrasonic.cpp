#include "Ultrasonic.h"
#include "config.h"
#include <Arduino.h>

UltrasonicSensor::UltrasonicSensor() 
  : filter(DISTANCE_Q, DISTANCE_R), lastDistance(400.0) {}

void UltrasonicSensor::begin() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initial stabilization
  digitalWrite(TRIG_PIN, LOW);
  delay(50);
  
  Serial.println(" Ultrasonic sensor initialized");
  Serial.println("  TRIG: GPIO" + String(TRIG_PIN));
  Serial.println("  ECHO: GPIO" + String(ECHO_PIN));
  
  // Test reading
  float testDist = readRawDistance();
  Serial.println("  Test reading: " + String(testDist, 1) + " cm");
}

float UltrasonicSensor::readRawDistance() {
  // Ensure trigger is LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  
  // Send 10us pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo with timeout (38ms = ~650cm max range)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 38000);
  
  // Calculate distance in cm
  // Speed of sound = 343 m/s = 0.0343 cm/us
  // Distance = (duration * 0.0343) / 2
  float distance = (duration * 0.0343) / 2.0;
  
  // Validate reading
  if (duration == 0 || distance < 2.0 || distance > 400.0) {
    // Invalid reading, return last valid distance
    return lastDistance;
  }
  
  return distance;
}

float UltrasonicSensor::getDistance() {
  // Take multiple readings and average
  const int numReadings = 3;
  float sum = 0;
  int validReadings = 0;
  
  for (int i = 0; i < numReadings; i++) {
    float reading = readRawDistance();
    
    // Only include valid readings (within reasonable range)
    if (reading >= 2.0 && reading <= 400.0) {
      sum += reading;
      validReadings++;
    }
    
    // Small delay between readings
    if (i < numReadings - 1) {
      delay(10);
    }
  }
  
  // Calculate average of valid readings
  float avgDistance;
  if (validReadings > 0) {
    avgDistance = sum / validReadings;
  } else {
    avgDistance = lastDistance; // No valid readings, use last known
  }
  
  // Apply Kalman filter
  lastDistance = filter.update(avgDistance);
  
  return lastDistance;
}
// Get the last filtered distance
float UltrasonicSensor::getFilteredDistance() {
  return lastDistance;
}

// Get raw distance without filtering
float UltrasonicSensor::getRawDistance() {
  return readRawDistance();
}
