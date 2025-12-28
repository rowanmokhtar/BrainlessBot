#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
private:
  volatile long count;
  uint8_t pinA;
  uint8_t pinB;
  float pulsesPerMeter;
  unsigned long lastTime;
  float velocity;
  float distance;
  
  // State tracking for polling
  bool lastStateA;
  bool lastStateB;
  
  void poll();  // Internal polling function

public:
  Encoder(uint8_t pinA, uint8_t pinB, float wheelDiameter, int ppr);
  void begin();
  void update();  // Call this frequently to poll encoder
  void reset();
  
  long getCount() const;
  float getVelocity() const;
  float getDistance() const;
  float getRPM() const;
};

#endif