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

public:  
  static void IRAM_ATTR isrA_Left();
  static void IRAM_ATTR isrA_Right();
  
  Encoder(uint8_t pinA, uint8_t pinB, float wheelDiameter, int ppr);
  void begin();
  void update();
  void reset();
  
  long getCount() const;
  float getVelocity() const;
  float getDistance() const;
  float getRPM() const;
  
  void handleInterrupt();
  
  static Encoder* leftEncoder;
  static Encoder* rightEncoder;
};

#endif