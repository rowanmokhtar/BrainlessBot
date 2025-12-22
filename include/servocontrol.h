#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <ESP32Servo.h>

class ServoControl {
private:
  Servo servo;
  int position;

public:
  ServoControl();
  void begin();
  void moveUp();
  void moveDown();
  void setPosition(int pos);
  int getPosition() const;
};

#endif