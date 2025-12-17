#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <WebServer.h>
#include <ESP32Servo.h>

class WebInterface {
private:
  WebServer server;
  Servo armServo;
  int armPosition;

public:
  WebInterface();
  void begin();
  void handleClient();
  int getArmPosition() const { return armPosition; }
  
private:
  void handleRoot();
  void handleControl();
  void handleSensors();
  
  static const char webpage[];
};

#endif
