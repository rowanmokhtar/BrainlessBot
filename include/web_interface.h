#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <WebServer.h>
#include <ESP32Servo.h>

class WebInterface {
public:
  WebInterface();
  void begin();
  void handleClient();

private:
  WebServer server;
  Servo armServo;
  int armPosition;

  void handleRoot();
  void handleControl();
  void handleSensorData();
};

#endif
