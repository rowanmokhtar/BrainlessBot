#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <WebServer.h>
#include <Arduino.h>

// Forward declarations
class MotorControl;
class UltrasonicSensor;
class ServoControl;
class MPU6050Sensor;
class Encoder;

// Global logging function - MUST be declared before class
void webLog(String msg);

class WebInterface {
private:
  WebServer server;
  MotorControl* motors;
  UltrasonicSensor* sensor;
  ServoControl* servo;
  MPU6050Sensor* imu;
  Encoder* leftEnc;
  Encoder* rightEnc;
  
  void handleRoot();
  void handleCommand();
  void handleData();
  void handleLogs();
  void handleClearLogs();

public:
  WebInterface();
  void begin(MotorControl* m, UltrasonicSensor* s, ServoControl* srv, 
             MPU6050Sensor* imu, Encoder* left, Encoder* right);
  void handleClient();
};

#endif