#pragma once
#include <Arduino.h>

class MotorController {
public:
  MotorController(byte in1, byte in2, bool reverse = false);
  void begin();
  void leftPwm(int power);
  void rightPwm(int power);
  void neutral();
private:
  byte in1_, in2_;
  bool reverse_;
};
