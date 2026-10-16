#include "MotorController.h"
MotorController::MotorController(byte in1, byte in2, bool reverse) : in1_(in1), in2_(in2), reverse_(reverse) {}
void MotorController::begin() { pinMode(in1_, OUTPUT); pinMode(in2_, OUTPUT); neutral(); }
void MotorController::leftPwm(int power) {
  power = constrain(power, 0, 255);
  if (!reverse_) { digitalWrite(in2_, HIGH); analogWrite(in1_, 255 - power); }
  else { digitalWrite(in2_, LOW); analogWrite(in1_, power); }
}
void MotorController::rightPwm(int power) {
  power = constrain(power, 0, 255);
  if (!reverse_) { digitalWrite(in2_, LOW); analogWrite(in1_, power); }
  else { digitalWrite(in2_, HIGH); analogWrite(in1_, 255 - power); }
}
void MotorController::neutral() { leftPwm(0); }
