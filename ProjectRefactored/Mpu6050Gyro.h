#pragma once
#include <Arduino.h>
class Mpu6050Gyro {
public:
  Mpu6050Gyro(byte address, byte yawAxis, float sign, float lsbPerDps);
  bool begin();
  bool calibrate(unsigned long samples, unsigned long intervalMs);
  bool update(unsigned long nowMs);
  bool ready() const; float yawRateDps() const; float yawDegrees() const; unsigned long lastReadMs() const;
private:
  byte address_, axis_; float sign_, lsb_, bias_, rate_, degrees_; unsigned long lastUs_, lastMs_, failures_; bool ready_;
  bool writeRegister(byte reg, byte value); bool readRaw(int16_t &raw);
};
