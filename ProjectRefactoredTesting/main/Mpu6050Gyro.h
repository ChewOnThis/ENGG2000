#pragma once
#include <Arduino.h>
class Mpu6050Gyro {
public:
  Mpu6050Gyro(byte address, byte yawAxis, float sign, float lsbPerDps);
  bool begin();
  bool calibrate(unsigned long samples, unsigned long intervalMs);
  bool update(unsigned long nowMs);
  bool ready() const;
  bool initialized() const;
  byte whoAmI() const;
  float yawRateDps() const;
  float yawDegrees() const;
  float rateDps(byte axis) const;
  float angleDegrees(byte axis) const;
  unsigned long readFailures() const;
  unsigned long lastReadMs() const;
private:
  byte address_, axis_, whoAmI_; float sign_, lsb_, bias_, rate_, degrees_;
  float biases_[3], rates_[3], angles_[3];
  unsigned long lastUs_, lastMs_, failures_; bool ready_, initialized_;
  bool writeRegister(byte reg, byte value);
  bool readWhoAmI();
  bool readRaw(int16_t raw[3]);
};
