#include "Mpu6050Gyro.h"
#include <Wire.h>
Mpu6050Gyro::Mpu6050Gyro(byte a, byte axis, float sign, float lsb)
  : address_(a), axis_(axis), whoAmI_(0xff), sign_(sign), lsb_(lsb),
    bias_(0), rate_(0), degrees_(0), lastUs_(0), lastMs_(0), failures_(0),
    ready_(false), initialized_(false) {
  for (byte i = 0; i < 3; ++i) {
    biases_[i] = rates_[i] = angles_[i] = 0.0f;
  }
}
bool Mpu6050Gyro::writeRegister(byte r,byte v){Wire.beginTransmission(address_);Wire.write(r);Wire.write(v);return Wire.endTransmission()==0;}
bool Mpu6050Gyro::readWhoAmI() {
  Wire.beginTransmission(address_);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address_, (byte)1) != 1) return false;
  whoAmI_ = (byte)Wire.read();
  return true;
}
bool Mpu6050Gyro::readRaw(int16_t raw[3]) {
  Wire.beginTransmission(address_); Wire.write((byte)0x43);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address_, (byte)6) != 6) return false;
  for (byte i = 0; i < 3; ++i) raw[i] = (int16_t)(((uint16_t)(byte)Wire.read() << 8) | (byte)Wire.read());
  return true;
}
bool Mpu6050Gyro::begin() {
  Wire.begin(); Wire.setClock(400000UL);
  initialized_ = readWhoAmI() && (whoAmI_ == 0x68 || whoAmI_ == 0x69);
  ready_ = initialized_ && writeRegister(0x6B, 0x01) &&
           writeRegister(0x1A, 0x03) && writeRegister(0x19, 0x09) &&
           writeRegister(0x1B, 0x00);
  return ready_;
}
bool Mpu6050Gyro::calibrate(unsigned long samples, unsigned long intervalMs) {
  ready_ = false; if (!initialized_ || samples == 0) return false;
  double sums[3] = {0, 0, 0}; unsigned long good = 0; int16_t raw[3];
  for (unsigned long i = 0; i < samples; ++i) {
    if (readRaw(raw)) { for (byte a = 0; a < 3; ++a) sums[a] += raw[a]; ++good; }
    delay(intervalMs);
  }
  if (good == 0) return false;
  for (byte a = 0; a < 3; ++a) biases_[a] = sums[a] / good;
  bias_ = biases_[axis_]; rate_ = degrees_ = 0;
  for (byte a = 0; a < 3; ++a) rates_[a] = angles_[a] = 0;
  lastUs_ = micros(); lastMs_ = millis(); failures_ = 0; ready_ = true; return true;
}
bool Mpu6050Gyro::update(unsigned long nowMs) {
  int16_t raw[3]; if (!readRaw(raw)) { ++failures_; return false; }
  unsigned long nowUs = micros(); float dt = (float)(nowUs - lastUs_) * 1e-6f;
  for (byte a = 0; a < 3; ++a) {
    float measured = ((float)raw[a] - biases_[a]) / lsb_;
    rates_[a] += 0.15f * (measured - rates_[a]);
    if (dt > 0 && dt <= 0.1f) angles_[a] += rates_[a] * dt;
  }
  rate_ = sign_ * rates_[axis_]; degrees_ = sign_ * angles_[axis_];
  lastUs_ = nowUs; lastMs_ = nowMs; return true;
}
bool Mpu6050Gyro::ready()const{return ready_;}
bool Mpu6050Gyro::initialized()const{return initialized_;}
byte Mpu6050Gyro::whoAmI()const{return whoAmI_;}
float Mpu6050Gyro::yawRateDps()const{return rate_;}
float Mpu6050Gyro::yawDegrees()const{return degrees_;}
float Mpu6050Gyro::rateDps(byte axis)const{return axis < 3 ? rates_[axis] : 0.0f;}
float Mpu6050Gyro::angleDegrees(byte axis)const{return axis < 3 ? angles_[axis] : 0.0f;}
unsigned long Mpu6050Gyro::readFailures()const{return failures_;}
unsigned long Mpu6050Gyro::lastReadMs()const{return lastMs_;}
