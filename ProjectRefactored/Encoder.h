#pragma once
#include <Arduino.h>

class Encoder {
public:
  Encoder(byte pinA, byte pinB, long countsPerRevolution);
  void begin();
  void poll();
  long count() const;
  void measure(unsigned long nowUs, unsigned long nowMs, byte windowTicks);
  void setSign(int sign);
  int sign() const;
  float rawRpm() const;
  float rpm() const;
  unsigned long lastCountChangeMs() const;
private:
  static Encoder *instance_;
  byte pinA_, pinB_;
  long countsPerRevolution_;
  volatile long count_;
  int sign_;
  byte historySize_, historyHead_;
  long *countHistory_;
  unsigned long *timeHistory_;
  bool historyPrimed_;
  long lastCountSeen_;
  unsigned long lastCountChangeMs_;
  float rawRpm_, rpm_;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  volatile uint8_t previousA_;
#endif
  void onPinChange();
  friend void PCINT0_vect(void);
};
