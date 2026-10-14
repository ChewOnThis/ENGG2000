#include "Encoder.h"
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#include <avr/interrupt.h>
#endif

Encoder *Encoder::instance_ = nullptr;
Encoder::Encoder(byte a, byte b, long cpr) : pinA_(a), pinB_(b), countsPerRevolution_(cpr), count_(0), sign_(1), historySize_(0), historyHead_(0), countHistory_(nullptr), timeHistory_(nullptr), historyPrimed_(false), lastCountSeen_(0), lastCountChangeMs_(0), rawRpm_(0), rpm_(0) {}
void Encoder::begin() {
  pinMode(pinA_, INPUT); pinMode(pinB_, INPUT); instance_ = this;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  previousA_ = PINB & _BV(1);
  PCIFR |= _BV(PCIF0); PCMSK0 |= _BV(PCINT1); PCICR |= _BV(PCIE0);
#endif
}
void Encoder::poll() {
#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__)
  static byte previous = LOW;
  byte current = digitalRead(pinA_);
  if (current == HIGH && previous == LOW) count_ += (digitalRead(pinB_) != current) ? 1 : -1;
  previous = current;
#endif
}
long Encoder::count() const { noInterrupts(); long value = count_; interrupts(); return value; }
void Encoder::setSign(int sign) { sign_ = sign < 0 ? -1 : 1; rpm_ = sign_ * rawRpm_; }
int Encoder::sign() const { return sign_; }
float Encoder::rawRpm() const { return rawRpm_; }
float Encoder::rpm() const { return rpm_; }
unsigned long Encoder::lastCountChangeMs() const { return lastCountChangeMs_; }
void Encoder::measure(unsigned long nowUs, unsigned long nowMs, byte windowTicks) {
  if (windowTicks < 1) windowTicks = 1;
  if (windowTicks + 1 != historySize_) {
    delete[] countHistory_; delete[] timeHistory_;
    historySize_ = windowTicks + 1; countHistory_ = new long[historySize_]; timeHistory_ = new unsigned long[historySize_]; historyHead_ = 0; historyPrimed_ = false;
  }
  long c = count();
  if (!historyPrimed_) { for (byte i=0; i<historySize_; ++i) { countHistory_[i]=c; timeHistory_[i]=nowUs; } historyPrimed_=true; lastCountSeen_=c; lastCountChangeMs_=nowMs; }
  historyHead_ = (historyHead_ + 1) % historySize_; countHistory_[historyHead_]=c; timeHistory_[historyHead_]=nowUs;
  byte oldest=(historyHead_+1)%historySize_; unsigned long dt=nowUs-timeHistory_[oldest];
  rawRpm_ = dt ? (float)(c-countHistory_[oldest])*60000000.0f/((float)countsPerRevolution_*(float)dt) : 0;
  rpm_ = sign_ * rawRpm_;
  if (c != lastCountSeen_) { lastCountSeen_=c; lastCountChangeMs_=nowMs; }
}
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
ISR(PCINT0_vect) { if (Encoder::instance_) Encoder::instance_->onPinChange(); }
void Encoder::onPinChange() { uint8_t pins=PINB, a=pins&_BV(1); if (a && !previousA_) count_ += (pins&_BV(2)) ? -1 : 1; previousA_=a; }
#endif
