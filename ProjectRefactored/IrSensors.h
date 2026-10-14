#pragma once
#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES
#include <Arduino.h>
#include <IRremote.hpp>
enum Zone { NONE, LEFT_ONLY, LEFT_CENTRE, CENTRE_ONLY, ALL_THREE, CENTRE_RIGHT, RIGHT_ONLY, LEFT_RIGHT };
class IrSensors {
public:
  IrSensors(byte left, byte centre, byte right, unsigned long holdMs);
  void begin(); void read(unsigned long now); void updateZone(unsigned long now);
  Zone zone() const; unsigned long zoneSince() const; unsigned long targetLastSeen() const; unsigned long lastLeft() const; unsigned long lastCentre() const; unsigned long lastRight() const;
  static void timerInterrupt();
private:
  static IrSensors *instance_; IRrecv centreReceiver_, rightReceiver_; byte leftPin_, centrePin_, rightPin_; unsigned long holdMs_, leftSeen_, centreSeen_, rightSeen_, targetSeen_, zoneSince_; Zone zone_;
  static Zone calculate(bool l,bool c,bool r);
};
