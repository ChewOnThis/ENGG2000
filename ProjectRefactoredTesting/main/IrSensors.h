#pragma once
#include <Arduino.h>
class IRrecv;
enum Zone { NONE, LEFT_ONLY, LEFT_CENTRE, CENTRE_ONLY, ALL_THREE, CENTRE_RIGHT, RIGHT_ONLY, LEFT_RIGHT };
class IrSensors {
public:
  IrSensors(byte left, byte centre, byte right, unsigned long holdMs);
  ~IrSensors();
  void begin(); void read(unsigned long now); void updateZone(unsigned long now);
  Zone zone() const; unsigned long zoneSince() const; unsigned long targetLastSeen() const; unsigned long lastLeft() const; unsigned long lastCentre() const; unsigned long lastRight() const;
  bool acquisitionConfirmed(unsigned long now) const;
  unsigned long detectionSerial() const;
  unsigned long acquisitionFirstSeen() const;
  unsigned long acquisitionLastSeen() const;
  static void timerInterrupt();
private:
  static IrSensors *instance_; IRrecv *centreReceiver_, *rightReceiver_; byte leftPin_, centrePin_, rightPin_; unsigned long holdMs_, leftSeen_, centreSeen_, rightSeen_, targetSeen_, zoneSince_; Zone zone_;
  unsigned long acquireConfirmMs_, acquireGapMs_, acquireFirstSeen_, acquireLastSeen_, acquireFrames_, detectionSerial_;
  bool acquireConfirmed_;
  static Zone calculate(bool l,bool c,bool r);
};
