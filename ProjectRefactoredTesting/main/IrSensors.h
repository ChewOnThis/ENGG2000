#pragma once
#include <Arduino.h>
class IRrecv;
enum Zone { NONE, CENTRE_ONLY };
class IrSensors {
public:
  IrSensors(byte centre, unsigned long holdMs);
  ~IrSensors();
  void begin(); void read(unsigned long now); void updateZone(unsigned long now);
  Zone zone() const; unsigned long zoneSince() const; unsigned long targetLastSeen() const;
  bool acquisitionConfirmed(unsigned long now) const;
  unsigned long detectionSerial() const;
  unsigned long acquisitionFirstSeen() const;
  unsigned long acquisitionLastSeen() const;
  static void timerInterrupt();
private:
  static IrSensors *instance_; IRrecv *centreReceiver_; byte centrePin_; unsigned long holdMs_, centreSeen_, targetSeen_, zoneSince_; Zone zone_;
  unsigned long acquireConfirmMs_, acquireGapMs_, acquireFirstSeen_, acquireLastSeen_, acquireFrames_, detectionSerial_;
  bool acquireConfirmed_;
};
