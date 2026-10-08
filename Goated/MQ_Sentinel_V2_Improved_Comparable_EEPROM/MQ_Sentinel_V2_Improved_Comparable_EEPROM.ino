// ============================================================
// MQ SENTINEL - PRECISION V2 IMPROVED - COMPARABLE EEPROM DATA
// Arduino Nano
//
// D2  = Left IR sensor
// D3  = Centre IR sensor
// D4  = Right IR sensor
// D6  = Motor IN1 (PWM)
// D7  = Motor IN2
// D8  = Laser
// D9  = Encoder A
// D10 = Encoder B
//
// Main improvements:
//  1. Side detection starts braking BEFORE centre is reached.
//  2. Centre is treated as a capture region, not an instant "stop".
//  3. High-speed capture uses wheel RPM + side->centre crossing time.
//  4. After every correction, the new flywheel RPM is actively held.
//     This prevents flywheel/gearbox drag slowly giving momentum back
//     to the chassis after it looked stationary.
//  5. Fine alignment uses much smaller impulses and much longer
//     observation periods to reduce L-C-R hunting.
//  6. Laser only fires after a stable centre-only dwell.
//
// IMPORTANT:
// The reaction wheel is the actuator. Braking/locking the wheel does
// not directly lock the chassis. This controller therefore controls
// chassis motion by changing wheel momentum, then holds the resulting
// wheel speed while the satellite is aligned.
// ============================================================

#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES
void UserIRReceiveTimerInterruptHandler();
#include <IRremote.hpp>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// ============================================================
// PINS
// ============================================================

const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;

const byte MOTOR_IN1 = 6;   // PWM capable on Nano
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte ENCODER_A = 9;
const byte ENCODER_B = 10;

// ============================================================
// IR RECEIVERS
// ============================================================

IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

// ============================================================
// GENERAL SETTINGS
// ============================================================

const bool REVERSE_MOTOR = false;

// This is a genuine A/B experiment: same logger, power protection and pins.
// Only TUNED_CONTROL changes the tracking strategy.
const bool TUNED_CONTROL = true;
const char CONTROLLER_NAME[] = "V2_IMPROVED_COMPARISON_R4";

// Both variants use the SAME 190 PWM ceiling and 1.2 s startup ramp.
// Important: this is a stability fix shared by both, not an improvement metric.
const int MOTOR_SAFE_MAX_PWM = 190;
const unsigned long MOTOR_RAMP_MS = 1200UL;

// Keep battery-mode serial overhead out of motor/sensor timing.
const bool PRINT_LIVE_TELEMETRY = false;


// Search direction, using the same convention as the original code:
// -1 = motorLeftPWM(), +1 = motorRightPWM()
const int SEARCH_DIRECTION = -1;

// Shorter than the old 60 ms so a stale centre signal cannot survive
// for long while the chassis is moving quickly.
const unsigned long IR_HOLD_MS = 65;

// Centre-only must remain stable for this long before lock.
const unsigned long LOCK_CONFIRM_MS = 280;

// While locked, losing all target information for this long returns
// to fine alignment.
const unsigned long TARGET_LOST_MS = 130;

// Sensor zone must remain unchanged briefly before a precision
// correction is allowed.
const unsigned long FINE_ZONE_CONFIRM_MS = 28;
const unsigned long HOLD_ZONE_CONFIRM_MS = 22;

// ============================================================
// ENCODER / WHEEL SPEED
// ============================================================

const long COUNTS_PER_REV = 700;

long encoderCount = 0;
byte previousEncoderA = LOW;

long previousSpeedCount = 0;
unsigned long previousSpeedTime = 0;

float wheelRPM = 0.0;

// Sign of encoder RPM produced by motorLeftPWM().
// Learned automatically during the initial search.
int leftDriveEncoderSign = 0;

// ============================================================
// IR TIMING
// ============================================================

unsigned long lastLeftIR   = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR  = 0;
unsigned long targetLastSeen = 0;

// ============================================================
// SENSOR ZONE
//
// -2 = LEFT only
// -1 = LEFT + CENTRE
//  0 = CENTRE only (or all three)
// +1 = CENTRE + RIGHT
// +2 = RIGHT only
// 99 = no useful/ambiguous target
// ============================================================

int currentZone = 99;
int previousZone = 99;
unsigned long zoneSince = 0;

// ============================================================
// STATE MACHINE
// ============================================================

enum ControlState {
  SEARCHING,
  CAPTURE_APPROACH,
  CAPTURE_DAMPING,
  FINE_ALIGN,
  TARGET_LOCKED
};

ControlState state = SEARCHING;

unsigned long stateStartTime = 0;
unsigned long centreStableSince = 0;

// Direction in which the target was approached:
// -1 = from LEFT, +1 = from RIGHT
int approachSide = 0;
int lastKnownSide = 0;

unsigned long firstSideTime = 0;
unsigned long firstCentreTime = 0;

// ============================================================
// SEARCH TUNING
// ============================================================
//
// The old program allowed a large amount of wheel momentum to build.
// The satellite can still scan quickly, but the wheel is now ramped
// into a bounded search band rather than continuously driven hard.
// ============================================================

const float SEARCH_RPM_FAST_END = 95.0;
const float SEARCH_RPM_MID_END  = 125.0;
const float SEARCH_RPM_LIMIT    = 155.0;

const int SEARCH_POWER_FAST = 255;
const int SEARCH_POWER_MID  = 175;
const int SEARCH_POWER_LOW  = 95;

// ============================================================
// HIGH-SPEED CAPTURE TUNING
// ============================================================

// Pre-brake begins as soon as a side sensor sees the beacon.
// Exact values should be tuned from Serial telemetry.
int capturePreBrakePower = 0;
unsigned long capturePreBrakeDuration = 0;

int centreDampPower = 0;
unsigned long centreDampDuration = 0;

int counterDirection = 0;

bool centreSeenDuringCapture = false;
unsigned long captureObservationStart = 0;

// Maximum time to spend trying to damp one high-speed pass before
// handing over to the fine controller.
const unsigned long CAPTURE_TIMEOUT_MS = 850;

// Stable centre required during capture before handing to fine align.
const unsigned long CAPTURE_CENTRE_STABLE_MS = 140;

// ============================================================
// FINE ALIGNMENT TUNING
// ============================================================
//
// Near-centre corrections are intentionally tiny.
// Long settle times allow the suspended chassis to respond before
// another impulse is added.
// ============================================================

// LEFT/RIGHT only
const int FINE_OUTER_POWER = 112;
const unsigned long FINE_OUTER_PULSE_MS = 30;
const unsigned long FINE_OUTER_SETTLE_MS = 145;

// LEFT+CENTRE / CENTRE+RIGHT
const int FINE_INNER_POWER = 90;
const unsigned long FINE_INNER_PULSE_MS = 16;
const unsigned long FINE_INNER_SETTLE_MS = 160;

// ============================================================
// LOCKED HOLD TUNING
// ============================================================

const int HOLD_OUTER_POWER = 95;
const unsigned long HOLD_OUTER_PULSE_MS = 22;
const unsigned long HOLD_OUTER_SETTLE_MS = 170;

const int HOLD_INNER_POWER = 75;
const unsigned long HOLD_INNER_PULSE_MS = 12;
const unsigned long HOLD_INNER_SETTLE_MS = 180;

// ============================================================
// WHEEL-SPEED HOLD
//
// Once chassis motion has been arrested, the wheel may still be
// spinning quickly. If it is simply allowed to coast, gearbox/motor
// friction can slowly transfer that stored momentum back into the
// chassis. We therefore hold the wheel near the RPM produced by the
// last attitude correction.
// ============================================================

float holdWheelRPM = 0.0;
bool holdWheelSpeedValid = false;

const float MIN_WHEEL_HOLD_RPM = 18.0;
const float WHEEL_HOLD_DEADBAND_RPM = 4.0;

const int WHEEL_HOLD_MIN_POWER = 38;
const int WHEEL_HOLD_MAX_POWER = 82;

const unsigned long WHEEL_HOLD_PULSE_MS = 5;
const unsigned long WHEEL_HOLD_CHECK_MS = 18;

bool wheelHoldPulseActive = false;
unsigned long wheelHoldPulseEnd = 0;
unsigned long lastWheelHoldCheck = 0;

// ============================================================
// POINTING TORQUE PULSE MANAGER
// ============================================================

bool torquePulseActive = false;
unsigned long torquePulseEnd = 0;
unsigned long nextTorqueAllowed = 0;

// When true, capture the new wheel RPM when the pulse finishes and
// maintain that speed afterwards.
bool captureWheelSpeedAtPulseEnd = true;

// ============================================================
// SERIAL
// ============================================================

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 120;

void motorCoast();  // Safe-output helper defined below.

// ============================================================
// V2 EEPROM EXPERIMENT LOGGER -- CLASSIC ARDUINO NANO, NO JUMPER
// ============================================================
// Recording sequence:
//   USB / Serial Monitor at 115200: C = clear & arm NEXT boot
//   Disconnect USB, insert Nano, switch on battery: record 5 mins.
//   Battery off; remove Nano; connect USB: D = dump results.
// Saved data is never erased at boot. Old recordings remain until C.
// NOTE: This logger records V2 as it ACTUALLY behaves: its 'centre'
// zone combines centre-only and all-three, and LASER can stay on beyond
// 2.1 seconds. The export separates both sensor conditions.
// EEPROM: 300 x 2-byte 1s snapshots, 49 x 7-byte recent events,
// plus one persistent summary/header; compile-time bound checks.
// ============================================================

const uint16_t FLIGHT_MAGIC = 0xB2E7;
const byte FLIGHT_VERSION = 4;
const uint32_t FLIGHT_RUN_MS = 300000UL;
const uint16_t FLIGHT_MAX_SNAPSHOTS = 300;
const byte FLIGHT_MAX_EVENTS = 49;
const uint16_t SHOT_QUALIFY_MS = 2100;

// Packed physical IR zone IDs. Not the controller's zone IDs.
// 0 none, 1 left, 2 left+centre, 3 centre-only, 4 centre+right,
// 5 right, 6 ambiguous (left+right), 7 all-three.
// State IDs preserve original V2 state values (0=SEARCH ... 4=LOCKED).
struct __attribute__((packed)) FlightHeader {
  uint16_t magic;
  byte version;
  byte armed;           // 1 if C command prepared the NEXT power-up
  byte phase;           // 0=armed/idle,1=recording,2=finished
  byte head;            // event ring insertion position
  byte eventCount;
  uint16_t snapshotCount;
  uint16_t unexpectedRestarts; // Power resets during recording (not successes)
  uint16_t targetSeenCount;
  uint16_t targetLostCount;
  uint16_t zoneTransitions;
  uint16_t allThreeArrivals;
  uint16_t centreZoneArrivals;
  uint16_t directionChanges;
  uint16_t confirmedAlignments;
  uint16_t shotAttempts;
  uint16_t shotCompletions;
  uint16_t shotAborts;
  uint16_t maxLaserOnMs;
  uint16_t peakRPM10;
  uint16_t acquireCount;
  uint32_t totalAcquireMs;
  uint32_t totalLaserOnMs;
  uint32_t encoderPulses;
  uint32_t firstDetectionMs;
  uint32_t firstAlignmentMs;
  uint32_t lastElapsedMs;
};

struct __attribute__((packed)) FlightEventRecord {
  uint16_t t10;       // 10ms units from experiment start; 5 minutes fits
  byte kind;
  byte zoneStateDir;  // 3 bits zone + 3 bits state + 2 bits motor dir
  int8_t rpm2;        // wheel RPM, quantised to 2 RPM units
  uint16_t auxMs;    // e.g. laser duration or acquisition duration
};

struct __attribute__((packed)) FlightSnapshot {
  byte zoneStateDir;
  int8_t rpm2;
};

const int FLIGHT_HEADER_ADDRESS = 0;
const int FLIGHT_EVENTS_ADDRESS = sizeof(FlightHeader);
const int FLIGHT_SNAP_ADDRESS = FLIGHT_EVENTS_ADDRESS +
       (int)FLIGHT_MAX_EVENTS * (int)sizeof(FlightEventRecord);
const int FLIGHT_BYTES = FLIGHT_SNAP_ADDRESS +
       (int)FLIGHT_MAX_SNAPSHOTS * (int)sizeof(FlightSnapshot);
static_assert(FLIGHT_BYTES <= 1024, "Comparable V2 logger exceeds Nano 1KB");

// Numerical values are used in function parameters to avoid Arduino .ino
// auto-prototype issues with enums.
const byte EV_BOOT = 1;
const byte EV_TARGET_SEEN = 2;
const byte EV_TARGET_LOST = 3;
const byte EV_ALL_THREE = 4;
const byte EV_ALIGNED = 5;
const byte EV_DIRECTION_CHANGE = 6;
const byte EV_CENTRE_ZONE = 7;
const byte EV_LASER_ON = 8;
const byte EV_LASER_ABORT = 9;
const byte EV_SHOT_COMPLETE = 10;
const byte EV_ZONE_CHANGE = 11;

bool flightRecording();

// Persistent acquisition timestamps use 0xFFFFFFFF to mean "never happened".
// They are NOT valid measurements and must not be graphed as huge numbers.
void printOptionalTimestamp(uint32_t timestamp) {
  if (timestamp == 0xFFFFFFFFUL || timestamp > FLIGHT_RUN_MS) Serial.println(F("NA"));
  else Serial.println(timestamp);
}

unsigned long centreBrakedAtZoneTime = 0;


// Independent encoder monitor: does NOT change encoderCount/wheelRPM used
// by the original V2 control. It remains accurate during EEPROM writes.
volatile long flightEncoderPosition = 0;
volatile uint32_t flightEncoderEdges = 0;
volatile byte flightPreviousEncoderA = LOW;
long flightLastRPMCount = 0;
unsigned long flightLastRPMTime = 0;
float flightMeasuredRPM = 0.0f;

ISR(PCINT0_vect) {
  byte a = (PINB & _BV(PB1)) ? HIGH : LOW;
  if (a == HIGH && flightPreviousEncoderA == LOW) {
    byte b = (PINB & _BV(PB2)) ? HIGH : LOW;
    if (b != a) flightEncoderPosition++;
    else flightEncoderPosition--;
    flightEncoderEdges++;
  }
  flightPreviousEncoderA = a;
}

void flightUpdateMeasuredRPM(unsigned long now) {
  if (!flightRecording()) return;
  if (now - flightLastRPMTime < 50UL) return;
  long count;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = flightEncoderPosition; }
  unsigned long dt = now - flightLastRPMTime;
  long delta = count - flightLastRPMCount;
  flightMeasuredRPM = ((float)delta / (float)COUNTS_PER_REV)
                     * (60000.0f / (float)dt);
  flightLastRPMCount = count;
  flightLastRPMTime = now;
}

FlightHeader flight;
unsigned long flightStart = 0;
unsigned long flightNextSnapshot = 0;
unsigned long flightNextSave = 0;
unsigned long flightAcquisitionStarted = 0;
unsigned long flightLaserStarted = 0;
byte flightPrevIRZone = 255;
byte flightPrevState = 255;
byte flightLastMotorDir = 0;
byte flightLastNonzeroDir = 0;
byte flightCurrentDir = 0;
byte flightCurrentPower = 0;
bool flightLaserWasOn = false;
bool flightLaserQualified = false;

bool flightRecording() { return flight.magic == FLIGHT_MAGIC && flight.phase == 1; }

uint32_t flightElapsed(unsigned long now) { return now - flightStart; }

void flightSave() { EEPROM.put(FLIGHT_HEADER_ADDRESS, flight); }

void flightClearArm() {
  memset(&flight, 0, sizeof(flight));
  flight.magic = FLIGHT_MAGIC;
  flight.version = FLIGHT_VERSION;
  flight.armed = 1;
  flight.phase = 0;
  flight.firstDetectionMs = 0xFFFFFFFFUL;
  flight.firstAlignmentMs = 0xFFFFFFFFUL;
  flightSave();
}

byte flightIRZone(bool L, bool C, bool R) {
  if (L && C && R) return 7;
  if (L && R) return 6;
  if (L && C) return 2;
  if (C && R) return 4;
  if (L) return 1;
  if (R) return 5;
  if (C) return 3;
  return 0;
}

byte flightPack(byte zone) {
  return (byte)(zone | (((byte)state & 7) << 3) | (flightCurrentDir << 6));
}

int8_t flightRPM2() {
  float n = flightMeasuredRPM / 2.0f;
  if (n > 127.0f) n = 127.0f;
  if (n < -128.0f) n = -128.0f;
  return (int8_t)n;
}

const __FlashStringHelper* flightEventName(byte e) {
  switch (e) {
    case EV_BOOT:             return F("BOOT");
    case EV_TARGET_SEEN:      return F("TARGET_SEEN");
    case EV_TARGET_LOST:      return F("TARGET_LOST");
    case EV_ALL_THREE:        return F("ALL_THREE");
    case EV_ALIGNED:          return F("ALIGNED");
    case EV_DIRECTION_CHANGE: return F("DIRECTION_CHANGE");
    case EV_CENTRE_ZONE:      return F("CENTRE_ZONE");
    case EV_LASER_ON:         return F("LASER_ON");
    case EV_LASER_ABORT:      return F("LASER_ABORT");
    case EV_SHOT_COMPLETE:    return F("SHOT_COMPLETE");
    case EV_ZONE_CHANGE:      return F("ZONE_CHANGE");
  }
  return F("OTHER");
}

void flightEvent(byte kind, unsigned long now, uint16_t auxiliary) {
  if (!flightRecording()) return;
  FlightEventRecord r;
  uint32_t t = flightElapsed(now) / 10UL;
  r.t10 = (uint16_t)(t > 65535UL ? 65535UL : t);
  r.kind = kind;
  r.zoneStateDir = flightPack(flightPrevIRZone == 255 ? 0 : flightPrevIRZone);
  r.rpm2 = flightRPM2();
  r.auxMs = auxiliary;
  EEPROM.put(FLIGHT_EVENTS_ADDRESS + (int)flight.head * sizeof(r), r);
  flight.head = (flight.head + 1) % FLIGHT_MAX_EVENTS;
  if (flight.eventCount < FLIGHT_MAX_EVENTS) flight.eventCount++;
}

void trackMotorCommand(int direction, int power) {
  // Called by the EXISTING motor outputs; no control commands are changed.
  byte d = (direction < 0) ? 1 : ((direction > 0) ? 2 : 0);
  flightCurrentDir = d;
  flightCurrentPower = (byte)constrain(power, 0, 255);
  if (!flightRecording() || d == 0) return;
  if (flightLastNonzeroDir != 0 && d != flightLastNonzeroDir) {
    flight.directionChanges++;
    // Direction changes can occur in fast pulses. Count all of them,
    // but do not write an event for every pulse (disturbs timing).
  }
  flightLastNonzeroDir = d;
}

void flightBegin(unsigned long now) {
  if (!flight.armed || flight.phase != 0) return;
  flight.armed = 0;
  flight.phase = 1;
  flightStart = now;
  flightNextSnapshot = now + 1000UL;
  flightNextSave = now + 1000UL;
  flightAcquisitionStarted = 0;
  flightLaserStarted = 0;
  flightPrevIRZone = 255;
  flightPrevState = 255;
  flightLastNonzeroDir = 0;
  flightLaserWasOn = false;
  flightLaserQualified = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    flightEncoderPosition = 0;
    flightEncoderEdges = 0;
  }
  flightLastRPMCount = 0;
  flightLastRPMTime = now;
  flightMeasuredRPM = 0.0f;
  flight.head = 0;
  flight.eventCount = 0;
  flight.snapshotCount = 0;
  flightSave();
  flightEvent(EV_BOOT, now, 0);
  flightSave();
}

void flightCheckZone(byte zone, unsigned long now) {
  if (!flightRecording()) return;
  if (flightPrevIRZone == 255) {
    // A beacon may already be in view when the experiment starts.
    flightPrevIRZone = 0;
    if (zone == 0) return;
  }
  if (flightPrevIRZone == zone) return;
  byte old = flightPrevIRZone;
  flightPrevIRZone = zone;
  flight.zoneTransitions++;
  if (old == 0 && zone != 0) {
    flight.targetSeenCount++;
    flightAcquisitionStarted = now;
    if (flight.firstDetectionMs == 0xFFFFFFFFUL)
      flight.firstDetectionMs = flightElapsed(now);
    flightEvent(EV_TARGET_SEEN, now, 0);
  }
  if (old != 0 && zone == 0) {
    flight.targetLostCount++;
    flightEvent(EV_TARGET_LOST, now, 0);
    // Keep acquisition timing running through brief signal loss so we
    // measure time to actual lock, not just the latest sensor pulse.
  }
  if (zone == 7 && old != 7) {
    flight.allThreeArrivals++;
    flightEvent(EV_ALL_THREE, now, 0);
  }
  if ((zone == 3 || zone == 7) && !(old == 3 || old == 7)) {
    flight.centreZoneArrivals++;
    flightEvent(EV_CENTRE_ZONE, now, 0);
  }
}

void flightCheckLock(unsigned long now) {
  if (!flightRecording()) return;
  if (flightPrevState != (byte)state) {
    if (state == TARGET_LOCKED) {
      flight.confirmedAlignments++;
      uint16_t acquireMs = 0;
      if (flightAcquisitionStarted) {
        uint32_t t = now - flightAcquisitionStarted;
        flight.acquireCount++;
        flight.totalAcquireMs += t;
        acquireMs = (uint16_t)(t > 65535UL ? 65535UL : t);
        flightAcquisitionStarted = 0;
      }
      if (flight.firstAlignmentMs == 0xFFFFFFFFUL)
        flight.firstAlignmentMs = flightElapsed(now);
      flightEvent(EV_ALIGNED, now, acquireMs);
    }
    flightPrevState = (byte)state;
  }
}

void flightFinishLaser(unsigned long now) {
  if (!flightLaserWasOn || !flightRecording()) return;
  uint32_t elapsed = now - flightLaserStarted;
  flight.totalLaserOnMs += elapsed;
  if (elapsed > flight.maxLaserOnMs)
    flight.maxLaserOnMs = (uint16_t)(elapsed > 65535UL ? 65535UL : elapsed);
  if (elapsed >= SHOT_QUALIFY_MS) {
    flight.shotCompletions++;
    flightEvent(EV_SHOT_COMPLETE, now,
                (uint16_t)(elapsed > 65535UL ? 65535UL : elapsed));
  } else {
    flight.shotAborts++;
    flightEvent(EV_LASER_ABORT, now, (uint16_t)elapsed);
  }
  flightLaserWasOn = false;
  flightLaserQualified = false;
  flightSave();
}

void flightCheckLaser(unsigned long now) {
  if (!flightRecording()) return;
  bool on = digitalRead(LASER_PIN) == HIGH;
  if (on && !flightLaserWasOn) {
    flightLaserStarted = now;
    flightLaserWasOn = true;
    flightLaserQualified = false;
    flight.shotAttempts++;
    flightEvent(EV_LASER_ON, now, 0);
  } else if (!on && flightLaserWasOn) {
    flightFinishLaser(now);
  }
  if (on && flightLaserWasOn && !flightLaserQualified &&
      now - flightLaserStarted >= SHOT_QUALIFY_MS) {
    // Only a tentative milestone; completion recorded on laser OFF.
    flightLaserQualified = true;
  }
}

void flightSample(unsigned long now) {
  if (!flightRecording()) return;
  // Record the sample at 300 s before marking the run finished.
  if (now >= flightNextSnapshot && flight.snapshotCount < FLIGHT_MAX_SNAPSHOTS) {
    FlightSnapshot r;
    r.zoneStateDir = flightPack(flightPrevIRZone == 255 ? 0 : flightPrevIRZone);
    r.rpm2 = flightRPM2();
    EEPROM.put(FLIGHT_SNAP_ADDRESS + (int)flight.snapshotCount * sizeof(r), r);
    flight.snapshotCount++;
    flightNextSnapshot += 1000UL;
  }
  if (now - flightStart >= FLIGHT_RUN_MS) {
    if (flightLaserWasOn) flightFinishLaser(now);
    flight.phase = 2;
    flight.lastElapsedMs = FLIGHT_RUN_MS;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { flight.encoderPulses = flightEncoderEdges; }
    flightSave();
    // The demonstration has ended; do not restart motors on USB power.
    motorCoast();
    digitalWrite(LASER_PIN, LOW);
    return;
  }
  float rpm10 = flightMeasuredRPM < 0 ? -flightMeasuredRPM * 10.0f : flightMeasuredRPM * 10.0f;
  if (rpm10 > (float)flight.peakRPM10)
    flight.peakRPM10 = (uint16_t)(rpm10 > 65535.0f ? 65535.0f : rpm10);
  if (now >= flightNextSave) {
    flight.lastElapsedMs = flightElapsed(now);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { flight.encoderPulses = flightEncoderEdges; }
    flightSave();
    flightNextSave += 1000UL;
  }
}

void flightDump() {
  EEPROM.get(FLIGHT_HEADER_ADDRESS, flight);
  Serial.println(F("BEGIN SENTINEL PRECISION V2 EEPROM"));
  if (flight.magic != FLIGHT_MAGIC || flight.version != FLIGHT_VERSION ||
      flight.eventCount > FLIGHT_MAX_EVENTS ||
      flight.snapshotCount > FLIGHT_MAX_SNAPSHOTS) {
    Serial.println(F("NO_VALID_LOG"));
    Serial.println(F("END SENTINEL PRECISION V2 EEPROM"));
    return;
  }
  Serial.print(F("status,controller,")); Serial.println(CONTROLLER_NAME);
  Serial.println(F("status,logger_version,4"));
  Serial.print(F("status,phase,")); Serial.println(flight.phase);
  Serial.print(F("status,armed,")); Serial.println(flight.armed);
  Serial.print(F("status,eeprom_bytes,")); Serial.println(FLIGHT_BYTES);
  Serial.print(F("status,snapshot_count,")); Serial.println(flight.snapshotCount);
  Serial.print(F("status,recent_event_count,")); Serial.println(flight.eventCount);
  Serial.println(F("status,alignment_rule,V2_CENTRE_ONLY_OR_ALL_THREE"));
  Serial.println(F("status,shot_rule,CONTINUOUS_LASER_ON_AT_LEAST_2100MS"));
  Serial.print(F("metric,unexpected_restarts,")); Serial.println(flight.unexpectedRestarts);
  Serial.print(F("metric,target_seen,")); Serial.println(flight.targetSeenCount);
  Serial.print(F("metric,target_lost,")); Serial.println(flight.targetLostCount);
  Serial.print(F("metric,zone_transitions,")); Serial.println(flight.zoneTransitions);
  Serial.print(F("metric,all_three_arrivals,")); Serial.println(flight.allThreeArrivals);
  Serial.print(F("metric,centre_zone_arrivals,")); Serial.println(flight.centreZoneArrivals);
  Serial.print(F("metric,direction_changes,")); Serial.println(flight.directionChanges);
  Serial.print(F("metric,confirmed_alignments,")); Serial.println(flight.confirmedAlignments);
  Serial.print(F("metric,shot_attempts,")); Serial.println(flight.shotAttempts);
  Serial.print(F("metric,shot_completions,")); Serial.println(flight.shotCompletions);
  Serial.print(F("metric,shot_aborts,")); Serial.println(flight.shotAborts);
  Serial.print(F("metric,max_laser_on_ms,")); Serial.println(flight.maxLaserOnMs);
  Serial.print(F("metric,total_laser_on_ms,")); Serial.println(flight.totalLaserOnMs);
  Serial.print(F("metric,encoder_pulses,")); Serial.println(flight.encoderPulses);
  Serial.print(F("metric,peak_wheel_rpm_x10,")); Serial.println(flight.peakRPM10);
  Serial.print(F("metric,acquire_count,")); Serial.println(flight.acquireCount);
  Serial.print(F("metric,total_acquire_ms,")); Serial.println(flight.totalAcquireMs);
  Serial.print(F("metric,first_detection_ms,")); printOptionalTimestamp(flight.firstDetectionMs);
  Serial.print(F("metric,first_alignment_ms,")); printOptionalTimestamp(flight.firstAlignmentMs);
  Serial.print(F("metric,last_elapsed_ms,")); Serial.println(flight.lastElapsedMs);
  Serial.println(F("section,event_records_latest"));
  Serial.println(F("time_ms,event,zone,state,direction,rpm_approx,aux_ms"));
  byte start = (flight.eventCount == FLIGHT_MAX_EVENTS) ? flight.head : 0;
  for (byte i=0; i<flight.eventCount; i++) {
    byte slot = (start + i) % FLIGHT_MAX_EVENTS;
    FlightEventRecord r;
    EEPROM.get(FLIGHT_EVENTS_ADDRESS + slot * (int)sizeof(r), r);
    Serial.print((uint32_t)r.t10 * 10UL); Serial.print(',');
    Serial.print(flightEventName(r.kind)); Serial.print(',');
    Serial.print(r.zoneStateDir & 7); Serial.print(',');
    Serial.print((r.zoneStateDir >> 3) & 7); Serial.print(',');
    Serial.print((r.zoneStateDir >> 6) & 3); Serial.print(',');
    Serial.print((int)r.rpm2 * 2); Serial.print(',');
    Serial.println(r.auxMs);
  }
  Serial.println(F("section,snapshots_1_per_second"));
  Serial.println(F("time_ms,zone,state,direction,rpm_approx"));
  for (uint16_t i=0; i<flight.snapshotCount; i++) {
    FlightSnapshot r;
    EEPROM.get(FLIGHT_SNAP_ADDRESS + i * (int)sizeof(r), r);
    Serial.print((uint32_t)(i+1) * 1000UL); Serial.print(',');
    Serial.print(r.zoneStateDir & 7); Serial.print(',');
    Serial.print((r.zoneStateDir >> 3) & 7); Serial.print(',');
    Serial.print((r.zoneStateDir >> 6) & 3); Serial.print(',');
    Serial.println((int)r.rpm2 * 2);
  }
  Serial.println(F("END SENTINEL PRECISION V2 EEPROM"));
}

void flightDumpMode() {
  motorCoast();
  digitalWrite(LASER_PIN, LOW);
  Serial.println(F("DATA MODE: D=dump, C=clear+arm next battery experiment"));
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c=='D' || c=='d') flightDump();
      else if (c=='C' || c=='c') {
        flightClearArm();
        Serial.println(F("ARMED. Unplug USB, reinstall Nano, power from batteries."));
      }
      else if (c=='I' || c=='i') {
        Serial.print(F("EEPROM bytes: ")); Serial.println(FLIGHT_BYTES);
        Serial.print(F("Phase: ")); Serial.println(flight.phase);
      }
    }
  }
}

// ============================================================
// SMALL HELPERS
// ============================================================

float absFloat(float x) {
  return (x < 0.0f) ? -x : x;
}

int signOfFloat(float x) {
  if (x > 0.0f) return 1;
  if (x < 0.0f) return -1;
  return 0;
}

int signOfInt(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void motorLeftPWM(int power) {
  power = constrain(power, 0, 255);
  trackMotorCommand(-1, power);

  if (!REVERSE_MOTOR) {
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_IN1, 255 - power);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, power);
  }
}

void motorRightPWM(int power) {
  power = constrain(power, 0, 255);
  trackMotorCommand(+1, power);

  if (!REVERSE_MOTOR) {
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, power);
  } else {
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_IN1, 255 - power);
  }
}

void motorBrake() {
  trackMotorCommand(0, 0);
  analogWrite(MOTOR_IN1, 255);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

void motorCoast() {
  trackMotorCommand(0, 0);
  analogWrite(MOTOR_IN1, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void driveDirection(int direction, int power) {
  // Shared safety limit: prevent hard 255 PWM startup / brownout bursts.
  power = constrain(power, 0, MOTOR_SAFE_MAX_PWM);
  if (flightRecording()) {
    uint32_t age = millis() - flightStart;
    if (age < MOTOR_RAMP_MS) {
      int rampMax = 85 + (int)(age * (MOTOR_SAFE_MAX_PWM - 85) / MOTOR_RAMP_MS);
      if (power > rampMax) power = rampMax;
    }
  }
  if (direction < 0) {
    motorLeftPWM(power);
  } else if (direction > 0) {
    motorRightPWM(power);
  } else {
    motorCoast();
  }
}

// ============================================================
// ENCODER
// ============================================================

void updateEncoder() {
  // Shared reliability fix: the existing PCINT encoder counter is used for
  // BOTH motor control and EEPROM. Polling misses pulses during EEPROM writes.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { encoderCount = flightEncoderPosition; }
}

void updateWheelSpeed(unsigned long now) {
  if (now - previousSpeedTime < 40) {
    return;
  }

  unsigned long dt = now - previousSpeedTime;
  long delta = encoderCount - previousSpeedCount;

  wheelRPM =
      ((float)delta / (float)COUNTS_PER_REV) *
      (60000.0f / (float)dt);

  previousSpeedCount = encoderCount;
  previousSpeedTime = now;
}

float getWheelAngle() {
  return ((float)encoderCount * 360.0f) / (float)COUNTS_PER_REV;
}

// ============================================================
// WHEEL DIRECTION LEARNING
// ============================================================

void learnLeftDriveEncoderSign() {
  // Search uses motorLeftPWM(). Once the encoder clearly reports motion,
  // remember which encoder sign corresponds to that motor direction.
  if (leftDriveEncoderSign == 0 && absFloat(wheelRPM) > 25.0f) {
    leftDriveEncoderSign = signOfFloat(wheelRPM);

    Serial.print(">>> Learned motorLeft encoder sign: ");
    Serial.println(leftDriveEncoderSign);
  }
}

int motorDirectionForWheelSign(int wheelSign) {
  // If mapping has not yet been learned, do not guess aggressively.
  if (leftDriveEncoderSign == 0 || wheelSign == 0) {
    return 0;
  }

  if (wheelSign == leftDriveEncoderSign) {
    return -1; // motorLeftPWM()
  }

  return +1;   // motorRightPWM()
}

// ============================================================
// WHEEL-SPEED HOLD
// ============================================================

void disableWheelSpeedHold() {
  holdWheelSpeedValid = false;
  wheelHoldPulseActive = false;
}

void captureCurrentWheelSpeedForHold() {
  holdWheelRPM = wheelRPM;

  holdWheelSpeedValid =
      (leftDriveEncoderSign != 0) &&
      (absFloat(holdWheelRPM) >= MIN_WHEEL_HOLD_RPM);

  if (holdWheelSpeedValid) {
    Serial.print(">>> Holding wheel at ");
    Serial.print(holdWheelRPM, 1);
    Serial.println(" RPM");
  }
}

void stopWheelHoldPulse() {
  wheelHoldPulseActive = false;
  motorCoast();
}

void maintainWheelSpeed(unsigned long now) {
  if (TUNED_CONTROL && currentZone != 0) {
    // Don't continue maintaining old wheel momentum while the target
    // has moved to the opposite side. Let new pointing pulses take over.
    disableWheelSpeedHold();
    motorCoast();
    return;
  }
  if (!holdWheelSpeedValid) {
    motorCoast();
    return;
  }

  // Finish a small maintenance pulse.
  if (wheelHoldPulseActive) {
    if (now - wheelHoldPulseEnd < 0x80000000UL && now >= wheelHoldPulseEnd) {
      stopWheelHoldPulse();
    }
    return;
  }

  if (now - lastWheelHoldCheck < WHEEL_HOLD_CHECK_MS) {
    return;
  }

  lastWheelHoldCheck = now;

  int targetSign = signOfFloat(holdWheelRPM);

  if (targetSign == 0) {
    motorCoast();
    return;
  }

  float targetMagnitude = absFloat(holdWheelRPM);

  // Signed current speed in the target wheel direction.
  float currentAlongTarget = wheelRPM * (float)targetSign;

  float speedError = targetMagnitude - currentAlongTarget;

  // If wheel is at/above target speed, do not add torque.
  // Let the tiny excess decay naturally.
  if (speedError <= WHEEL_HOLD_DEADBAND_RPM) {
    motorCoast();
    return;
  }

  int motorDirection = motorDirectionForWheelSign(targetSign);

  if (motorDirection == 0) {
    motorCoast();
    return;
  }

  int power =
      WHEEL_HOLD_MIN_POWER +
      (int)(speedError * 2.0f);

  power = constrain(
      power,
      WHEEL_HOLD_MIN_POWER,
      WHEEL_HOLD_MAX_POWER
  );

  driveDirection(motorDirection, power);

  wheelHoldPulseActive = true;
  wheelHoldPulseEnd = now + WHEEL_HOLD_PULSE_MS;
}

// ============================================================
// POINTING TORQUE PULSES
// ============================================================

void cancelWheelHoldPulse() {
  wheelHoldPulseActive = false;
}

void startTorquePulse(
    int direction,
    int power,
    unsigned long duration,
    unsigned long settleTime,
    unsigned long now,
    bool captureSpeedAfter = true) {

  if (torquePulseActive) {
    return;
  }

  cancelWheelHoldPulse();

  torquePulseActive = true;
  torquePulseEnd = now + duration;
  nextTorqueAllowed = torquePulseEnd + settleTime;
  captureWheelSpeedAtPulseEnd = captureSpeedAfter;

  driveDirection(direction, power);
}

void serviceTorquePulse(unsigned long now) {
  if (!torquePulseActive) {
    return;
  }

  if (now >= torquePulseEnd) {
    torquePulseActive = false;
    motorCoast();

    // The new wheel speed is the momentum state that corresponds to
    // the attitude correction we just made. Preserve it rather than
    // allowing it to decay back into chassis motion.
    if (captureWheelSpeedAtPulseEnd) {
      captureCurrentWheelSpeedForHold();
    }
  }
}

bool torqueReady(unsigned long now) {
  return !torquePulseActive && now >= nextTorqueAllowed;
}

// ============================================================
// SENSOR ZONE
// ============================================================

int calculateZone(bool L, bool C, bool R) {
  if (C) {
    if (L && !R) return -1;
    if (R && !L) return +1;

    // Centre-only or all-three is treated as the central capture zone.
    return 0;
  }

  if (L && !R) return -2;
  if (R && !L) return +2;

  return 99;
}

void updateZone(int newZone, unsigned long now) {
  if (newZone != currentZone) {
    if (TUNED_CONTROL && state != SEARCHING) {
      int a = (currentZone < 0) ? -1 : ((currentZone == 1 || currentZone == 2) ? 1 : 0);
      int b = (newZone < 0) ? -1 : ((newZone == 1 || newZone == 2) ? 1 : 0);
      if (a != 0 && b != 0 && a != b) {
        // Sudden LEFT <-> RIGHT change: never finish a stale command.
        torquePulseActive = false;
        disableWheelSpeedHold();
        motorCoast();
        nextTorqueAllowed = now + 25UL;
      }
    }
    previousZone = currentZone;
    currentZone = newZone;
    zoneSince = now;
  }
}

bool zoneStableFor(unsigned long duration, unsigned long now) {
  return currentZone != 99 && (now - zoneSince >= duration);
}

// ============================================================
// STATE HELPERS
// ============================================================

const char* stateName(ControlState s) {
  switch (s) {
    case SEARCHING:         return "SEARCH";
    case CAPTURE_APPROACH:  return "CAPTURE_APPROACH";
    case CAPTURE_DAMPING:   return "CAPTURE_DAMP";
    case FINE_ALIGN:        return "FINE";
    case TARGET_LOCKED:     return "LOCKED";
  }

  return "?";
}

void changeState(ControlState next, unsigned long now) {
  if (state == next) {
    return;
  }

  state = next;
  stateStartTime = now;

  Serial.print("***** STATE -> ");
  Serial.println(stateName(state));
}

// ============================================================
// SEARCH DRIVE
// ============================================================

void runSearchDrive() {
  disableWheelSpeedHold();

  if (TUNED_CONTROL) {
    // Continuous full PWM can saturate the flywheel and twist the string.
    // Short coasts limit wheel energy while still scanning autonomously.
    uint32_t elapsed = millis() - stateStartTime;
    if ((elapsed % 300UL) >= 220UL) { motorCoast(); return; }
  }

  float rpm = absFloat(wheelRPM);

  // If the wheel is still rotating strongly in the opposite encoder
  // direction from the normal search direction, use full power to
  // recover the intended search direction.
  if (leftDriveEncoderSign != 0 &&
      absFloat(wheelRPM) > 20.0f &&
      signOfFloat(wheelRPM) != leftDriveEncoderSign) {

    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_FAST);
    return;
  }

  if (rpm < SEARCH_RPM_FAST_END) {
    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_FAST);
  }
  else if (rpm < SEARCH_RPM_MID_END) {
    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_MID);
  }
  else if (rpm < SEARCH_RPM_LIMIT) {
    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_LOW);
  }
  else {
    // At search speed, stop accelerating the wheel further.
    motorCoast();
  }
}

// ============================================================
// CAPTURE STRENGTH CALCULATION
// ============================================================

void choosePreBrakeFromWheelRPM() {
  float rpm = absFloat(wheelRPM);

  if (rpm >= 145.0f) {
    capturePreBrakePower = 255;
    capturePreBrakeDuration = 72;
  }
  else if (rpm >= 115.0f) {
    capturePreBrakePower = 225;
    capturePreBrakeDuration = 54;
  }
  else if (rpm >= 80.0f) {
    capturePreBrakePower = 180;
    capturePreBrakeDuration = 38;
  }
  else {
    capturePreBrakePower = 125;
    capturePreBrakeDuration = 22;
  }
}

void chooseCentreDamping(
    unsigned long crossingTimeMs,
    float rpmMagnitude) {

  // Very quick side->centre movement means the chassis still has
  // significant angular velocity, regardless of where the wheel RPM
  // happens to be after the pre-brake pulse.
  if (crossingTimeMs <= 70 || rpmMagnitude >= 145.0f) {
    centreDampPower = 255;
    centreDampDuration = 105;
  }
  else if (crossingTimeMs <= 125 || rpmMagnitude >= 115.0f) {
    centreDampPower = 225;
    centreDampDuration = 78;
  }
  else if (crossingTimeMs <= 210 || rpmMagnitude >= 80.0f) {
    centreDampPower = 180;
    centreDampDuration = 55;
  }
  else {
    centreDampPower = 130;
    centreDampDuration = 32;
  }
}

// ============================================================
// START TARGET CAPTURE
// ============================================================

void beginCaptureFromSide(
    int side,
    unsigned long now) {

  if (TUNED_CONTROL) {
    // The old V2 incorrectly assumes target side identifies angular
    // velocity, and immediately pre-brakes in -side direction.
    // Instead: stop the search drive and steer toward the observed side
    // using bounded impulses with sensor feedback.
    motorCoast();
    torquePulseActive = false;
    disableWheelSpeedHold();
    nextTorqueAllowed = now + 35UL;
    centreStableSince = 0;
    approachSide = side;
    lastKnownSide = side;
    changeState(FINE_ALIGN, now);
    return;
  }

  approachSide = side;
  lastKnownSide = side;

  firstSideTime = now;
  firstCentreTime = 0;

  centreSeenDuringCapture = false;
  centreStableSince = 0;

  // Opposite attitude command removes the existing chassis motion.
  counterDirection = -approachSide;

  choosePreBrakeFromWheelRPM();

  Serial.print(">>> SIDE ACQUIRE ");
  Serial.print(side < 0 ? "LEFT" : "RIGHT");
  Serial.print(" | wheel=");
  Serial.print(wheelRPM, 1);
  Serial.print(" RPM | pre-brake ");
  Serial.print(capturePreBrakePower);
  Serial.print(" / ");
  Serial.print(capturePreBrakeDuration);
  Serial.println(" ms");

  disableWheelSpeedHold();

  startTorquePulse(
      counterDirection,
      capturePreBrakePower,
      capturePreBrakeDuration,
      25,
      now,
      true
  );

  changeState(CAPTURE_APPROACH, now);
}

void beginCaptureFromCentre(unsigned long now) {
  if (TUNED_CONTROL) {
    motorCoast();
    torquePulseActive = false;
    disableWheelSpeedHold();
    nextTorqueAllowed = now + 80UL;
    centreStableSince = 0;
    approachSide = SEARCH_DIRECTION;
    changeState(FINE_ALIGN, now);
    return;
  }
  // If centre is hit directly during the normal search, the body is
  // almost certainly travelling in the search direction.
  approachSide = SEARCH_DIRECTION;
  lastKnownSide = approachSide;

  firstSideTime = now;
  firstCentreTime = now;

  centreSeenDuringCapture = true;
  centreStableSince = 0;

  counterDirection = -approachSide;

  chooseCentreDamping(
      0,
      absFloat(wheelRPM)
  );

  disableWheelSpeedHold();

  startTorquePulse(
      counterDirection,
      centreDampPower,
      centreDampDuration,
      45,
      now,
      true
  );

  captureObservationStart = now;

  changeState(CAPTURE_DAMPING, now);
}

void flightResume(unsigned long now) {
  if (flight.phase != 1) return;
  flight.unexpectedRestarts++;
  flightStart = now - flight.lastElapsedMs;
  flightNextSnapshot = now + 1000UL;
  flightNextSave = now + 1000UL;
  flightLastRPMTime = now;
  flightLastRPMCount = 0;
  flightMeasuredRPM = 0;
  flightPrevIRZone = 255;
  flightPrevState = 255;
  flightLastNonzeroDir = 0;
  flightLaserWasOn = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    flightEncoderPosition = 0;
    flightEncoderEdges = flight.encoderPulses;
  }
  flightSave();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Safe initial state.
  motorBrake();

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  // A USB reset must never overwrite an already-recorded experiment.
  EEPROM.get(FLIGHT_HEADER_ADDRESS, flight);
  if (flight.magic != FLIGHT_MAGIC || flight.version != FLIGHT_VERSION ||
      flight.eventCount > FLIGHT_MAX_EVENTS ||
      flight.snapshotCount > FLIGHT_MAX_SNAPSHOTS) {
    memset(&flight, 0, sizeof(flight));
  }

  if (!((flight.phase == 0 && flight.armed == 1) || flight.phase == 1)) {
    flightDumpMode();
  }

  // The Nano cannot sense USB versus battery reliably. A 10-second window
  // allows D to enter safe dump mode, even after an unexpected reset.
  Serial.print(F("FIRMWARE,")); Serial.println(CONTROLLER_NAME);
  if (flight.phase == 1) Serial.println(F("RECOVERY: send D now to dump saved test"));
  else Serial.println(F("ARMED: send D to dump, or disconnect USB for battery run"));
  unsigned long startWindow = millis();
  while (millis() - startWindow < 10000UL) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c=='D' || c=='d' || c=='C' || c=='c') flightDumpMode();
    }
  }

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  previousEncoderA = digitalRead(ENCODER_A);
  flightPreviousEncoderA = previousEncoderA;
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT1);  // D9 / PB1 / PCINT1; D10 sampled for direction
  previousSpeedTime = millis();

  IrReceiver.begin(
      LEFT_SENSOR,
      false
  );

  centreReceiver.begin(
      CENTRE_SENSOR,
      false
  );

  rightReceiver.begin(
      RIGHT_SENSOR,
      false
  );

  stateStartTime = millis();
  zoneSince = millis();
  if (flight.phase == 1) flightResume(millis());
  else flightBegin(millis());

  Serial.println();
  Serial.println("================================================");
  Serial.println(" MQ SENTINEL PRECISION TRACKER V2");
  Serial.println("================================================");
  Serial.println("Early braking + centre damping + wheel-speed hold");
  Serial.println("Fine corrections use small impulse / long settle.");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  if (!flightRecording()) {
    // This run completed: remain safe until next USB connection/clear.
    motorCoast();
    digitalWrite(LASER_PIN, LOW);
    flightDumpMode();
  }
  unsigned long now = millis();

  updateEncoder();
  updateWheelSpeed(now);
  flightUpdateMeasuredRPM(now);

  // Finish any attitude torque pulse before state logic decides
  // whether another command is needed.
  serviceTorquePulse(now);

  // ==========================================================
  // READ IR RECEIVERS
  // ==========================================================

  bool newLeft = false;
  bool newCentre = false;
  bool newRight = false;

  if (IrReceiver.decode()) {
    lastLeftIR = now;
    targetLastSeen = now;
    newLeft = true;
    IrReceiver.resume();
  }

  if (centreReceiver.decode()) {
    lastCentreIR = now;
    targetLastSeen = now;
    newCentre = true;
    centreReceiver.resume();
  }

  if (rightReceiver.decode()) {
    lastRightIR = now;
    targetLastSeen = now;
    newRight = true;
    rightReceiver.resume();
  }

  if (TUNED_CONTROL) {
    // Existing IRremote decoding remains active; augment with instantaneous
    // demodulated receiver LOW activity to catch short/panned 38 kHz bursts.
    if (digitalRead(LEFT_SENSOR) == LOW)   { lastLeftIR = now; targetLastSeen = now; }
    if (digitalRead(CENTRE_SENSOR) == LOW) { lastCentreIR = now; targetLastSeen = now; }
    if (digitalRead(RIGHT_SENSOR) == LOW)  { lastRightIR = now; targetLastSeen = now; }
  }

  bool L =
      lastLeftIR != 0 &&
      now - lastLeftIR <= IR_HOLD_MS;

  bool C =
      lastCentreIR != 0 &&
      now - lastCentreIR <= IR_HOLD_MS;

  bool R =
      lastRightIR != 0 &&
      now - lastRightIR <= IR_HOLD_MS;

  updateZone(
      calculateZone(L, C, R),
      now
  );
  flightCheckZone(flightIRZone(L, C, R), now);

  // In tuned mode, react to the side->centre *transition* once only.
  // This is not chassis yaw feedback; a real gyro would be needed for that.
  if (TUNED_CONTROL && state == FINE_ALIGN && currentZone == 0 &&
      (previousZone == -1 || previousZone == -2 ||
       previousZone == 1 || previousZone == 2) &&
      centreBrakedAtZoneTime != zoneSince &&
      torqueReady(now)) {
    centreBrakedAtZoneTime = zoneSince;
    centreStableSince = 0;
    int towardCentre = (previousZone < 0) ? -1 : 1;
    // Small impulse opposing the last towards-centre command.
    startTorquePulse(-towardCentre, 76, 24UL, 135UL, now, false);
  }


  if (currentZone < 0) {
    lastKnownSide = -1;
  }
  else if (currentZone > 0 && currentZone != 99) {
    lastKnownSide = +1;
  }

  // ==========================================================
  // SEARCHING
  // ==========================================================

  if (state == SEARCHING) {
    digitalWrite(LASER_PIN, LOW);

    centreStableSince = 0;
    firstSideTime = 0;
    firstCentreTime = 0;

    // Learn encoder direction while executing the known search command.
    learnLeftDriveEncoderSign();

    // Centre detection gets priority.
    if (C) {
      beginCaptureFromCentre(now);
    }
    else if (L && !R) {
      beginCaptureFromSide(-1, now);
    }
    else if (R && !L) {
      beginCaptureFromSide(+1, now);
    }
    else {
      runSearchDrive();
    }
  }

  // ==========================================================
  // CAPTURE APPROACH
  //
  // A side sensor has appeared. We are already applying the
  // opposite reaction-wheel torque BEFORE centre is reached.
  // We do NOT add more torque toward the target at this stage.
  // ==========================================================

  else if (state == CAPTURE_APPROACH) {
    digitalWrite(LASER_PIN, LOW);

    if (C && !centreSeenDuringCapture) {
      centreSeenDuringCapture = true;
      firstCentreTime = now;

      Serial.print(">>> CENTRE ENTERED after ");
      Serial.print(now - firstSideTime);
      Serial.println(" ms");
    }

    // Let the pre-brake pulse finish.
    if (torquePulseActive) {
      // Motor command is managed by the pulse manager.
    }
    else {
      // Preserve the new wheel momentum while observing the chassis.
      maintainWheelSpeed(now);

      if (centreSeenDuringCapture) {
        unsigned long crossingTime =
            firstCentreTime - firstSideTime;

        chooseCentreDamping(
            crossingTime,
            absFloat(wheelRPM)
        );

        Serial.print(">>> CENTRE DAMP power=");
        Serial.print(centreDampPower);
        Serial.print(" duration=");
        Serial.print(centreDampDuration);
        Serial.println(" ms");

        startTorquePulse(
            counterDirection,
            centreDampPower,
            centreDampDuration,
            55,
            now,
            true
        );

        centreStableSince = 0;
        captureObservationStart = now;

        changeState(CAPTURE_DAMPING, now);
      }
      else {
        // If the early pre-brake stopped the chassis just before the
        // centre sensor, that is preferable to blasting through.
        // Fine alignment will now walk the final small angle.
        if (now - stateStartTime >= 230) {
          centreStableSince = 0;
          changeState(FINE_ALIGN, now);
        }
      }
    }
  }

  // ==========================================================
  // CAPTURE DAMPING
  //
  // Centre has been reached. Continue removing chassis angular
  // velocity, then preserve the resulting wheel speed.
  //
  // Sensor sequence is used as a coarse velocity sign:
  //
  // approach LEFT  -> centre -> RIGHT = overshoot
  // approach RIGHT -> centre -> LEFT  = overshoot
  //
  // If it returns to the original side, braking was too strong.
  // ==========================================================

  else if (state == CAPTURE_DAMPING) {
    digitalWrite(LASER_PIN, LOW);

    if (C && firstCentreTime == 0) {
      firstCentreTime = now;
    }

    if (torquePulseActive) {
      // Pulse manager owns motor.
    }
    else {
      maintainWheelSpeed(now);

      int zoneSign = 0;

      if (currentZone < 0) zoneSign = -1;
      else if (currentZone > 0 && currentZone != 99) zoneSign = +1;

      // --------------------------------------------------------
      // Centre only: observe. Do not immediately "lock".
      // --------------------------------------------------------
      if (currentZone == 0) {
        if (centreStableSince == 0) {
          centreStableSince = now;
        }

        if (now - centreStableSince >= CAPTURE_CENTRE_STABLE_MS) {
          Serial.println(">>> Chassis captured in centre region");
          changeState(FINE_ALIGN, now);
        }
      }

      // --------------------------------------------------------
      // Opposite side after centre = still travelling through.
      // Add another short counter-torque impulse.
      // --------------------------------------------------------
      else if (
          zoneSign != 0 &&
          approachSide != 0 &&
          zoneSign == -approachSide &&
          zoneStableFor(20, now) &&
          torqueReady(now)
      ) {
        centreStableSince = 0;

        Serial.println(">>> OVERSHOOT still moving through centre");

        startTorquePulse(
            counterDirection,
            185,
            18,
            85,
            now,
            true
        );
      }

      // --------------------------------------------------------
      // Back on original approach side after centre = the last
      // counter-torque reversed the chassis too far.
      // Use a much smaller impulse back toward centre.
      // --------------------------------------------------------
      else if (
          centreSeenDuringCapture &&
          zoneSign != 0 &&
          approachSide != 0 &&
          zoneSign == approachSide &&
          zoneStableFor(28, now) &&
          torqueReady(now)
      ) {
        centreStableSince = 0;

        Serial.println(">>> Capture reversed slightly - gentle recovery");

        startTorquePulse(
            approachSide,
            95,
            7,
            120,
            now,
            true
        );
      }

      else {
        centreStableSince = 0;
      }

      if (now - stateStartTime >= CAPTURE_TIMEOUT_MS) {
        Serial.println(">>> Capture timeout -> fine alignment");
        changeState(FINE_ALIGN, now);
      }
    }
  }

  // ==========================================================
  // FINE ALIGN
  //
  // Small impulse + long observation. The old controller used
  // corrections that were too frequent, producing visible hunting.
  // ==========================================================

  else if (state == FINE_ALIGN) {
    digitalWrite(LASER_PIN, LOW);

    if (torquePulseActive) {
      // Pulse manager owns motor.
    }
    else {
      // Between pointing corrections preserve wheel speed so stored
      // flywheel momentum does not leak back into chassis motion.
      maintainWheelSpeed(now);

      // --------------------------------------------------------
      // CENTRE ONLY
      // --------------------------------------------------------
      if (currentZone == 0) {
        if (centreStableSince == 0) {
          centreStableSince = now;
        }

        if (
            now - centreStableSince >= LOCK_CONFIRM_MS &&
            !wheelHoldPulseActive
        ) {
          changeState(TARGET_LOCKED, now);
        }
      }

      // --------------------------------------------------------
      // LEFT + CENTRE / CENTRE + RIGHT
      // Very small correction.
      // --------------------------------------------------------
      else if (
          (currentZone == -1 || currentZone == +1) &&
          zoneStableFor(FINE_ZONE_CONFIRM_MS, now)
      ) {
        centreStableSince = 0;

        if (torqueReady(now)) {
          int direction = signOfInt(currentZone);

          startTorquePulse(
              direction,
              FINE_INNER_POWER,
              FINE_INNER_PULSE_MS,
              FINE_INNER_SETTLE_MS,
              now,
              true
          );
        }
      }

      // --------------------------------------------------------
      // LEFT only / RIGHT only
      // Still small, but slightly larger than near-centre pulse.
      // --------------------------------------------------------
      else if (
          (currentZone == -2 || currentZone == +2) &&
          zoneStableFor(FINE_ZONE_CONFIRM_MS, now)
      ) {
        centreStableSince = 0;

        if (torqueReady(now)) {
          int direction = signOfInt(currentZone);

          startTorquePulse(
              direction,
              FINE_OUTER_POWER,
              FINE_OUTER_PULSE_MS,
              FINE_OUTER_SETTLE_MS,
              now,
              true
          );
        }
      }

      else {
        centreStableSince = 0;

        // If signal flickers out briefly, maintain the wheel momentum
        // and WAIT rather than immediately kicking the chassis.
        //
        // Only after a longer loss do we gently move toward the last
        // known target side.
        if (
            currentZone == 99 &&
            targetLastSeen != 0 &&
            now - targetLastSeen > 180 &&
            now - targetLastSeen <= 950 &&
            lastKnownSide != 0 &&
            torqueReady(now)
        ) {
          startTorquePulse(
              lastKnownSide,
              48,
              5,
              160,
              now,
              true
          );
        }

        // Fully lost target -> return to bounded search.
        if (
            targetLastSeen != 0 &&
            now - targetLastSeen > 1200
        ) {
          disableWheelSpeedHold();
          approachSide = 0;
          firstSideTime = 0;
          firstCentreTime = 0;
          lastKnownSide = 0;

          motorCoast();

          changeState(SEARCHING, now);
        }
      }
    }
  }

  // ==========================================================
  // TARGET LOCKED
  //
  // Centre-only does NOT mean "switch motor off".
  // The flywheel speed achieved at capture is maintained so motor/
  // gearbox drag cannot slowly hand its momentum back to the chassis.
  // ==========================================================

  else if (state == TARGET_LOCKED) {

    if (torquePulseActive) {
      digitalWrite(LASER_PIN, LOW);
    }
    else {
      maintainWheelSpeed(now);

      // --------------------------------------------------------
      // PERFECT CENTRE
      // --------------------------------------------------------
      if (currentZone == 0) {
        if (centreStableSince == 0) {
          centreStableSince = now;
        }

        // Only fire while the sensor remains in the centre zone.
        digitalWrite(LASER_PIN, HIGH);
      }

      // --------------------------------------------------------
      // SMALL LEFT/RIGHT DRIFT NEAR CENTRE
      // --------------------------------------------------------
      else if (
          (currentZone == -1 || currentZone == +1) &&
          zoneStableFor(HOLD_ZONE_CONFIRM_MS, now)
      ) {
        digitalWrite(LASER_PIN, LOW);
        centreStableSince = 0;

        if (torqueReady(now)) {
          int direction = signOfInt(currentZone);

          startTorquePulse(
              direction,
              HOLD_INNER_POWER,
              HOLD_INNER_PULSE_MS,
              HOLD_INNER_SETTLE_MS,
              now,
              true
          );
        }
      }

      // --------------------------------------------------------
      // LARGER DRIFT
      // --------------------------------------------------------
      else if (
          (currentZone == -2 || currentZone == +2) &&
          zoneStableFor(HOLD_ZONE_CONFIRM_MS, now)
      ) {
        digitalWrite(LASER_PIN, LOW);
        centreStableSince = 0;

        if (torqueReady(now)) {
          int direction = signOfInt(currentZone);

          startTorquePulse(
              direction,
              HOLD_OUTER_POWER,
              HOLD_OUTER_PULSE_MS,
              HOLD_OUTER_SETTLE_MS,
              now,
              true
          );
        }
      }

      else {
        digitalWrite(LASER_PIN, LOW);
        centreStableSince = 0;

        if (
            targetLastSeen != 0 &&
            now - targetLastSeen > TARGET_LOST_MS
        ) {
          changeState(FINE_ALIGN, now);
        }
      }
    }
  }

  // Same metrics and EEPROM schema in both build variants.
  flightCheckLock(now);
  flightCheckLaser(now);
  flightSample(now);

  // ==========================================================
  // SERIAL TELEMETRY
  // ==========================================================

  // Keep V2's original serial telemetry intact for debugging.
  // The EEPROM log does not depend on Serial being connected.
  if (PRINT_LIVE_TELEMETRY && now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    Serial.print("STATE:");
    Serial.print(stateName(state));

    Serial.print(" | IR:");
    Serial.print(L ? "L" : "-");
    Serial.print(C ? "C" : "-");
    Serial.print(R ? "R" : "-");

    Serial.print(" | ZONE:");
    if (currentZone == 99) {
      Serial.print("NONE");
    } else {
      Serial.print(currentZone);
    }

    Serial.print(" | RPM:");
    Serial.print(wheelRPM, 1);

    Serial.print(" | HOLD_RPM:");
    if (holdWheelSpeedValid) {
      Serial.print(holdWheelRPM, 1);
    } else {
      Serial.print("---");
    }

    Serial.print(" | ENC:");
    Serial.print(encoderCount);

    Serial.print(" | WHEEL:");
    Serial.print(getWheelAngle(), 1);
    Serial.print("deg");

    Serial.print(" | APPROACH:");
    if (approachSide < 0) {
      Serial.print("LEFT");
    } else if (approachSide > 0) {
      Serial.print("RIGHT");
    } else {
      Serial.print("NONE");
    }

    Serial.print(" | PULSE:");
    Serial.print(torquePulseActive ? "ON" : "OFF");

    Serial.print(" | LASER:");
    Serial.println(
        digitalRead(LASER_PIN) ? "ON" : "OFF"
    );
  }
}

// ============================================================
// IRREMOTE MULTIPLE RECEIVER INTERRUPT HANDLER
// ============================================================

void UserIRReceiveTimerInterruptHandler() {
  centreReceiver.ReceiveInterruptHandler();
  rightReceiver.ReceiveInterruptHandler();
}
