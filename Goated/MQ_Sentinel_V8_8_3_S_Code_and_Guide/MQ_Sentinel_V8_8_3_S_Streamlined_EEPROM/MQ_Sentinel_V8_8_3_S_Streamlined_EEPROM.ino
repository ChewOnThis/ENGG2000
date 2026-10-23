/*
  MQ SENTINEL V8.8.3-S - STREAMLINED, ORIGINAL V8.8.3 BEHAVIOUR
  ATmega328P Nano / DRV8874 IN-IN / three active-LOW 38 kHz DEMODULATING receivers
  D2 LEFT, D3 CENTRE, D4 RIGHT, D6 IN1(PWM), D7 IN2,
  D8 laser, D9 encoder A, D10 encoder B, D13 status.
  EEPROM: USB send C to clear/arm NEXT boot. Power down, reinstall Nano,
  battery boot waits 5 seconds, starts 5-minute data log with motor off,
  and FIRST VALID IR FLASH starts tracking. Battery off -> USB send D.
  A reboot or USB reconnect preserves the stored log; no jumpers required.
  D=dump; C=clear/arm next boot; T=telemetry; X=safe stop;
  R=standby during an existing recording. Serial 115200 baud.
  T=telemetry displays raw electrical edges / validated pulses / rejection
  counts for L/C/R every 250 ms, in addition to the filtered mask.
  Centre-only is the DEFAULT laser lock; all-three counts as ambiguous until
  calibrated by a real target test. No software can reject a genuine reflected
  38-kHz pulse by intensity on these digital receivers.
  IMPORTANT: centre stability is a proxy for chassis stability, NOT measured yaw.
  Encoder reads flywheel speed, not chassis angular velocity. No gyro is fitted.
  All time/power constants below are starting points for physical calibration.
*/
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
// ---- Fixed, previously verified wiring/directions ----
const uint8_t PIN_IR[3] = {2, 3, 4};
const uint8_t PIN_IN1 = 6, PIN_IN2 = 7, PIN_LASER = 8;
const uint8_t PIN_ENC_A = 9, PIN_ENC_B = 10, PIN_LED = 13;
const int8_t BODY_CCW = -1, BODY_CW = 1;
const int8_t LEFT_TORQUE = BODY_CCW, RIGHT_TORQUE = BODY_CW;
const uint16_t ENCODER_CPR = 700; // 1x rising A edges per output revolution: verify
const uint8_t IR_MASK = _BV(PD2) | _BV(PD3) | _BV(PD4);
// ---- Tune after testing, avoid altering all values at once ----
const float SEEK_PWM = 174.0f;           // higher torque, soft-limited by RPM
const float SEEK_RPM_SOFT = 145.0f;
const float SEEK_RPM_HARD = 195.0f;
const float SEEK_MIN_PWM = 108.0f;
const float COARSE_PWM = 148.0f; // outer-only: seek with reduced torque
const float FINE_PWM = 100.0f;  // short corrections for centre+one side
const float COUNTER_PWM = 70.0f; // gentler counter-torque on first centre entry
const uint16_t COUNTER_MS = 145; // limit braking impulse and reduce overcorrection
// Digital 38-kHz receivers have no analogue sensitivity setting. These
// thresholds make the FILTER less permissive; verify against your IR beacon.
const uint16_t IR_FRESH_MS = 65;           // down from 100 ms
const uint16_t IR_MIN_PULSE_US = 500;       // completed marks only
const uint16_t IR_MAX_PULSE_US = 20000;    // reject long stuck-low intervals
const uint16_t IR_CONFIRM_WINDOW_MS = 160; // two valid marks must arrive this close
const uint8_t IR_REQUIRED_PULSES = 2;
const uint16_t MICRO_SIDE_STABLE_MS = 70; // overlapping side must persist
const uint16_t MICRO_PULSE_MS = 145;      // smoothing envelope needs real time to respond
const uint16_t MICRO_COOLDOWN_MS = 240;   // zero-demand coast between fine pulses
const uint16_t MIN_TARGET_LOST_MS = 230;
const uint16_t SETTLE_MS = 550;        // valid centre region dwell before laser
const uint16_t QUIET_MOTOR_MS = 180;  // reaction wheel speed does NOT equal chassis yaw
const uint16_t FIRE_MS = 2100;         // full 2.1 s demonstration requirement
const uint16_t CENTRE_DROPOUT_GRACE_MS = 80; // ignore brief receiver flicker, not prolonged target loss
const uint16_t POST_SHOT_CLEAR_MS = 180;
const uint16_t TRACE_MS = 250;
// PWM envelope is a critically damped second-order low-pass system.
// It eases IN and OUT, and crosses zero before any motor reversal.
// Higher response gives snappier movement but more mechanical jerk.
const float MOTOR_OMEGA = 0.0125f;      // 1/ms (roughly 80ms time constant)
const float MOTOR_MAX_RATE = 0.76f;    // PWM units/ms (max slew)
const uint16_t MOTOR_TICK_MS = 5;
const uint16_t ZERO_CROSS_GUARD_MS = 18;
// ---- IR edges (interrupt captures brief pulses while main loop is busy) ----
volatile uint8_t irPreviousPort = 0;
volatile uint32_t irFallMicros[3] = {0,0,0};
volatile uint8_t irCompleted[3] = {0,0,0};
volatile uint16_t irRawFalls[3] = {0,0,0};
volatile uint16_t irAccepted[3] = {0,0,0};
volatile uint16_t irRejected[3] = {0,0,0};
volatile uint8_t irLowValid[3] = {0,0,0};
volatile long encoderTicks = 0;
volatile unsigned long encoderEdges = 0;
volatile uint8_t previousEncoderA = 0;
uint32_t lastIR[3] = {0,0,0};
uint32_t firstEvidenceAt[3] = {0,0,0};
uint32_t lastEvidenceAt[3] = {0,0,0};
uint8_t evidenceCount[3] = {0,0,0};
uint8_t rawIRMask = 0; // instantaneous electrical levels, for diagnostics
uint8_t sensorMask = 0;
unsigned long centreCandidateSince = 0;
unsigned long centreLastSeenAt = 0;
float wheelRPM = 0;
long previousEncoder = 0;
unsigned long speedSampleAt = 0;
ISR(PCINT0_vect) { // D9/PB1 encoder A; D10/PB2 encoder B
  uint8_t a = (PINB & _BV(PB1)) ? 1 : 0;
  if (a && !previousEncoderA) {
    encoderTicks += ((PINB & _BV(PB2)) ? -1 : 1);
    encoderEdges++;
  }
  previousEncoderA = a;
}
ISR(PCINT2_vect) { // D2, D3, D4 active-low demodulated envelope
  uint8_t port = PIND & IR_MASK;
  uint8_t changed = (port ^ irPreviousPort) & IR_MASK;
  irPreviousPort = port;
  if (!changed) return;
  uint32_t nowUs = micros(); // 32-bit timestamp also rejects long stuck-low signals
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t bit = (uint8_t)(1 << (i + 2));
    if (!(changed & bit)) continue;
    if (!(port & bit)) {
      irFallMicros[i] = nowUs;
      irLowValid[i] = 1;
      irRawFalls[i]++;
    } else if (irLowValid[i]) {
      uint32_t width = nowUs - irFallMicros[i];
      irLowValid[i] = 0;
      if (width >= IR_MIN_PULSE_US && width <= IR_MAX_PULSE_US) {
        if (irCompleted[i] != 255) irCompleted[i]++;
        irAccepted[i]++;
      } else {
        irRejected[i]++;
      }
    }
  }
}
static bool hasLeft(uint8_t m) { return m & 1; }
static bool hasCentre(uint8_t m) { return m & 2; }
static bool hasRight(uint8_t m) { return m & 4; }
// IMPORTANT: All-three (111) is not proof of boresight: it can be caused by
// wide receiver fields of view or reflections. Only 010 is a default lock.
// Enable 111 ONLY after verifying laser/target coincidence physically.
const bool ACCEPT_ALL_THREE_AS_CENTRED = false;
static bool isCentred(uint8_t m) {
  return m==2 || (ACCEPT_ALL_THREE_AS_CENTRED && m==7);
}
void readSensors(unsigned long now) {
  uint8_t pulses[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i=0; i<3; i++) {
      pulses[i] = irCompleted[i];
      irCompleted[i] = 0;
    }
  }
  uint8_t next=0;
  rawIRMask=0;
  for (uint8_t i=0; i<3; i++) {
    const uint8_t bit = (uint8_t)(1U << i);
    // Count only completed validated receiver marks. A sustained LOW input,
    // a single glitch, and an electrically floating input do not by
    // themselves declare target acquisition.
    if (digitalRead(PIN_IR[i]) == LOW) rawIRMask |= bit;
    if (pulses[i]) {
      if (!evidenceCount[i] ||
          (now - firstEvidenceAt[i] > IR_CONFIRM_WINDOW_MS)) {
        firstEvidenceAt[i] = now;
        evidenceCount[i] = 0;
      }
      evidenceCount[i] = (uint8_t)min((uint16_t)IR_REQUIRED_PULSES,
                    (uint16_t)(evidenceCount[i] + pulses[i]));
      lastEvidenceAt[i] = now;
      if (evidenceCount[i] >= IR_REQUIRED_PULSES) {
        lastIR[i] = now;
        // Keep the evidence window rolling while genuine marks continue.
        firstEvidenceAt[i] = now;
      }
    } else if (evidenceCount[i] &&
               now - lastEvidenceAt[i] > IR_CONFIRM_WINDOW_MS) {
      evidenceCount[i] = 0;
    }
    if (lastIR[i] && now - lastIR[i] <= IR_FRESH_MS) next |= bit;
  }
  sensorMask = next;
  // 010 centre candidate must persist; brief AGC gaps are tolerated, but
  // an unrelated side / all-three combination cannot by itself arm laser.
  if (isCentred(next)) {
    if (!centreCandidateSince) centreCandidateSince = now;
    centreLastSeenAt = now;
  } else if (!centreLastSeenAt ||
             now - centreLastSeenAt > CENTRE_DROPOUT_GRACE_MS) {
    centreCandidateSince = 0;
  }
}
void readWheel(unsigned long now) {
  if (now - speedSampleAt < 75UL) return;
  long count;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = encoderTicks; }
  unsigned long dt = now - speedSampleAt;
  wheelRPM = (count - previousEncoder) * 60000.0f / (ENCODER_CPR * (float)dt);
  previousEncoder = count;
  speedSampleAt = now;
}
// ---- Motor driver: use existing verified DRV8874 IN/IN polarity ----
float requestedPWM = 0, motorPWM = 0, motorVelocity = 0;
unsigned long lastMotorAt = 0, zeroCrossSince = 0;
int8_t lastPhysicalPolarity = 0;
int16_t lastAppliedPWM = 0;
void releaseMotor() {
  analogWrite(PIN_IN1, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  lastAppliedPWM = 0;
}
void setMotorRaw(int16_t signedPower) {
  int power = constrain(abs(signedPower), 0, 255);
  if (power < 8) { releaseMotor(); return; }
  if (signedPower < 0) { // body CCW, wheel clockwise
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_IN1, 255 - power);
  } else { // body CW, wheel counterclockwise
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_IN1, power);
  }
  lastAppliedPWM = signedPower;
}
void requestMotor(float signedTarget) {
  requestedPWM = constrain(signedTarget, -205.0f, 205.0f);
}
void emergencyMotorOff() {
  requestedPWM = motorPWM = motorVelocity = 0; lastPhysicalPolarity = 0;
  releaseMotor();
}
void serviceMotor(unsigned long now) {
  if (now - lastMotorAt < MOTOR_TICK_MS) return;
  float dt = (float)min(now - lastMotorAt, (unsigned long)25);
  lastMotorAt = now;
  // Critically damped acceleration profile, plus fixed rate bound.
  float error = requestedPWM - motorPWM;
  float accel = MOTOR_OMEGA * MOTOR_OMEGA * error - 2.0f * MOTOR_OMEGA * motorVelocity;
  motorVelocity = constrain(motorVelocity + accel * dt,
                            -MOTOR_MAX_RATE, MOTOR_MAX_RATE);
  float next = motorPWM + motorVelocity * dt;
  // Prevent numeric overshoot and retain the smooth stop at the destination.
  if ((requestedPWM - motorPWM) * (requestedPWM - next) <= 0) {
    next = requestedPWM;
    motorVelocity = 0;
  }
  motorPWM = next;
  int16_t output = (int16_t)motorPWM;
  int8_t sign = output > 7 ? 1 : (output < -7 ? -1 : 0);
  if (!sign) { releaseMotor(); return; }
  // Physical direction never flips from one non-zero polarity directly to
  // the other. All reversals are zero-crossing, and require a guard delay.
  if (lastPhysicalPolarity && sign != lastPhysicalPolarity) {
    lastPhysicalPolarity = 0;
    zeroCrossSince = now;
    releaseMotor();
    return;
  }
  if (!lastPhysicalPolarity) {
    if (zeroCrossSince == 0) zeroCrossSince = now;
    if (now - zeroCrossSince < ZERO_CROSS_GUARD_MS) {
      releaseMotor(); return;
    }
    lastPhysicalPolarity = sign;
  }
  setMotorRaw(output);
}
// ---- Flight recorder: V2-style arm/boot/dump, retain V6/1021-byte layout ----
const uint16_t LOG_MAGIC = 0x86A6;
const uint8_t LOG_VERSION = 6;
const uint32_t RUN_LIMIT_MS = 300000UL;
const uint16_t SNAPSHOT_LIMIT = 300;
const uint8_t EVENT_CAPACITY = 52;
enum EventType : uint8_t {
  EVT_BOOT=1, EVT_TARGET_SEEN, EVT_TARGET_LOST, EVT_CENTRE_REACHED,
  EVT_ALIGNED, EVT_DIRECTION_CHANGE, EVT_BRAKE, EVT_LASER_ON,
  EVT_LASER_ABORT, EVT_SHOT_COMPLETE, EVT_SEEK_START,
  EVT_ZONE_CHANGE, EVT_CENTRE_LOST, EVT_CENTRE_ACQUIRED,
  EVT_FIRST_FLASH, EVT_MICRO_ADJUST
};
struct __attribute__((packed)) FlightHeader {
  uint16_t magic;
  uint8_t version, armed, phase, eventHead, eventCount;
  uint16_t snapshotCount, targetSeenCount, targetLostCount, zoneTransitions;
  uint16_t allThreeArrivals, directionChanges, confirmedAlignments;
  uint16_t shotAttempts, shotCompletions, shotAborts;
  uint16_t maxLaserOnMs, peakRPM10, acquireCount;
  uint32_t totalAcquireMs, totalLaserOnMs, encoderPulses;
  uint32_t firstDetectionMs, firstAlignmentMs, lastElapsedMs;
};
struct __attribute__((packed)) FlightEventRecord {
  uint16_t t10;
  uint8_t kind, zoneStateDir;
  int8_t rpm2;
  uint16_t aux;
};
struct __attribute__((packed)) FlightSnapshot {
  uint8_t zoneStateDir;
  int8_t rpm2;
};
const int ADDR_HEADER = 0;
const int ADDR_EVENTS = sizeof(FlightHeader);
const int ADDR_SNAPSHOTS = ADDR_EVENTS + EVENT_CAPACITY * sizeof(FlightEventRecord);
const int STORAGE_BYTES = ADDR_SNAPSHOTS + SNAPSHOT_LIMIT * sizeof(FlightSnapshot);
static_assert(STORAGE_BYTES <= 1024, "EEPROM overflow");
FlightHeader logHeader;
uint32_t runStart = 0, snapshotDue = 0, acquireStarted = 0;
uint8_t lastLoggedZone = 0;
unsigned long lastZoneLogAt = 0;
bool stopAtEnd = false;
enum Mode : uint8_t { WAIT_FLASH, SEEK, TRACK, SETTLE, FIRE, HALT };
Mode mode = WAIT_FLASH;
unsigned long laserStarted = 0;
int8_t seekDirection = BODY_CW, lastCorrectionDirection = 0;
unsigned long counterStarted = 0;
unsigned long fineZoneSince = 0, microPulseAt = 0, microCooldownUntil = 0;
int8_t fineZoneDirection = 0;
bool microPulseActive = false;
int8_t counterDirection = 0;
unsigned long clearStarted = 0, missingStarted = 0;
bool requireClearAfterShot = false, telemetryEnabled = false;
unsigned long telemetryLastAt = 0;
// written per main-loop pass (\~3.3ms max), rather than pausing for the whole
const uint8_t WRITE_QUEUE_SIZE = 160;
uint16_t writeAddress[WRITE_QUEUE_SIZE];
uint8_t writeValue[WRITE_QUEUE_SIZE];
uint8_t writeHead=0, writeTail=0, writeCount=0;
void serviceEEPROM() {
  if (!writeCount) return;
  EEPROM.update(writeAddress[writeTail],writeValue[writeTail]);
  writeTail=(uint8_t)((writeTail+1)%WRITE_QUEUE_SIZE); writeCount--;
}
void flushEEPROM() { while(writeCount) serviceEEPROM(); }
void queueEEPROM(int address, const void *source, uint8_t count) {
  const uint8_t* bytes=(const uint8_t*)source;
  for(uint8_t i=0;i<count;i++) {
    // Extreme log overload: synchronous fallback prefers valid data to loss.
    if(writeCount==WRITE_QUEUE_SIZE) flushEEPROM();
    writeAddress[writeHead]=(uint16_t)(address+i); writeValue[writeHead]=bytes[i];
    writeHead=(uint8_t)((writeHead+1)%WRITE_QUEUE_SIZE); writeCount++;
  }
}
void queueHeader() { queueEEPROM(ADDR_HEADER,&logHeader,sizeof(logHeader)); }
bool recording() { return logHeader.magic==LOG_MAGIC && logHeader.phase==1; }
void saveHeader() { flushEEPROM(); EEPROM.put(ADDR_HEADER, logHeader); }
void resetHeader(uint8_t phase, uint8_t armed) {
  memset(&logHeader, 0, sizeof(logHeader));
  logHeader.magic=LOG_MAGIC; logHeader.version=LOG_VERSION;
  logHeader.armed=armed; logHeader.phase=phase;
  logHeader.firstDetectionMs=0xFFFFFFFFUL; logHeader.firstAlignmentMs=0xFFFFFFFFUL;
}
void armRecorder() {
  resetHeader(0,1);
  saveHeader();
}
uint32_t elapsedFlight(unsigned long now) { return now-runStart; }
uint8_t packedStatus() {
  // Same V6 bit positions: 3-bit zone, 3-bit state, 2-bit direction
  uint8_t dir = lastAppliedPWM < 0 ? 1 : (lastAppliedPWM > 0 ? 2 : 0);
  return (sensorMask & 7) | (((uint8_t)mode & 7) << 3) | (dir << 6);
}
int8_t rpmByte() {
  return (int8_t)constrain((int)(wheelRPM/2.0f), -128, 127);
}
void logEvent(uint8_t kind, unsigned long now, uint16_t aux=0) {
  if (!recording()) return;
  FlightEventRecord event;
  event.t10=(uint16_t)min(elapsedFlight(now)/10UL, 65535UL); event.kind=kind;
  event.zoneStateDir=packedStatus(); event.rpm2=rpmByte();
  event.aux=aux;
  queueEEPROM(ADDR_EVENTS + logHeader.eventHead*sizeof(event), &event, sizeof(event));
  logHeader.eventHead=(logHeader.eventHead+1)%EVENT_CAPACITY;
  if (logHeader.eventCount<EVENT_CAPACITY) logHeader.eventCount++;
}
void startRecording(unsigned long now) {
  // Same arm-next-boot approach as the proven V2 recorder.
  // Time zero is battery boot, so acquisition latency is measurable.
  if (logHeader.armed!=1 || logHeader.phase!=0) return;
  resetHeader(1,0);
  runStart=now;
  snapshotDue=now+1000UL; lastLoggedZone=0;
  acquireStarted=0;
  saveHeader();
  logEvent(EVT_BOOT,now);
  flushEEPROM(); // ensure BOOT is on nonvolatile storage
}

void logSensorTransition(unsigned long now) {
  if (!recording() || lastLoggedZone==sensorMask) return;
  uint8_t old = lastLoggedZone;
  lastLoggedZone=sensorMask; logHeader.zoneTransitions++;
  if (!old && sensorMask) {
    logHeader.targetSeenCount++;
    if (logHeader.firstDetectionMs==0xFFFFFFFFUL)
      logHeader.firstDetectionMs=elapsedFlight(now);
    if (!acquireStarted) acquireStarted=now;
    logEvent(EVT_TARGET_SEEN,now,(uint16_t)abs(lastAppliedPWM));
  } else if (old && !sensorMask) {
    logHeader.targetLostCount++;
    // Retain acquisition start through brief losses, as in V2.
    logEvent(EVT_TARGET_LOST,now);
  }
  if (isCentred(sensorMask) && !isCentred(old)) {
    // Count tentative sensor alignment separately from confirmed laser lock.
    logHeader.allThreeArrivals++;
    logEvent(EVT_CENTRE_ACQUIRED,now,(uint16_t)abs(lastAppliedPWM));
  } else if (isCentred(old) && !isCentred(sensorMask)) {
    logEvent(EVT_CENTRE_LOST,now,(uint16_t)abs(lastAppliedPWM));
  } else if (now-lastZoneLogAt>=220UL && old && sensorMask) {
    logEvent(EVT_ZONE_CHANGE,now,(uint16_t)abs(lastAppliedPWM));
    lastZoneLogAt=now;
  }
}
void finishRecording(unsigned long now) {
  if (!recording()) return;
  logHeader.phase=2; logHeader.lastElapsedMs=min(elapsedFlight(now),RUN_LIMIT_MS);
  saveHeader();
  stopAtEnd=true;
}
void sampleFlight(unsigned long now) {
  if (!recording()) return;
  if ((long)(now-snapshotDue)<0) return;
  if (logHeader.snapshotCount<SNAPSHOT_LIMIT) {
    FlightSnapshot sample={packedStatus(),rpmByte()};
    queueEEPROM(ADDR_SNAPSHOTS+logHeader.snapshotCount*sizeof(sample),&sample,sizeof(sample));
    logHeader.snapshotCount++; logHeader.lastElapsedMs=elapsedFlight(now);
    float peak=abs(wheelRPM)*10.0f;
    if (peak>logHeader.peakRPM10)
      logHeader.peakRPM10=(uint16_t)min(peak,65535.0f);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { logHeader.encoderPulses=encoderEdges; }
    // Queue a full checkpoint without blocking the motor controller.
    queueHeader();
  }
  snapshotDue += 1000UL;
  if (elapsedFlight(now)>=RUN_LIMIT_MS) finishRecording(now);
}
const __FlashStringHelper *eventName(uint8_t kind) {
  switch(kind) {
    case EVT_BOOT:return F("BOOT");
    case EVT_TARGET_SEEN:return F("TARGET_SEEN");
    case EVT_TARGET_LOST:return F("TARGET_LOST");
    case EVT_CENTRE_REACHED:return F("CENTRE_REACHED");
    case EVT_ALIGNED:return F("ALIGNED");
    case EVT_DIRECTION_CHANGE:return F("DIRECTION_CHANGE");
    case EVT_BRAKE:return F("COUNTER_TORQUE");
    case EVT_LASER_ON:return F("LASER_ON");
    case EVT_LASER_ABORT:return F("LASER_ABORT");
    case EVT_SHOT_COMPLETE:return F("SHOT_COMPLETE");
    case EVT_SEEK_START:return F("SEEK_START");
    case EVT_ZONE_CHANGE:return F("ZONE_CHANGE");
    case EVT_CENTRE_LOST:return F("CENTRE_LOST");
    case EVT_CENTRE_ACQUIRED:return F("CENTRE_ACQUIRED");
    case EVT_FIRST_FLASH:return F("FIRST_FLASH");
    case EVT_MICRO_ADJUST:return F("MICRO_ADJUST");
    default:return F("OTHER");
  }
}
void printStatusFields(uint8_t packed, int8_t rpm2) {
  Serial.print(packed&7);Serial.print(','); Serial.print((packed>>3)&7);Serial.print(',');
  Serial.print((packed>>6)&3);Serial.print(','); Serial.print((int)rpm2*2);
}
void dumpRecorder() {
  flushEEPROM();
  EEPROM.get(ADDR_HEADER,logHeader);
  Serial.println(F("BEGIN SENTINEL V8.8.3-S / EEPROM FORMAT V6"));
  if (logHeader.magic!=LOG_MAGIC || logHeader.version!=LOG_VERSION) {
    Serial.println(F("NO_VALID_LOG"));return;
  }
#define PRINT_M(name,field) do { Serial.print(F("" name ","));Serial.println(logHeader.field); } while(0)
  PRINT_M("status,phase",phase);
  PRINT_M("status,armed",armed);
  Serial.print(F("status,eeprom_bytes,"));Serial.println(STORAGE_BYTES);
  PRINT_M("status,snapshot_count",snapshotCount);
  PRINT_M("status,recent_event_count",eventCount);
  PRINT_M("metric,target_seen",targetSeenCount);
  PRINT_M("metric,target_lost",targetLostCount);
  PRINT_M("metric,zone_transitions",zoneTransitions);
  PRINT_M("metric,centre_arrivals",allThreeArrivals);
  PRINT_M("metric,direction_changes",directionChanges);
  PRINT_M("metric,confirmed_alignments",confirmedAlignments);
  PRINT_M("metric,shot_attempts",shotAttempts);
  PRINT_M("metric,shot_completions",shotCompletions);
  PRINT_M("metric,shot_aborts",shotAborts);
  PRINT_M("metric,max_laser_on_ms",maxLaserOnMs);
  PRINT_M("metric,total_laser_on_ms",totalLaserOnMs);
  PRINT_M("metric,encoder_pulses",encoderPulses);
  PRINT_M("metric,peak_wheel_rpm_x10",peakRPM10);
  PRINT_M("metric,acquire_count",acquireCount);
  PRINT_M("metric,total_acquire_ms",totalAcquireMs);
  PRINT_M("metric,first_detection_ms",firstDetectionMs);
  PRINT_M("metric,first_alignment_ms",firstAlignmentMs);
  PRINT_M("metric,last_elapsed_ms",lastElapsedMs);
#undef PRINT_M
  Serial.println(F("section,event_records_latest_52"));
  Serial.println(F("time_ms,event,zone,state,direction,rpm_approx,aux_pwm_or_laser_ms"));
  uint8_t n=min(logHeader.eventCount,EVENT_CAPACITY);
  uint8_t first=n==EVENT_CAPACITY ? logHeader.eventHead : 0;
  for (uint8_t i=0;i<n;i++) {
    FlightEventRecord e;
    EEPROM.get(ADDR_EVENTS+((first+i)%EVENT_CAPACITY)*sizeof(e),e);
    Serial.print((uint32_t)e.t10*10UL);Serial.print(','); Serial.print(eventName(e.kind));Serial.print(',');
    printStatusFields(e.zoneStateDir,e.rpm2); Serial.print(',');Serial.println(e.aux);
  }
  Serial.println(F("section,snapshots_1_per_second")); Serial.println(F("time_ms,zone,state,direction,rpm_approx"));
  uint16_t nSamples=min(logHeader.snapshotCount,SNAPSHOT_LIMIT);
  for (uint16_t i=0;i<nSamples;i++) {
    FlightSnapshot s;
    EEPROM.get(ADDR_SNAPSHOTS+i*sizeof(s),s);
    Serial.print((uint32_t)(i+1)*1000UL);Serial.print(','); printStatusFields(s.zoneStateDir,s.rpm2);
    Serial.println();
  }
  Serial.println(F("END SENTINEL V8.8.3-S"));
}
// ---- State machine / fine correction ----
// Direct state assignments; the old state timestamp was never read.
void enterStandby(unsigned long now) {
  digitalWrite(PIN_LASER,LOW);
  emergencyMotorOff();
  lastCorrectionDirection=0; clearStarted=missingStarted=0;
  counterStarted=0; fineZoneSince=microPulseAt=microCooldownUntil=0;
  fineZoneDirection=0; microPulseActive=false;
  mode=WAIT_FLASH;
}
void enterSeek(unsigned long now) {
  // Every true new SEEK activation reverses the next search direction.
  seekDirection=-seekDirection;
  counterStarted=0; lastCorrectionDirection=0;
  missingStarted=0;
  microPulseActive=false; fineZoneDirection=0;
  requestMotor(0); // envelope will ramp into the new sign
  mode=SEEK;
  logEvent(EVT_SEEK_START,now,seekDirection<0?1:2);
}
void enterTrack(unsigned long now) {
  Mode fromMode=mode;
  mode=TRACK;
  missingStarted=0;
  counterStarted=0; microPulseActive=false;
  fineZoneDirection=0;
  // Preserve seek direction as the previous source of body torque, for braking.
  lastCorrectionDirection=(fromMode==SEEK) ? seekDirection : 0;
}
float seekPower() {
  float rpm=abs(wheelRPM);
  if (rpm<=SEEK_RPM_SOFT) return SEEK_PWM;
  if (rpm>=SEEK_RPM_HARD) return SEEK_MIN_PWM;
  return SEEK_PWM-(SEEK_PWM-SEEK_MIN_PWM)*
    ((rpm-SEEK_RPM_SOFT)/(SEEK_RPM_HARD-SEEK_RPM_SOFT));
}
int8_t zoneCorrection(uint8_t mask) {
  // Centre with side overlap is NOT a confirmed bullseye. Small impulses
  // bring the receiver back toward a centre-only detection.
  if (mask==1 || mask==3) return LEFT_TORQUE;
  if (mask==4 || mask==6) return RIGHT_TORQUE;
  // 101 (outer sensors but no centre) and 111 are ambiguous with digital
  // receivers; never chase reflected or tie-break signals by timestamp.
  return 0;
}
void driveTracking(unsigned long now) {
  int8_t correction=zoneCorrection(sensorMask);
  bool fineOverlap = sensorMask==3 || sensorMask==6;
  if (fineOverlap) {
    if (fineZoneDirection != correction) {
      fineZoneDirection=correction; fineZoneSince=now;
      microPulseActive=false; microCooldownUntil=now+MICRO_SIDE_STABLE_MS;
      requestMotor(0);
      return;
    }
    if (now-fineZoneSince < MICRO_SIDE_STABLE_MS) {
      requestMotor(0); return;
    }
    if (!microPulseActive && (long)(now-microCooldownUntil)>=0) {
      microPulseActive=true; microPulseAt=now;
      if (lastCorrectionDirection && correction!=lastCorrectionDirection) {
        if (recording()) logHeader.directionChanges++;
        logEvent(EVT_DIRECTION_CHANGE,now,(uint16_t)FINE_PWM);
      }
      lastCorrectionDirection=correction;
      logEvent(EVT_MICRO_ADJUST,now,(uint16_t)FINE_PWM);
    }
    if (microPulseActive && now-microPulseAt<MICRO_PULSE_MS) {
      counterStarted=0;
      requestMotor(correction*FINE_PWM);
    } else {
      if (microPulseActive) {
        microPulseActive=false; microCooldownUntil=now+MICRO_COOLDOWN_MS;
      }
      requestMotor(0); // let envelope and chassis settle between impulses
    }
    return;
  }
  microPulseActive=false; fineZoneDirection=0;
  if (correction) {
    if (lastCorrectionDirection && correction!=lastCorrectionDirection) {
      if (recording()) logHeader.directionChanges++;
      logEvent(EVT_DIRECTION_CHANGE,now,(uint16_t)COARSE_PWM);
    }
    lastCorrectionDirection=correction; counterStarted=0;
    requestMotor(correction*COARSE_PWM);
    return;
  }
  if (isCentred(sensorMask)) {
    // Apply one modest counter impulse only on entering confirmed centre.
    if (lastCorrectionDirection && !counterStarted) {
      counterStarted=now?now:1; counterDirection=(int8_t)-lastCorrectionDirection;
      logEvent(EVT_BRAKE,now,(uint16_t)COUNTER_PWM);
      lastCorrectionDirection=0; // do not trigger a new brake every loop
    }
    if (counterStarted && now-counterStarted<COUNTER_MS) {
      requestMotor(counterDirection*COUNTER_PWM);
    } else {
      requestMotor(0);
    }
    return;
  }
  // 000, 101 or 111 without calibrated all-three acceptance: coast.
  counterStarted=0;
  requestMotor(0);
}
void saveHeaderIfRecording();

// Record a completed or aborted activation using the same duration formula.
void endLaser(unsigned long now, bool completed) {
  uint16_t duration=(uint16_t)min(now-laserStarted,65535UL);
  digitalWrite(PIN_LASER,LOW);
  if (recording()) {
    if (completed) logHeader.shotCompletions++;
    else logHeader.shotAborts++;
    logHeader.totalLaserOnMs+=now-laserStarted; logHeader.maxLaserOnMs=max(logHeader.maxLaserOnMs,duration);
  }
  logEvent(completed?EVT_SHOT_COMPLETE:EVT_LASER_ABORT,now,duration);
  saveHeaderIfRecording();
}

void serviceController(unsigned long now) {
  if (mode==HALT) { digitalWrite(PIN_LASER,LOW); emergencyMotorOff();return; }
  if (mode==WAIT_FLASH) {
    digitalWrite(PIN_LASER,LOW);
    requestMotor(0);
    if (sensorMask) {
        // Logger has been recording since battery boot. This marks first
        // flash accurately instead of resetting the data clock here.
        logEvent(EVT_FIRST_FLASH,now);
        saveHeaderIfRecording();
      enterTrack(now);
    }
    return;
  }
  if (mode==SEEK) {
    digitalWrite(PIN_LASER,LOW);
    // Once a target has been fired, first leave its beam before reacquiring.
    // Search motor keeps turning during this period: NO WAIT_CLEAR delay.
    if (requireClearAfterShot) {
      if (sensorMask==0) {
        if (!clearStarted) clearStarted=now;
        if (now-clearStarted>=POST_SHOT_CLEAR_MS) {
          requireClearAfterShot=false; clearStarted=0;
        }
      } else clearStarted=0;
    }
    if (!requireClearAfterShot && sensorMask) {
      enterTrack(now);
      return;
    }
    requestMotor(seekDirection*seekPower());
    return;
  }
  if (mode==TRACK) {
    digitalWrite(PIN_LASER,LOW);
    if (!sensorMask) {
      requestMotor(0);
      if (!missingStarted) missingStarted=now;
      if (now-missingStarted>=MIN_TARGET_LOST_MS) enterSeek(now);
      return;
    }
    missingStarted=0;
    driveTracking(now);
    if (isCentred(sensorMask) && centreCandidateSince && now-centreCandidateSince >= 40UL) {
      mode=SETTLE;
    }
    return;
  }
  if (mode==SETTLE) {
    digitalWrite(PIN_LASER,LOW);
    if (!isCentred(sensorMask)) {
      // Brief missing 38kHz bursts should not restart the whole settle timer.
      if (centreLastSeenAt && now-centreLastSeenAt <= CENTRE_DROPOUT_GRACE_MS) {
        requestMotor(0);
        return;
      }
      mode=TRACK; counterStarted=0;
      driveTracking(now);
      return;
    }

    driveTracking(now);
    // Only calibrated alignment regions qualify; default 010. A stable
    // detector zone and motor at rest are required before firing.
    // Flywheel RPM does not prove chassis yaw, so it is NOT a firing veto.
    bool centreStable=centreCandidateSince &&
                      now-centreCandidateSince>=SETTLE_MS;
    bool brakeFinished=(!counterStarted || now-counterStarted>=COUNTER_MS);
    bool motorQuiet=abs(motorPWM)<8.0f && abs(motorVelocity)<0.08f &&
                    lastAppliedPWM==0;
    static unsigned long motorQuietSince=0;
    if (motorQuiet && brakeFinished) {
      if (!motorQuietSince) motorQuietSince=now;
    } else motorQuietSince=0;
    if (centreStable && motorQuietSince &&
        now-motorQuietSince>=QUIET_MOTOR_MS) {
      if (recording()) {
        logHeader.confirmedAlignments++;
        if (logHeader.firstAlignmentMs==0xFFFFFFFFUL)
          logHeader.firstAlignmentMs=elapsedFlight(now);
        if (acquireStarted) {
          logHeader.acquireCount++; logHeader.totalAcquireMs+=now-acquireStarted;
          acquireStarted=0;
        }
      }
      logEvent(EVT_ALIGNED,now,(uint16_t)abs(lastAppliedPWM));
      requestMotor(0);
      mode=FIRE;
      laserStarted=now;
      digitalWrite(PIN_LASER,HIGH);
      if (recording()) logHeader.shotAttempts++;
      logEvent(EVT_LASER_ON,now);
    }
    return;
  }
  if (mode==FIRE) {
    requestMotor(0);
    // A brief IR dropout is tolerated, but sustained loss aborts the shot.
    if (!isCentred(sensorMask) &&
        (!centreLastSeenAt || now-centreLastSeenAt>CENTRE_DROPOUT_GRACE_MS)) {
      endLaser(now,false);
      mode=TRACK; counterStarted=0;
      return;
    }
    if (now-laserStarted>=FIRE_MS) {
      endLaser(now,true);
      requireClearAfterShot=true; clearStarted=0;
      enterSeek(now);
    }
  }
}
void saveHeaderIfRecording() { if (recording()) queueHeader(); }
void showTelemetry(unsigned long now) {
  if (!telemetryEnabled || now-telemetryLastAt<TRACE_MS) return;
  telemetryLastAt=now;
  // Raw electrical falling edges, valid mark lengths and rejected marks are
  // distinct from debounced/filtered detection mask. Values are per interval.
  static uint16_t prevRaw[3]={0,0,0}, prevValid[3]={0,0,0}, prevReject[3]={0,0,0};
  uint16_t raw[3], valid[3], rejected[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(uint8_t i=0;i<3;i++) {
      raw[i]=irRawFalls[i];
      valid[i]=irAccepted[i];
      rejected[i]=irRejected[i];
    }
  }
  Serial.print(F("t="));Serial.print(now); Serial.print(F(" mode="));Serial.print((int)mode);
  Serial.print(F(" filtered(LCR)=")); Serial.print(hasLeft(sensorMask)?'1':'0');
  Serial.print(hasCentre(sensorMask)?'1':'0'); Serial.print(hasRight(sensorMask)?'1':'0');
  Serial.print(F(" rawLOW(LCR)="));
  for(uint8_t i=0;i<3;i++) Serial.print((rawIRMask & (1<<i))?'1':'0');
  Serial.print(F(" falls(L/C/R)="));
  for(uint8_t i=0;i<3;i++) {
    if(i) Serial.print('/');
    Serial.print((uint16_t)(raw[i]-prevRaw[i])); prevRaw[i]=raw[i];
  }
  Serial.print(F(" valid="));
  for(uint8_t i=0;i<3;i++) {
    if(i) Serial.print('/');
    Serial.print((uint16_t)(valid[i]-prevValid[i])); prevValid[i]=valid[i];
  }
  Serial.print(F(" reject="));
  for(uint8_t i=0;i<3;i++) {
    if(i) Serial.print('/');
    Serial.print((uint16_t)(rejected[i]-prevReject[i])); prevReject[i]=rejected[i];
  }
  Serial.print(F(" rpm="));Serial.print(wheelRPM,1); Serial.print(F(" pwm="));Serial.print(motorPWM,0);
  Serial.print(F(" log="));Serial.println(logHeader.phase);
}
void serialCommands(unsigned long now) {
  while (Serial.available()) {
    char c=Serial.read();
    if(c=='T'||c=='t') telemetryEnabled=!telemetryEnabled;
    else if(c=='D'||c=='d') {
      // Interrupt movement for safe EEPROM readout; don't erase anything.
      enterStandby(now);
      if (recording()) { logHeader.phase=2; saveHeader(); }
      dumpRecorder(); // completed or partial log remains preserved
      mode=HALT; // stay motionless until R or C
    } else if(c=='C'||c=='c') {
      enterStandby(now);
      armRecorder();
      stopAtEnd=false;
      mode=HALT; // C arms NEXT boot; cannot record on current USB session
      Serial.println(F("ARMED: unplug USB, reinstall Nano, run batteries. Motor OFF until reboot."));
    } else if(c=='R'||c=='r') {
      if (recording()) {
        enterStandby(now);
        stopAtEnd=false;
        Serial.println(F("READY: recording continues, waiting for first IR flash."));
      } else {
        mode=HALT; Serial.println(F("STANDBY: send C, then reconnect on batteries to start a run."));
      }
    } else if(c=='X'||c=='x') {
      digitalWrite(PIN_LASER,LOW); emergencyMotorOff();
      if(recording()) { logHeader.phase=2; saveHeader(); }
      mode=HALT;
      Serial.println(F("HALTED: send R for IR-triggered standby."));
    }
  }
}
void setup() {
  Serial.begin(115200); pinMode(PIN_IN1,OUTPUT);pinMode(PIN_IN2,OUTPUT);
  pinMode(PIN_LASER,OUTPUT);pinMode(PIN_LED,OUTPUT); digitalWrite(PIN_LASER,LOW);emergencyMotorOff();
  for(uint8_t i=0;i<3;i++) pinMode(PIN_IR[i],INPUT);
  pinMode(PIN_ENC_A,INPUT);pinMode(PIN_ENC_B,INPUT); previousEncoderA=digitalRead(PIN_ENC_A);
  irPreviousPort=PIND & IR_MASK;
  PCMSK0 |= _BV(PCINT1); PCICR |= _BV(PCIE0);
  PCMSK2 |= _BV(PCINT18)|_BV(PCINT19)|_BV(PCINT20);
  PCICR |= _BV(PCIE2);
  EEPROM.get(ADDR_HEADER,logHeader);
  bool validLog=logHeader.magic==LOG_MAGIC && logHeader.version==LOG_VERSION &&
       logHeader.eventHead<EVENT_CAPACITY && logHeader.eventCount<=EVENT_CAPACITY &&
       logHeader.snapshotCount<=SNAPSHOT_LIMIT;

  if (!validLog) {
    // Fresh / incompatible EEPROM stays safe until explicit C (like V2).
    armRecorder();
    logHeader.armed=0; logHeader.phase=2;
    saveHeader();
  } else if (logHeader.phase==1) {
    // Battery test finished or reset. Retain all measured data for USB D.
    logHeader.phase=2;
    saveHeader();
  }

  uint32_t now=millis();
  speedSampleAt=now; lastMotorAt=now;
  enterStandby(now);

  Serial.println(F("MQ SENTINEL V8.8.3-S: STREAMLINED V8.8.3 (2.1s SHOT)"));
  Serial.println(F("D=dump  C=clear+arm NEXT BOOT  R=standby  T=telemetry  X=stop"));

  if (logHeader.phase!=0 || logHeader.armed!=1) {
    // No valid arm -> USB retrieval mode. Nothing can start moving.
    mode=HALT; Serial.println(F("DATA MODE: D=dump old test, C=arm a NEW battery run."));
    return;
  }

  // Nano cannot detect USB versus battery by software alone. Allow the
  // computer 5 seconds to cancel. C from Serial Monitor always arms a
  // FUTURE boot, not the current USB session.
  Serial.println(F("ARMED BOOT: 5-second window. Send D to dump instead."));
  unsigned long windowStarted=millis();
  while (millis()-windowStarted < 5000UL) {
    if (Serial.available()) {
      char c=Serial.read();
      if (c=='D' || c=='d') {
        mode=HALT;
        dumpRecorder();
        return;
      }
      if (c=='C' || c=='c') {
        armRecorder();
        mode=HALT; Serial.println(F("RE-ARMED: unplug USB and power from batteries."));
        return;
      }
      if (c=='X' || c=='x') {
        mode=HALT; Serial.println(F("STOPPED: existing armed EEPROM data is preserved."));
        return;
      }
    }
  }

  // Battery boot: start logging IMMEDIATELY, before the first IR flash.
  // Tracking itself remains in the original WAIT_FLASH motor-off mode.
  now=millis();
  startRecording(now);
  Serial.println(F("FLIGHT RECORDING: 5 minutes. Motor waits for IR flash."));
}
void loop() {
  unsigned long now=millis();
  serialCommands(now);
  readSensors(now);
  readWheel(now);
  if (stopAtEnd) {
    digitalWrite(PIN_LASER,LOW);
    emergencyMotorOff();
    mode=HALT;
  } else {
    logSensorTransition(now);
    serviceController(now);
    serviceMotor(now); sampleFlight(now);
    serviceEEPROM(); // <= 1 changed byte per loop, background recording
  }
  digitalWrite(PIN_LED,(mode==FIRE || mode==SETTLE)?HIGH:LOW);
  showTelemetry(now);
}
