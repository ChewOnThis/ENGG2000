/*
  MQ SENTINEL - REFERENCE-SAFE IR/GYRO CONTROLLER + TEST HARNESS
  Arduino Nano / ATmega328P. Serial Monitor 115200 baud, Newline.

  CURRENT HARDWARE: 3 x active-LOW 38kHz TSSP outputs (D2/D3/D4),
    MPU-6050 (SDA A4, SCL A5), optional properly driven eye-safe laser (D8),
    onboard status LED D13. NO MOTOR OR ENCODER REQUIRED.

  OPTIONAL FUTURE HARDWARE (set ENABLE_MOTOR_TESTS 1):
    DRV8874 in IN/IN mode IN1=D6(PWM), IN2=D7, encoder A=D9, B=D10.
    Motor commands are ONLY supervised tests, not autonomous pointing.

  First verified IR detection -> still gyro calibration -> wait until all IR
  clears -> survey a complete 360-degree net rotation -> map 1-3 primary
  receiver arcs. Reference held only in RAM; no EEPROM or silent re-anchoring.

  CRITICAL: No MPU-6050 gyro-only solution can recover absolute azimuth after
  power loss, unnoticed drift, or gyro saturation. The IR beacon can provide
  a fresh landmark, but this sketch does not pretend the angle is absolute.
*/
#define ENABLE_MOTOR_TESTS 0       // 0 = works with sensors/gyro alone
#define ENABLE_LASER_TEST 0        // set 1 ONLY for a correctly driven eye-safe laser
#define IR_SENSOR_COUNT 3          // change to 1 for only D2 connected
#define PRIMARY_SENSOR 1           // 1 = centre D3 (change to 0 for D2)

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#if ENABLE_MOTOR_TESTS
#include <avr/interrupt.h>
#include <util/atomic.h>
#endif

// --------------------- USER CONFIGURATION ---------------------------
const uint8_t IR_PINS[3] = {2, 3, 4};
const uint8_t LASER_PIN = 8;
const uint8_t STATUS_LED = 13;
const uint8_t MPU_ADDRESS = 0x68;
const float GYRO_LSB_DPS = 131.0f;       // MPU gyro +/-250 deg/s
const float ACC_LSB_G = 16384.0f;       // MPU accelerometer +/-2g

// TSSP active-low signal is sampled at 500 Hz. A rolling 32 ms window
// plus acceptance delays rejects isolated pulses. Tune from FLICKER logs.
const uint16_t IR_SAMPLE_MS = 2;
const uint16_t IR_ON_VERIFY_MS = 65;
const uint16_t IR_OFF_VERIFY_MS = 130;
const uint8_t IR_ON_OCCUPANCY = 9;      // out of 16 samples
const uint8_t IR_OFF_OCCUPANCY = 5;
const uint16_t STARTUP_CLEAR_MS = 200;
const uint16_t MIN_ARC_MS = 180;
const float MIN_ARC_DEG = 0.7f;
const float MAX_ARC_DEG = 90.0f;
const float TARGET_MERGE_DEG = 18.0f;
const uint8_t MAX_TARGETS = 3;
const float ALIGN_TOL_DEG = 2.0f;
const float REST_RATE_DPS = 0.75f;
const uint16_t GYRO_SAMPLE_MS = 10;
const uint16_t CAL_SAMPLES_REQUIRED = 220; // ~2.2 seconds
const float CAL_MAX_SD_DPS = 0.65f;
const float CAL_ACCEL_G_TOLERANCE = 0.15f;
const float JOLT_ACCEL_G = 1.65f;       // conservative invalidation threshold
const float JOLT_ACCEL_LOW_G = 0.45f;
const float JOLT_GYRO_DPS = 235.0f;    // near +/-250 dps full scale
const uint8_t MAX_GYRO_READ_FAILURES = 4;

#if ENABLE_MOTOR_TESTS
const uint8_t MOTOR_IN1 = 6;
const uint8_t MOTOR_IN2 = 7;
const uint8_t ENCODER_A = 9;
const uint8_t ENCODER_B = 10;
const float ENCODER_COUNTS_PER_REV = 700.0f; // VERIFY on your actual encoder
const uint8_t MAX_TEST_PWM = 180;
const uint16_t MAX_TEST_DRIVE_MS = 1200;
const uint16_t COAST_RECORD_MS = 1800;
const float TRIP_ENCODER_RPM = 170.0f;
#endif
// --------------------------------------------------------------------

#if (IR_SENSOR_COUNT < 1 || IR_SENSOR_COUNT > 3)
#error IR_SENSOR_COUNT must be 1, 2, or 3
#endif
#if (PRIMARY_SENSOR >= IR_SENSOR_COUNT)
#error PRIMARY_SENSOR must be less than IR_SENSOR_COUNT
#endif

enum Mode : uint8_t {
  WAIT_TRIGGER, CALIBRATING, WAIT_CLEAR, SURVEYING, MAP_READY,
  MAP_INCOMPLETE, VERIFYING, REFERENCE_LOST
};
Mode mode = WAIT_TRIGGER;
const __FlashStringHelper *modeName() {
  switch (mode) {
    case WAIT_TRIGGER: return F("WAIT_TRIGGER");
    case CALIBRATING: return F("CALIBRATING");
    case WAIT_CLEAR: return F("WAIT_CLEAR");
    case SURVEYING: return F("SURVEYING");
    case MAP_READY: return F("MAP_READY");
    case MAP_INCOMPLETE: return F("MAP_INCOMPLETE");
    case VERIFYING: return F("VERIFYING");
    default: return F("REFERENCE_LOST");
  }
}

struct IrFilter {
  uint16_t recent;
  bool qualified;
  bool candidate;
  bool pending;
  uint32_t candidateAt;
  float candidateAngle;
  uint32_t qualifiedAt;
  float entryAngle;
  uint32_t rawLowSamples;
  uint32_t totalSamples;
  uint32_t qualifiedSamples;
  uint32_t changes;
  uint32_t shortLowRuns;
  bool lastRaw;
  uint16_t rawRun;
};
IrFilter ir[3];

struct Target {
  float centre; // [0, 360), referenced ONLY to THIS calibration
  float width;
  float largestDeviation;
  uint16_t observations;
};
Target targets[MAX_TARGETS];
uint8_t targetCount = 0;
uint8_t targetGoal = 1; // SINGLE BEACON DEFAULT; TARGETS 3 for three fixed beacons
bool primaryArcOpen = false;
float primaryEntry = 0;
uint32_t primaryEntryAt = 0;
float scanStartYaw = 0;
bool scanTurnAnnounced = false;
bool verifyReady = false;
uint32_t verifyClearSince = 0;
bool marked = false;
float markedYaw = 0;
uint32_t allClearSince = 0;

float gyroBias = 0;
float gyroYaw = 0;         // UNWRAPPED, integration drift is unavoidable
float gyroRate = 0;
float gyroRawRate = 0;
float accelMagnitude = 1;
float temperatureC = 0;
bool gyroHealthy = false;
bool gyroCalibrated = false;
uint32_t lastGyroMs = 0;
uint32_t lastGyroUs = 0;
uint8_t gyroFailures = 0;
uint32_t joltCount = 0;

uint16_t calSamples = 0;
float calMean = 0;
float calM2 = 0;
float calAccMin = 1e6;
float calAccMax = 0;
uint32_t calStartedMs = 0;

bool traceOn = false;
uint32_t lastTraceMs = 0;
uint32_t lastIrMs = 0;
uint32_t lastStatusLedMs = 0;

struct TimedTest {
  bool active;
  uint32_t beganMs;
  uint32_t durationMs;
  uint32_t lastPrintMs;
  uint32_t rawSamplesAtStart[3];
  uint32_t lowSamplesAtStart[3];
  uint32_t changeAtStart[3];
  uint32_t shortRunAtStart[3];
  float yawStart;
  float minYaw;
  float maxYaw;
  float maxRate;
  float tempStart;
  uint32_t motionSamples;
};
TimedTest flickerTest = {};
TimedTest driftTest = {};

uint32_t laserOffAt = 0;
bool laserArmed = false;
uint32_t laserArmAt = 0;

#if ENABLE_MOTOR_TESTS
volatile int32_t encoderCount = 0;
ISR(PCINT0_vect) {
  // Quadrature relative wheel rotation; verify direction and counts/rev.
  static bool prevA = false;
  bool a = (PINB & _BV(PB1)) != 0;
  if (a && !prevA) {
    bool b = (PINB & _BV(PB2)) != 0;
    encoderCount += (b ? -1 : +1);
  }
  prevA = a;
}
int32_t readCounts() {
  int32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = encoderCount; }
  return value;
}
void motorOff() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW); // coast; does NOT physically halt the wheel
}
void motorForward(uint8_t pwm) {
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_IN1, pwm);
}
struct MotorTest {
  bool active;
  bool coasting;
  bool aborted;
  uint8_t pwm;
  uint32_t startMs;
  uint32_t driveUntil;
  uint32_t finishAt;
  uint32_t lastLogMs;
  uint32_t lastRpmMs;
  int32_t startCounts;
  int32_t rpmCounts;
  float startYaw;
  float wheelRpm;
  float peakRpm;
  float ratePeak;
  float tailRPMTotal;
  uint16_t tailSamples;
};
MotorTest motorTest = {};
uint32_t motorArmedUntil = 0;
#endif

float normalize360(float a) {
  a = fmodf(a, 360.0f);
  return a < 0 ? a + 360.0f : a;
}
float wrap180(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}
float absf(float x) { return x < 0 ? -x : x; }
bool referenceUsable() {
  return gyroCalibrated && gyroHealthy && mode != REFERENCE_LOST;
}
uint8_t qualifiedMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
    if (ir[i].qualified) mask |= (1 << i);
  }
  return mask;
}
uint8_t bitCount16(uint16_t x) {
  uint8_t n = 0;
  while (x) { n += x & 1; x >>= 1; }
  return n;
}

void laserOff() {
  digitalWrite(LASER_PIN, LOW);
  laserOffAt = 0;
}
void clearMap() {
  targetCount = 0;
  primaryArcOpen = false;
  scanTurnAnnounced = false;
  verifyReady = false;
  verifyClearSince = 0;
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    targets[i].centre = 0;
    targets[i].width = 0;
    targets[i].largestDeviation = 0;
    targets[i].observations = 0;
  }
}
void invalidateReference(const __FlashStringHelper *reason) {
  if (mode == REFERENCE_LOST) return;
  mode = REFERENCE_LOST;
  gyroCalibrated = false;
  laserOff();
  laserArmed = false;
  clearMap();
  if (flickerTest.active || driftTest.active)
    Serial.println(F("TEST_ABORT,gyro reference invalidated"));
  flickerTest.active = false; driftTest.active = false;
#if ENABLE_MOTOR_TESTS
  motorOff();
  motorTest.active = false;
  motorArmedUntil = 0;
#endif
  Serial.print(F("FAULT,REFERENCE_LOST,ms=")); Serial.print(millis());
  Serial.print(F(",reason=")); Serial.println(reason);
  Serial.println(F("All target angles discarded. Keep still near beacon; type RESET."));
}

bool writeMPU(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool readMPU(float &zDps, float &magG, float &tempC) {
  // Burst sample: ACCEL_X/Y/Z, TEMP, GYRO_X/Y/Z
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDRESS, (uint8_t)14, (uint8_t)true) != 14) return false;
  int16_t v[7];
  for (uint8_t i = 0; i < 7; i++) {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    v[i] = (int16_t)((high << 8) | low);
  }
  float ax = v[0] / ACC_LSB_G;
  float ay = v[1] / ACC_LSB_G;
  float az = v[2] / ACC_LSB_G;
  magG = sqrtf(ax * ax + ay * ay + az * az);
  tempC = v[3] / 340.0f + 36.53f;
  zDps = v[6] / GYRO_LSB_DPS;
  return true;
}

void resetForTrigger() {
  laserOff(); laserArmed = false;
#if ENABLE_MOTOR_TESTS
  motorOff(); motorArmedUntil = 0; motorTest.active = false;
#endif
  clearMap();
  marked = false;
  gyroCalibrated = false;
  gyroHealthy = true;
  gyroBias = 0;
  gyroYaw = 0;
  gyroRate = 0;
  gyroFailures = 0;
  calSamples = 0;
  flickerTest.active = false;
  driftTest.active = false;
  mode = WAIT_TRIGGER;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
    ir[i].recent = 0;
    ir[i].qualified = false;
    ir[i].candidate = false;
    ir[i].pending = false;
    ir[i].rawRun = 0;
    ir[i].lastRaw = false;
  }
  lastGyroUs = micros();
  Serial.println(F("RESET,Waiting for sustained IR trigger; then HOLD STILL for gyro bias calibration."));
}

void startCalibration() {
  mode = CALIBRATING;
  gyroCalibrated = false;
  calSamples = 0;
  calMean = 0;
  calM2 = 0;
  calAccMin = 1e6;
  calAccMax = 0;
  calStartedMs = millis();
  Serial.println(F("CAL_START,IR verified. HOLD COMPLETELY STILL for ~2.2 seconds."));
}
void acceptCalibration() {
  if (calSamples < CAL_SAMPLES_REQUIRED) return;
  float sd = calSamples > 1 ? sqrtf(calM2 / (calSamples - 1)) : 999;
  if (sd > CAL_MAX_SD_DPS ||
      calAccMin < 1.0f - CAL_ACCEL_G_TOLERANCE ||
      calAccMax > 1.0f + CAL_ACCEL_G_TOLERANCE) {
    Serial.print(F("CAL_REJECT,gyro_sd_dps=")); Serial.print(sd, 4);
    Serial.print(F(",acc_min_g=")); Serial.print(calAccMin, 3);
    Serial.print(F(",acc_max_g=")); Serial.println(calAccMax, 3);
    Serial.println(F("Keep STILL; restarting calibration."));
    calSamples = 0; calMean = 0; calM2 = 0;
    calAccMin = 1e6; calAccMax = 0;
    return;
  }
  gyroBias = calMean;
  gyroYaw = 0;
  gyroRate = 0;
  lastGyroUs = micros();
  gyroCalibrated = true;
  mode = WAIT_CLEAR;
  allClearSince = 0;
  clearMap();
  Serial.print(F("CAL_OK,bias_dps=")); Serial.print(gyroBias, 6);
  Serial.print(F(",noise_sd_dps=")); Serial.print(sd, 5);
  Serial.print(F(",temp_c=")); Serial.println(temperatureC, 2);
  Serial.println(F("Reference=0 DEG AT IR TRIGGER. Move OUT of all IR cones; then sweep steadily 360 DEG."));
  Serial.println(F("The first partial cone is intentionally DISCARDED."));
}

void updateGyro(uint32_t ms) {
  if ((uint32_t)(ms - lastGyroMs) < GYRO_SAMPLE_MS) return;
  lastGyroMs = ms;
  float rate, mag, temp;
  uint32_t timeUs = micros();
  if (!readMPU(rate, mag, temp)) {
    if (++gyroFailures >= MAX_GYRO_READ_FAILURES) {
      gyroHealthy = false;
      invalidateReference(F("MPU_READ_FAILURE"));
    }
    return;
  }
  gyroFailures = 0;
  gyroHealthy = true;
  gyroRawRate = rate;
  accelMagnitude = mag;
  temperatureC = temp;

  // During calibration the still gyro rate is measured, not integrated.
  if (mode == CALIBRATING) {
    ++calSamples;
    float delta = rate - calMean;
    calMean += delta / calSamples;
    calM2 += delta * (rate - calMean);
    if (mag < calAccMin) calAccMin = mag;
    if (mag > calAccMax) calAccMax = mag;
    if (calSamples >= CAL_SAMPLES_REQUIRED) acceptCalibration();
  } else if (gyroCalibrated) {
    float dt = (uint32_t)(timeUs - lastGyroUs) * 0.000001f;
    // If the loop stalls enough to miss meaningful motion, reference is unsafe.
    if (dt > 0.120f) {
      invalidateReference(F("GYRO_SAMPLE_GAP"));
    } else if (mode != REFERENCE_LOST) {
      gyroRate = rate - gyroBias;
      gyroYaw += gyroRate * dt;
      if (absf(rate) >= JOLT_GYRO_DPS || mag >= JOLT_ACCEL_G || mag <= JOLT_ACCEL_LOW_G) {
        ++joltCount;
        Serial.print(F("JOLT,rate_dps=")); Serial.print(rate, 2);
        Serial.print(F(",accel_g=")); Serial.println(mag, 2);
        invalidateReference(F("GYRO_SATURATION_OR_LARGE_ACCELERATION"));
      }
    }
  }
  lastGyroUs = timeUs;
}

void addTargetArc(float centreRaw, float width, uint32_t durationMs) {
  if (mode != SURVEYING && mode != VERIFYING) return;
  if (durationMs < MIN_ARC_MS || width < MIN_ARC_DEG || width > MAX_ARC_DEG) {
    Serial.print(F("REJECT_ARC,duration_ms=")); Serial.print(durationMs);
    Serial.print(F(",width_deg=")); Serial.println(width, 3);
    return;
  }
  float centre = normalize360(centreRaw);
  Serial.print(F("ARC,ms=")); Serial.print(millis());
  Serial.print(F(",centre_deg=")); Serial.print(centre, 3);
  Serial.print(F(",width_deg=")); Serial.print(width, 3);
  Serial.print(F(",duration_ms=")); Serial.println(durationMs);

  int8_t found = -1;
  for (uint8_t i = 0; i < targetCount; i++) {
    if (absf(wrap180(centre - targets[i].centre)) <= TARGET_MERGE_DEG) {
      found = i;
      break;
    }
  }
  if (found >= 0) {
    Target &t = targets[found];
    float deviation = wrap180(centre - t.centre);
    if (absf(deviation) > t.largestDeviation) t.largestDeviation = absf(deviation);
    t.centre = normalize360(t.centre + deviation / (t.observations + 1));
    t.width += (width - t.width) / (t.observations + 1);
    t.observations++;
    Serial.print(F("TARGET_UPDATE,index=")); Serial.print(found + 1);
    Serial.print(F(",observations=")); Serial.println(t.observations);
  } else if (targetCount < targetGoal) {
    Target &t = targets[targetCount];
    t.centre = centre; t.width = width;
    t.largestDeviation = 0; t.observations = 1;
    targetCount++;
    Serial.print(F("TARGET_NEW,index=")); Serial.print(targetCount);
    Serial.print(F(",centre_deg=")); Serial.println(centre, 3);
  } else {
    Serial.println(F("ARC_EXTRA,No free target slots; possible reflection or too-close duplicate merge threshold."));
  }
}

void qualifiedEdge(uint8_t idx, bool nowActive, float edgeAngle, uint32_t edgeAt) {
  IrFilter &s = ir[idx];
  ++s.changes;
  if (nowActive) {
    s.qualifiedAt = edgeAt;
    s.entryAngle = edgeAngle;
  } else if (s.qualifiedAt) {
    float width = absf(edgeAngle - s.entryAngle);
    Serial.print(F("SENSOR_ARC,sensor=")); Serial.print(idx == 0 ? 'L' : idx == 1 ? 'C' : 'R');
    Serial.print(F(",entry=")); Serial.print(s.entryAngle, 3);
    Serial.print(F(",exit=")); Serial.print(edgeAngle, 3);
    Serial.print(F(",width=")); Serial.print(width, 3);
    Serial.print(F(",duration_ms=")); Serial.println((uint32_t)(edgeAt - s.qualifiedAt));
  }
  if (idx != PRIMARY_SENSOR || (mode != SURVEYING && mode != VERIFYING) ||
      (mode == VERIFYING && !verifyReady)) return;
  if (nowActive) {
    primaryArcOpen = true;
    primaryEntry = edgeAngle;
    primaryEntryAt = edgeAt;
  } else if (primaryArcOpen) {
    primaryArcOpen = false;
    float width = absf(edgeAngle - primaryEntry);
    addTargetArc((primaryEntry + edgeAngle) * 0.5f, width,
                 (uint32_t)(edgeAt - primaryEntryAt));
  }
}

void updateIr(uint32_t ms) {
  if ((uint32_t)(ms - lastIrMs) < IR_SAMPLE_MS) return;
  lastIrMs = ms;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
    IrFilter &s = ir[i];
    bool rawLow = (digitalRead(IR_PINS[i]) == LOW);
    s.totalSamples++;
    if (rawLow) s.rawLowSamples++;
    if (s.qualified) s.qualifiedSamples++;
    if (rawLow != s.lastRaw && s.rawRun > 0 && s.lastRaw && s.rawRun <= 3)
      ++s.shortLowRuns;
    s.lastRaw = rawLow;
    s.rawRun = rawLow ? (uint16_t)(s.rawRun + 1) : 0;
    s.recent = (uint16_t)((s.recent << 1) | (rawLow ? 1 : 0));
    uint8_t occupancy = bitCount16(s.recent);
    bool desired = s.qualified;
    if (!s.qualified && occupancy >= IR_ON_OCCUPANCY) desired = true;
    if (s.qualified && occupancy <= IR_OFF_OCCUPANCY) desired = false;

    if (desired == s.qualified) {
      s.pending = false;
    } else if (!s.pending || s.candidate != desired) {
      s.pending = true;
      s.candidate = desired;
      s.candidateAt = ms;
      s.candidateAngle = gyroYaw; // leading filter boundary, not confirmation angle
    } else if ((uint32_t)(ms - s.candidateAt) >=
                (desired ? IR_ON_VERIFY_MS : IR_OFF_VERIFY_MS)) {
      s.qualified = desired;
      s.pending = false;
      if (referenceUsable() && mode != WAIT_CLEAR && mode != WAIT_TRIGGER)
        qualifiedEdge(i, desired, s.candidateAngle, s.candidateAt);
      else if (desired) {
        s.qualifiedAt = s.candidateAt;
        s.entryAngle = s.candidateAngle;
      } else s.qualifiedAt = 0;
    }
  }
  if (mode == WAIT_TRIGGER && qualifiedMask() != 0) startCalibration();
}

void printMap() {
  Serial.print(F("MAP,state=")); Serial.print(modeName());
  Serial.print(F(",found=")); Serial.print(targetCount);
  Serial.print(F(",goal=")); Serial.println(targetGoal);
  if (!referenceUsable()) {
    Serial.println(F("MAP_UNTRUSTED,reference not calibrated/valid")); return;
  }
  for (uint8_t i = 0; i < targetCount; i++) {
    Serial.print(F("TARGET,index=")); Serial.print(i + 1);
    Serial.print(F(",bearing_deg=")); Serial.print(targets[i].centre, 3);
    Serial.print(F(",width_deg=")); Serial.print(targets[i].width, 3);
    Serial.print(F(",samples=")); Serial.print(targets[i].observations);
    Serial.print(F(",max_deviation_deg=")); Serial.println(targets[i].largestDeviation, 3);
  }
  Serial.println(F("Coordinates are gyro-relative RAM only, NOT absolute world headings."));
}

void updateSurvey(uint32_t ms) {
  if (mode == WAIT_CLEAR) {
    if (qualifiedMask() == 0) {
      if (allClearSince == 0) allClearSince = ms;
      if ((uint32_t)(ms - allClearSince) >= STARTUP_CLEAR_MS) {
        clearMap();
        scanStartYaw = gyroYaw;
        mode = SURVEYING;
        Serial.print(F("SURVEY_START,start_yaw_deg=")); Serial.println(scanStartYaw, 3);
        Serial.println(F("Rotate ONE direction slowly; keep beacon(s) fixed ~1m away."));
      }
    } else allClearSince = 0;
  } else if (mode == VERIFYING) {
    // A verification session starts only after the existing primary cone clears.
    if (!verifyReady) {
      if (!ir[PRIMARY_SENSOR].qualified) {
        if (!verifyClearSince) verifyClearSince = ms;
        if ((uint32_t)(ms - verifyClearSince) >= STARTUP_CLEAR_MS) {
          verifyReady = true;
          primaryArcOpen = false;
          Serial.println(F("VERIFY_READY,Pass through beacon back and forth; type VERIFY STOP when finished."));
        }
      } else verifyClearSince = 0;
    }
  } else if (mode == SURVEYING) {
    float travel = absf(gyroYaw - scanStartYaw);
    if (travel >= 360.0f && !scanTurnAnnounced) {
      scanTurnAnnounced = true;
      Serial.print(F("SWEEP_360_REACHED,net_travel_deg=")); Serial.print(travel, 2);
      Serial.print(F(",found=")); Serial.print(targetCount);
      Serial.print(F(",expected=")); Serial.println(targetGoal);
    }
    // Allow final full arc to finish. Do not end while any primary arc is open.
    if (scanTurnAnnounced && !primaryArcOpen && !ir[PRIMARY_SENSOR].qualified &&
        !ir[PRIMARY_SENSOR].pending && travel > 365.0f) {
      mode = targetCount == targetGoal ? MAP_READY : MAP_INCOMPLETE;
      Serial.print(F("SURVEY_END,status="));
      Serial.println(mode == MAP_READY ? F("READY") : F("INCOMPLETE"));
      printMap();
    }
    if (travel > 750.0f) {
      mode = MAP_INCOMPLETE;
      Serial.println(F("SURVEY_STOP,Exceeded 750 degrees without completion."));
      printMap();
    }
  }
}

void status() {
  Serial.print(F("STATUS,ms=")); Serial.print(millis());
  Serial.print(F(",state=")); Serial.print(modeName());
  Serial.print(F(",yaw_deg=")); Serial.print(gyroYaw, 3);
  Serial.print(F(",rate_dps=")); Serial.print(gyroRate, 3);
  Serial.print(F(",bias_dps=")); Serial.print(gyroBias, 5);
  Serial.print(F(",accel_g=")); Serial.print(accelMagnitude, 3);
  Serial.print(F(",temp_c=")); Serial.print(temperatureC, 2);
  Serial.print(F(",ir_mask=")); Serial.print(qualifiedMask(), BIN);
  Serial.print(F(",targets=")); Serial.print(targetCount);
  Serial.print(F(",jolts=")); Serial.println(joltCount);
  if (mode == MAP_READY && targetCount) {
    float err = wrap180(targets[0].centre - normalize360(gyroYaw));
    Serial.print(F("ALIGN,target1_error_deg=")); Serial.print(err, 3);
    Serial.print(F(",gyro_still=")); Serial.println(absf(gyroRate) < REST_RATE_DPS ? F("YES") : F("NO"));
  }
}

void startTimedTest(TimedTest &t, uint32_t durationSeconds) {
  memset(&t, 0, sizeof(t));
  t.active = true;
  t.beganMs = millis();
  t.durationMs = durationSeconds * 1000UL;
  t.lastPrintMs = t.beganMs;
  t.yawStart = gyroYaw;
  t.minYaw = t.maxYaw = gyroYaw;
  t.tempStart = temperatureC;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
    t.rawSamplesAtStart[i] = ir[i].totalSamples;
    t.lowSamplesAtStart[i] = ir[i].rawLowSamples;
    t.changeAtStart[i] = ir[i].changes;
    t.shortRunAtStart[i] = ir[i].shortLowRuns;
  }
}

void updateTests(uint32_t ms) {
  if (flickerTest.active) {
    TimedTest &t = flickerTest;
    if ((uint32_t)(ms - t.lastPrintMs) >= 5000) {
      t.lastPrintMs = ms;
      Serial.print(F("FLICKER_SAMPLE,elapsed_s="));
      Serial.print((uint32_t)(ms - t.beganMs) / 1000);
      Serial.print(F(",qualified_mask=")); Serial.println(qualifiedMask(), BIN);
    }
    if ((uint32_t)(ms - t.beganMs) >= t.durationMs) {
      t.active = false;
      for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        uint32_t samples = ir[i].totalSamples - t.rawSamplesAtStart[i];
        uint32_t low = ir[i].rawLowSamples - t.lowSamplesAtStart[i];
        uint32_t switches = ir[i].changes - t.changeAtStart[i];
        Serial.print(F("RESULT_FLICKER,sensor=")); Serial.print(i == 0 ? 'L' : i == 1 ? 'C' : 'R');
        Serial.print(F(",samples=")); Serial.print(samples);
        Serial.print(F(",raw_low_pct="));
        Serial.print(samples ? low * 100.0f / samples : 0, 2);
        Serial.print(F(",qualified_edges=")); Serial.print(switches);
        Serial.print(F(",short_low_runs="));
        Serial.println(ir[i].shortLowRuns - t.shortRunAtStart[i]);
      }
    }
  }
  if (driftTest.active) {
    TimedTest &t = driftTest;
    if (gyroYaw < t.minYaw) t.minYaw = gyroYaw;
    if (gyroYaw > t.maxYaw) t.maxYaw = gyroYaw;
    if (absf(gyroRate) > t.maxRate) t.maxRate = absf(gyroRate);
    if (absf(gyroRate) > 2.0f) t.motionSamples++;
    if ((uint32_t)(ms - t.lastPrintMs) >= 1000) {
      t.lastPrintMs = ms;
      Serial.print(F("DRIFT_SAMPLE,sec=")); Serial.print((uint32_t)(ms - t.beganMs) / 1000);
      Serial.print(F(",yaw_delta_deg=")); Serial.print(gyroYaw - t.yawStart, 5);
      Serial.print(F(",rate_dps=")); Serial.print(gyroRate, 5);
      Serial.print(F(",temp_c=")); Serial.println(temperatureC, 2);
    }
    if ((uint32_t)(ms - t.beganMs) >= t.durationMs) {
      t.active = false;
      Serial.print(F("RESULT_DRIFT,elapsed_s=")); Serial.print(t.durationMs / 1000);
      Serial.print(F(",net_deg=")); Serial.print(gyroYaw - t.yawStart, 5);
      Serial.print(F(",range_deg=")); Serial.print(t.maxYaw - t.minYaw, 5);
      Serial.print(F(",max_rate_dps=")); Serial.print(t.maxRate, 5);
      Serial.print(F(",temp_delta_c=")); Serial.print(temperatureC - t.tempStart, 3);
      Serial.print(F(",movement_flags=")); Serial.println(t.motionSamples);
      Serial.println(F("Caution: if the chassis physically moved, net_deg is not pure gyro drift."));
    }
  }
}

#if ENABLE_MOTOR_TESTS
void updateMotor(uint32_t ms) {
  if (!motorTest.active) return;
  MotorTest &t = motorTest;
  if (!t.coasting && (int32_t)(ms - t.driveUntil) >= 0) {
    motorOff(); t.coasting = true;
    Serial.println(F("MOTOR_COAST,drive removed; wheel may continue rotating"));
  }
  if ((uint32_t)(ms - t.lastRpmMs) >= 100) {
    int32_t count = readCounts();
    float elapsed = (ms - t.lastRpmMs) / 1000.0f;
    t.wheelRpm = 60.0f * (count - t.rpmCounts) / (ENCODER_COUNTS_PER_REV * elapsed);
    t.lastRpmMs = ms;
    t.rpmCounts = count;
    if (absf(t.wheelRpm) > t.peakRpm) t.peakRpm = absf(t.wheelRpm);
    if (!t.coasting && (uint32_t)(ms - t.startMs) >=
        (uint32_t)(t.driveUntil - t.startMs) - 350) {
      t.tailRPMTotal += absf(t.wheelRpm);
      t.tailSamples++;
    }
    if (absf(t.wheelRpm) >= TRIP_ENCODER_RPM) {
      motorOff(); t.active = false; t.aborted = true;
      Serial.println(F("FAULT,MOTOR_ENCODER_RPM_TRIP; angle map invalidated by test"));
      clearMap(); mode = WAIT_CLEAR; allClearSince = 0;
      return;
    }
  }
  if (absf(gyroRate) > t.ratePeak) t.ratePeak = absf(gyroRate);
  if ((uint32_t)(ms - t.lastLogMs) >= 60) {
    t.lastLogMs = ms;
    Serial.print(F("MOTOR_DATA,ms=")); Serial.print(ms - t.startMs);
    Serial.print(F(",phase=")); Serial.print(t.coasting ? F("COAST") : F("DRIVE"));
    Serial.print(F(",pwm=")); Serial.print(t.coasting ? 0 : t.pwm);
    Serial.print(F(",yaw_deg=")); Serial.print(gyroYaw, 3);
    Serial.print(F(",rate_dps=")); Serial.print(gyroRate, 3);
    Serial.print(F(",counts=")); Serial.print(readCounts() - t.startCounts);
    Serial.print(F(",rpm=")); Serial.println(t.wheelRpm, 2);
  }
  if ((int32_t)(ms - t.finishAt) >= 0) {
    motorOff(); t.active = false;
    float wheelDeg = (readCounts() - t.startCounts) * 360.0f / ENCODER_COUNTS_PER_REV;
    float bodyDeg = gyroYaw - t.startYaw;
    Serial.print(F("RESULT_MOTOR,pwm=")); Serial.print(t.pwm);
    Serial.print(F(",drive_ms=")); Serial.print((uint32_t)(t.driveUntil - t.startMs));
    Serial.print(F(",body_deg=")); Serial.print(bodyDeg, 4);
    Serial.print(F(",wheel_deg=")); Serial.print(wheelDeg, 4);
    Serial.print(F(",K_observed="));
    if (absf(wheelDeg) > 1) Serial.print(-bodyDeg / wheelDeg, 5);
    else Serial.print(F("INVALID"));
    Serial.print(F(",peak_rpm=")); Serial.print(t.peakRpm, 2);
    Serial.print(F(",tail_mean_rpm="));
    if (t.tailSamples) Serial.print(t.tailRPMTotal / t.tailSamples, 2);
    else Serial.print(F("NA"));
    Serial.print(F(",end_rpm=")); Serial.print(t.wheelRpm, 2);
    Serial.print(F(",end_body_dps=")); Serial.println(gyroRate, 2);
    Serial.println(F("K_observed is provisional until body AND flywheel stop; no-load RPM requires steady speed."));
    clearMap(); mode = WAIT_CLEAR; allClearSince = 0;
  }
}
void beginMotorTest(uint8_t pwm, uint16_t driveMs) {
  if (mode == REFERENCE_LOST || !referenceUsable()) {
    Serial.println(F("ERR,Calibrate first.")); return;
  }
  if ((int32_t)(millis() - motorArmedUntil) > 0) {
    Serial.println(F("ERR,First type ARM MOTOR (expires in 15 seconds).")); return;
  }
  if (motorTest.active || pwm < 1 || pwm > MAX_TEST_PWM || driveMs < 40 || driveMs > MAX_TEST_DRIVE_MS) {
    Serial.println(F("ERR,Motor busy or PWM/time outside conservative limits.")); return;
  }
  motorArmedUntil = 0;
  memset(&motorTest, 0, sizeof(motorTest));
  MotorTest &t = motorTest;
  t.active = true;
  t.pwm = pwm;
  t.startMs = t.lastLogMs = t.lastRpmMs = millis();
  t.driveUntil = t.startMs + driveMs;
  t.finishAt = t.driveUntil + COAST_RECORD_MS;
  t.startCounts = t.rpmCounts = readCounts();
  t.startYaw = gyroYaw;
  Serial.print(F("MOTOR_START,pwm=")); Serial.print(pwm);
  Serial.print(F(",drive_ms=")); Serial.println(driveMs);
  motorForward(pwm);
}
#endif

void printHelp() {
  Serial.println(F("\n=== MQ SENTINEL 115200 baud / Newline ==="));
  Serial.println(F("HELP                - commands"));
  Serial.println(F("STATUS              - heading, bias, sensor mask, state"));
  Serial.println(F("MAP                 - relative target bearings / widths"));
  Serial.println(F("TARGETS 1           - one IR beacon (default), or TARGETS 3"));
  Serial.println(F("RESET               - invalidate angle; wait for next verified IR trigger"));
  Serial.println(F("SURVEY              - restart 360-degree sweep AFTER gyro calibration"));
  Serial.println(F("VERIFY              - after MAP_READY, collect repeat passes without clearing map"));
  Serial.println(F("VERIFY STOP         - finish repeated arc measurements"));
  Serial.println(F("MARK / CHECK 360    - compare physical marked rotation with gyro delta"));
  Serial.println(F("FLICKER 20          - raw/filtered signal stats for 20 seconds"));
  Serial.println(F("DRIFT 60            - stationary gyro data for 60 seconds"));
  Serial.println(F("TRACE ON | OFF      - 250ms streaming IMU / IR telemetry"));
  Serial.println(F("SPOT 20 1000        - optional measured laser offset mm/distance mm"));
#if ENABLE_LASER_TEST
  Serial.println(F("ARM LASER           - authorize short manual aligned pulse"));
  Serial.println(F("LASER 100           - pulse only if aligned and IR/gyro safe"));
#else
  Serial.println(F("LASER DISABLED in build. Do not connect directly to bare diode."));
#endif
#if ENABLE_MOTOR_TESTS
  Serial.println(F("ARM MOTOR           - authorize a single motor test for 15s"));
  Serial.println(F("PULSE 65 150        - motor PWM, milliseconds, then coast+log"));
  Serial.println(F("STOP                - motor off / laser off"));
#else
  Serial.println(F("Motor/encoder tests NOT COMPILED: ENABLE_MOTOR_TESTS=0."));
#endif
  Serial.println(F("No EEPROM / no absolute orientation recovery / no auto laser."));
}

char lineBuf[54];
uint8_t lineLen = 0;
void command(char *s) {
  char *cmd = strtok(s, " \t");
  if (!cmd) return;
  // Uppercase ASCII in-place for the command only.
  for (char *p = cmd; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
  char *a = strtok(NULL, " \t");
  char *b = strtok(NULL, " \t");
  if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) printHelp();
  else if (!strcmp(cmd, "STATUS")) status();
  else if (!strcmp(cmd, "MAP") || !strcmp(cmd, "SUMMARY")) printMap();
  else if (!strcmp(cmd, "RESET") || !strcmp(cmd, "R")) resetForTrigger();
  else if (!strcmp(cmd, "TARGETS")) {
    if (!a || atoi(a) < 1 || atoi(a) > 3) {
      Serial.println(F("ERR,Use TARGETS 1, TARGETS 2, or TARGETS 3"));
    } else {
      targetGoal = atoi(a);
      Serial.print(F("TARGET_GOAL,")); Serial.println(targetGoal);
      resetForTrigger();
    }
  } else if (!strcmp(cmd, "SURVEY")) {
    if (!referenceUsable() || mode == CALIBRATING) {
      Serial.println(F("ERR,Need a valid calibrated gyro; RESET then present IR."));
    } else {
      clearMap(); mode = WAIT_CLEAR; allClearSince = 0;
      Serial.println(F("SURVEY_RESET,Move OUT of IR first, then rotate one full turn."));
    }
  } else if (!strcmp(cmd, "VERIFY")) {
    if (a && (!strcmp(a, "STOP") || !strcmp(a, "stop"))) {
      if (mode == VERIFYING) { mode = MAP_READY; Serial.println(F("VERIFY_FINISHED")); printMap(); }
      else Serial.println(F("ERR,Not verifying."));
    } else if (mode == MAP_READY) {
      mode = VERIFYING; verifyReady = false; verifyClearSince = 0;
      primaryArcOpen = false;
      Serial.println(F("VERIFY_START,First exit the primary IR cone; then repeat passes."));
    } else Serial.println(F("ERR,VERIFY needs MAP_READY. Type MAP."));
  } else if (!strcmp(cmd, "MARK")) {
    if (!referenceUsable()) Serial.println(F("ERR,Calibrate first."));
    else { marked = true; markedYaw = gyroYaw;
      Serial.print(F("MARK_OK,yaw_deg=")); Serial.println(markedYaw, 4); }
  } else if (!strcmp(cmd, "CHECK")) {
    if (!marked || !referenceUsable() || !a) Serial.println(F("ERR,MARK first, then CHECK 360 or CHECK 0."));
    else { float expected = atof(a), measured = gyroYaw - markedYaw;
      Serial.print(F("RESULT_CHECK,expected_deg=")); Serial.print(expected, 3);
      Serial.print(F(",measured_deg=")); Serial.print(measured, 3);
      Serial.print(F(",error_deg=")); Serial.print(measured - expected, 3);
      Serial.print(F(",scale_ratio="));
      if (absf(expected) > 0.1f) Serial.println(measured / expected, 5);
      else Serial.println(F("NA")); }
  } else if (!strcmp(cmd, "FLICKER") || !strcmp(cmd, "DRIFT")) {
    uint32_t seconds = a ? atol(a) : 0;
    if (seconds < 5 || seconds > 180) {
      Serial.println(F("ERR,Use FLICKER 5..180 or DRIFT 5..180 seconds."));
    } else if (!strcmp(cmd, "DRIFT") && !referenceUsable()) {
      Serial.println(F("ERR,Run first-trigger gyro calibration before DRIFT."));
    } else {
      if (!strcmp(cmd, "FLICKER")) {
        startTimedTest(flickerTest, seconds);
        Serial.println(F("TEST_START,FLICKER. Hold sensor and beacon STATIONARY."));
      } else {
        startTimedTest(driftTest, seconds);
        Serial.println(F("TEST_START,DRIFT. Do NOT rotate or move chassis."));
      }
    }
  } else if (!strcmp(cmd, "TRACE")) {
    if (!a) Serial.println(F("ERR,TRACE ON or TRACE OFF"));
    else {
      if (a[0] >= 'a' && a[0] <= 'z') a[0] -= 32;
      traceOn = (a[0] == 'O' && (a[1] == 'N' || a[1] == 'n'));
      Serial.println(traceOn ? F("TRACE_ON") : F("TRACE_OFF"));
    }
  } else if (!strcmp(cmd, "SPOT")) {
    if (!a || !b || atof(b) <= 0) {
      Serial.println(F("ERR,SPOT <lateral_error_mm> <distance_mm>"));
    } else {
      float offset = atof(a), distance = atof(b);
      Serial.print(F("RESULT_SPOT,offset_mm=")); Serial.print(offset, 2);
      Serial.print(F(",distance_mm=")); Serial.print(distance, 2);
      Serial.print(F(",optical_offset_deg="));
      Serial.println(atan2f(offset, distance) * 180.0f / PI, 4);
    }
  } else if (!strcmp(cmd, "ARM")) {
    if (!a) { Serial.println(F("ERR,ARM LASER or ARM MOTOR")); return; }
#if ENABLE_LASER_TEST
    if (!strcmp(a, "LASER") || !strcmp(a, "laser")) {
      laserArmed = true; laserArmAt = millis();
      Serial.println(F("LASER_ARMED,10s window; manual aligned pulse only."));
      return;
    }
#endif
#if ENABLE_MOTOR_TESTS
    if (!strcmp(a, "MOTOR") || !strcmp(a, "motor")) {
      motorArmedUntil = millis() + 15000UL;
      Serial.println(F("MOTOR_ARMED,expires_in_15s"));
      return;
    }
#endif
    Serial.println(F("ERR,Requested device disabled in build."));
  } else if (!strcmp(cmd, "LASER")) {
#if ENABLE_LASER_TEST
    uint16_t ms = a ? atoi(a) : 0;
    if (!laserArmed || millis() - laserArmAt > 10000UL || ms < 1 || ms > 200 ||
        mode != MAP_READY || qualifiedMask() == 0 ||
        absf(gyroRate) >= REST_RATE_DPS ||
        targetCount == 0 || absf(wrap180(targets[0].centre - normalize360(gyroYaw))) > ALIGN_TOL_DEG) {
      Serial.println(F("LASER_REFUSED,need armed, valid map, IR visible, still and within 2deg"));
    } else {
      digitalWrite(LASER_PIN, HIGH);
      laserOffAt = millis() + ms;
      Serial.print(F("LASER_ON,ms=")); Serial.println(ms);
    }
    laserArmed = false;
#else
    Serial.println(F("LASER_DISABLED,compile ENABLE_LASER_TEST=1 for safe module"));
#endif
  } else if (!strcmp(cmd, "STOP")) {
    laserOff(); laserArmed = false;
#if ENABLE_MOTOR_TESTS
    motorOff(); motorTest.active = false; motorArmedUntil = 0;
#endif
    Serial.println(F("OUTPUTS_OFF,mechanical flywheel may still be spinning"));
  } else if (!strcmp(cmd, "PULSE")) {
#if ENABLE_MOTOR_TESTS
    if (!a || !b) Serial.println(F("ERR,PULSE <PWM> <ms>"));
    else beginMotorTest(atoi(a), atoi(b));
#else
    Serial.println(F("MOTOR_DISABLED,requires motor+encoder wiring and ENABLE_MOTOR_TESTS=1"));
#endif
  } else {
    Serial.println(F("ERR,Unknown command. Type HELP."));
  }
}
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen) {
        lineBuf[lineLen] = 0;
        command(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    else lineLen = 0;
  }
}

void firmwareSetup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) pinMode(IR_PINS[i], INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  laserOff();
#if ENABLE_MOTOR_TESTS
  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  motorOff();
  pinMode(ENCODER_A, INPUT_PULLUP); pinMode(ENCODER_B, INPUT_PULLUP);
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT1); // D9 encoder A change
#endif
  Wire.begin();
  Wire.setClock(100000L); // compatible default for small MPU breakout wiring
  delay(100);
  if (!writeMPU(0x6B, 0x00) || !writeMPU(0x1A, 0x03) ||
      !writeMPU(0x1B, 0x00) || !writeMPU(0x1C, 0x00) ||
      !writeMPU(0x19, 0x09)) {
    Serial.println(F("FATAL,MPU-6050 not responding at 0x68; inspect power/SDA/SCL/AD0"));
    mode = REFERENCE_LOST;
    return;
  }
  gyroHealthy = true;
  Serial.println(F("\nMQ SENTINEL - IR / GYRO TEST & REFERENCE-SAFE MAP"));
  Serial.println(F("Serial=115200, Newline. No EEPROM. No automatic laser firing."));
  Serial.print(F("IR count=")); Serial.print(IR_SENSOR_COUNT);
  Serial.print(F(" primary pin=")); Serial.println(IR_PINS[PRIMARY_SENSOR]);
  printHelp();
  resetForTrigger();
}

void firmwareLoop() {
  uint32_t ms = millis();
  readSerial();
  // Integrate angle before accepting edge snapshots.
  if (mode != REFERENCE_LOST) updateGyro(ms);
  updateIr(ms);
  if (referenceUsable()) updateSurvey(ms);
  updateTests(ms);
#if ENABLE_MOTOR_TESTS
  updateMotor(ms);
#endif
  if (laserOffAt && (int32_t)(ms - laserOffAt) >= 0) laserOff();
  if ((uint32_t)(ms - lastTraceMs) >= 250 && traceOn) {
    lastTraceMs = ms;
    Serial.print(F("TRACE,ms=")); Serial.print(ms);
    Serial.print(F(",state=")); Serial.print(modeName());
    Serial.print(F(",yaw_deg=")); Serial.print(gyroYaw, 3);
    Serial.print(F(",rate_dps=")); Serial.print(gyroRate, 3);
    Serial.print(F(",accel_g=")); Serial.print(accelMagnitude, 3);
    Serial.print(F(",mask=")); Serial.println(qualifiedMask(), BIN);
  }
  if ((uint32_t)(ms - lastStatusLedMs) >= 100) {
    lastStatusLedMs = ms;
    digitalWrite(STATUS_LED, mode == REFERENCE_LOST ? ((ms / 100) % 2) :
                 (mode == MAP_READY ? HIGH : LOW));
  }
}
