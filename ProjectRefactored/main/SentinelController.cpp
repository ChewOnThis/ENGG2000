/*
  MQ SENTINEL - AUTONOMOUS REACTION WHEEL CONTROLLER
  Target: classic Arduino Nano (ATmega328P), DRV8874 PWM / IN-IN mode.
  Serial: 115200 baud, Newline.

  SAFETY: The physical yaw response sign, encoder scale, IR receiver behaviour,
  motor RPM limits and laser alignment MUST be commissioned on your hardware.
  This is an autonomous prototype, NOT a verified flight/laser controller.
  Auto laser output defaults DISABLED and is simulated through serial until
  ENABLE_AUTO_LASER is deliberately enabled for an appropriate eye-safe module.

  STATE FLOW:
  First IR -> stationary gyro calibration ONCE (1000 samples at 5ms)
  -> clear starting cone -> wait for SECOND qualified IR trigger
  -> capture COMPLETE ON-to-OFF arc -> aim -> settle -> fire
  -> leave fired cone -> wait for ANOTHER trigger -> repeat without calibration.

  NO TARGET MAP: one temporary arc centre is used only for the current attempt.
  Each new shot / reacquisition discards that centre and records another arc.
  An already-detected beam is ignored until the receiver has fully cleared.
  If a serious reference or motor fault occurs: FAULT, outputs off.
  Angles live in RAM; never restore relative yaw from EEPROM after power loss.
*/
#include "SentinelController.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// =================== EXISTING SETTINGS: VALUES PRESERVED ===================
#define IR_SENSOR_COUNT 3           // set 1 if ONLY one sensor on D2
#define PRIMARY_SENSOR 1           // set 0 when IR_SENSOR_COUNT=1
const uint8_t IR_PINS[3] = {2, 3, 4}; // FIX: uploaded version declared length 1 with {4}
const uint8_t LASER_PIN = 8;
const uint8_t STATUS_LED = 13;
const uint8_t MPU_ADDRESS = 0x68;
const float GYRO_LSB_DPS = 131.0f;
const float ACC_LSB_G = 16384.0f;
const uint16_t IR_SAMPLE_MS = 2;
const uint16_t IR_ON_VERIFY_MS = 65;
const uint16_t IR_OFF_VERIFY_MS = 130;
const uint8_t IR_ON_OCCUPANCY = 9;
const uint8_t IR_OFF_OCCUPANCY = 5;
const uint16_t STARTUP_CLEAR_MS = 200;
const uint16_t MIN_ARC_MS = 180;
const float MIN_ARC_DEG = 0.7f;
const float MAX_ARC_DEG = 90.0f;
// Former target-merge/map settings are intentionally removed: no map is stored.
const float ALIGN_TOL_DEG = 2.0f;
const float REST_RATE_DPS = 0.75f;
const uint16_t GYRO_SAMPLE_MS = 5;
const uint16_t CAL_SAMPLES_REQUIRED = 1000; // nominal ~5 s
const float CAL_MAX_SD_DPS = 0.65f;
const float CAL_ACCEL_G_TOLERANCE = 0.15f;
const float JOLT_ACCEL_G = 1.65f;
const float JOLT_ACCEL_LOW_G = 0.45f;
const float JOLT_GYRO_DPS = 235.0f;
const uint8_t MAX_GYRO_READ_FAILURES = 4;
const uint8_t MOTOR_IN1 = 6;
const uint8_t MOTOR_IN2 = 7;
const uint8_t ENCODER_A = 9;
const uint8_t ENCODER_B = 10;
const float ENCODER_COUNTS_PER_REV = 700.0f;
const uint8_t MAX_TEST_PWM = 180;
const uint16_t MAX_TEST_DRIVE_MS = 1200;
const uint16_t COAST_RECORD_MS = 1800;
const float TRIP_ENCODER_RPM = 170.0f;
// ============================================================================

// ========================= NEW MOTOR / SAFETY SETTINGS =======================
// DRV8874 PMODE MUST BE HIGH on enable to select PWM (IN1/IN2) mode.
// Forward IN1=D6 PWM, IN2=D7 LOW. Reverse D7 HIGH, D6 inverted PWM.
// This gives reverse/brake drive; reverse differs electrically from forward.
// Verify with low-power PULSE tests before enabling autonomous operation.
const int8_t BODY_ACCEL_SIGN_FOR_MOTOR_FORWARD = -1;
const uint8_t AUTO_MAX_PWM = 110;       // conservative startup torque limit
const uint8_t AUTO_MIN_PWM = 48;        // minimum meaningful command (hardware-dependent)
const uint8_t MOTOR_PWM_SLEW_PER_TICK = 6;
const uint16_t MOTOR_CONTROL_MS = 10;
const uint16_t MOTOR_ZERO_CROSS_MS = 25;
const float MOTOR_RATE_KP = 9.0f;      // PWM / (body deg/s) (UNVALIDATED)
const float AIM_POS_KP = 1.4f;         // deg/s demanded per degree of angular error
const float SEEK_RATE_DPS = 9.0f;
const float SCAN_RATE_DPS = 12.0f;
const float AIM_RATE_LIMIT_DPS = 16.0f;
const float WHEEL_SOFT_LIMIT_RPM = 140.0f;
const float FIRE_MAX_WHEEL_RPM = 20.0f;
const uint16_t FIRST_IR_STILL_MS = 1000;
const uint16_t SETTLE_FOR_FIRE_MS = 550;
const uint16_t LASER_FIRE_MS = 2000;
const uint16_t POST_FIRE_MS = 350;
const uint32_t SEEK_TIMEOUT_MS = 25000UL;
const uint32_t SURVEY_TIMEOUT_MS = 55000UL; // now limits one fresh arc search
const uint32_t AIM_TIMEOUT_MS = 22000UL;
const uint32_t STILL_TIMEOUT_MS = 15000UL;
const uint32_t CAL_TIMEOUT_MS = 25000UL;
const uint32_t BOOT_GRACE_MS = 2000UL;
const uint8_t MAX_FIRE_ATTEMPTS = 3;
const float PRECAL_MAX_BODY_RATE_DPS = 35.0f; // new safety bound before bias calibration
const bool AUTORUN_ON_BOOT = true;
const bool AUTO_REPEAT = true;         // continuous cycles; gyro calibrates only on first verified IR
const uint32_t FRESH_ARC_MAX_AGE_MS = 12000UL; // new safety limit, requires a fresh arc before firing
const uint8_t MAX_FRESH_ARC_RETRIES = 3; // new safety limit for repeated failed acquisitions
#define ENABLE_AUTO_LASER 0          // 0 = simulated shot; 1 = D8 laser module output
// ============================================================================

#if IR_SENSOR_COUNT < 1 || IR_SENSOR_COUNT > 3
#error IR_SENSOR_COUNT must be 1, 2 or 3
#endif
#if PRIMARY_SENSOR >= IR_SENSOR_COUNT
#error PRIMARY_SENSOR is not connected
#endif

float clipf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
float absolute(float x) { return x < 0 ? -x : x; }
float wrap360(float x) { x = fmodf(x, 360.0f); return x < 0 ? x + 360.0f : x; }
float wrap180(float x) { return fmodf(wrap360(x + 180.0f) + 360.0f, 360.0f) - 180.0f; }

volatile int32_t encoderCounts = 0;
volatile bool encoderPrevA = false;
ISR(PCINT0_vect) {
  const bool a = (PINB & _BV(PB1)) != 0;
  if (a && !encoderPrevA) {
    const bool b = (PINB & _BV(PB2)) != 0;
    encoderCounts += b ? -1 : 1;
  }
  encoderPrevA = a;
}
int32_t countsNow() {
  int32_t n;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { n = encoderCounts; }
  return n;
}

enum State : uint8_t {
  BOOT_DELAY, SEEK_TRIGGER, WAIT_STILL, CALIBRATING, WAIT_CLEAR, SEEK_NEXT_IR, ACQUIRE_ARC,
  AIM, SETTLE, FIRE, POST_FIRE, FINISHED, STOPPED, MOTOR_TEST, FAULT
};
State state = BOOT_DELAY;
const __FlashStringHelper *stateName() {
  switch (state) {
    case BOOT_DELAY: return F("BOOT_DELAY");
    case SEEK_TRIGGER: return F("SEEK_TRIGGER");
    case WAIT_STILL: return F("WAIT_STILL");
    case CALIBRATING: return F("CALIBRATING");
    case WAIT_CLEAR: return F("WAIT_CLEAR");
    case SEEK_NEXT_IR: return F("SEEK_NEXT_IR");
    case ACQUIRE_ARC: return F("ACQUIRE_ARC");
    case AIM: return F("AIM");
    case SETTLE: return F("SETTLE");
    case FIRE: return F("FIRE");
    case POST_FIRE: return F("POST_FIRE");
    case FINISHED: return F("FINISHED");
    case STOPPED: return F("STOPPED");
    case MOTOR_TEST: return F("MOTOR_TEST");
    default: return F("FAULT");
  }
}

struct IrData {
  uint16_t bits;
  bool qualified, pending, pendingValue;
  uint32_t candidateAt, acceptedAt;
  float candidateAngle, entryAngle;
  uint32_t total, low, transitions, shortLowRuns;
  uint8_t shortRun;
  bool previousRaw;
};
IrData ir[3] = {};

// Single-use target direction. Not a map: discarded before every new scan.
struct FreshArc {
  bool valid;
  float centre;      // continuous gyro-relative yaw, not a persistent heading
  float width;
  uint32_t durationMs;
  uint32_t measuredAt;
};
FreshArc freshArc = {};
uint8_t fireAttempts = 0, freshArcRetries = 0;
bool arcOpen = false;
float arcEntryAngle = 0, scanStartYaw = 0;
uint32_t arcEntryAt = 0, clearSince = 0, alignedSince = 0;

float yaw = 0, rawRate = 0, yawRate = 0, bias = 0, accelG = 1, tempC = 0;
bool imuValid = false, calibrated = false, everCalibrated = false;
uint8_t badReads = 0;
uint32_t lastImuMs = 0, lastImuUs = 0, lastIrMs = 0;
uint16_t calSamples = 0;
float calMean = 0, calM2 = 0, calAccelMin = 1e6f, calAccelMax = 0;
uint32_t stateStarted = 0, calibrationStarted = 0, lastPrint = 0;

int16_t pwmApplied = 0, pwmRequested = 0;
uint32_t lastMotorMs = 0, motorZeroAt = 0;
float wheelRpm = 0, wheelRpmFiltered = 0;
uint32_t lastRpmMs = 0;
int32_t lastRpmCounts = 0;
uint32_t stuckSince = 0;
uint16_t consecutiveSoftLimit = 0;

bool traceOn = false;
uint32_t traceAt = 0;
bool laserIsOn = false;
uint32_t fireStarted = 0;

struct MotorPulse {
  int16_t pwm;
  uint16_t duration;
  uint32_t began, driveEnd, end, lastLog;
  int32_t initialCounts;
  float initialYaw, peakRpm, sumEndRpm;
  float uncalibratedBodyDeg; // integrates raw gyro for bench pulse only
  uint8_t endRpmSamples;
  bool coasting;
};
MotorPulse pulse = {};
uint32_t motorArmedUntil = 0;

struct TimedStats {
  bool active;
  uint32_t started, duration, lastPrint;
  uint32_t samples[3], low[3], changes[3], shorts[3];
  float yawStart, yawMin, yawMax, maxRate, temperatureStart;
};
TimedStats flicker = {}, drift = {};

void motorCoast() {
  analogWrite(MOTOR_IN1, 0);
  digitalWrite(MOTOR_IN2, LOW);
  pwmApplied = 0;
  pwmRequested = 0;
}

void motorSetHardware(int16_t command) {
  if (command == 0) {
    analogWrite(MOTOR_IN1, 0);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (command > 0) {
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, command);
  } else {
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_IN1, 255 + command); // inverse PWM: reverse/brake on IN-IN driver
  }
}

void laserOff() {
  digitalWrite(LASER_PIN, LOW);
  laserIsOn = false;
}

void enter(State next) {
  state = next;
  stateStarted = millis();
  Serial.print(F("STATE,")); Serial.print(stateName());
  Serial.print(F(",ms=")); Serial.println(stateStarted);
}

void clearFreshArc() {
  freshArc.valid = false;
  freshArc.centre = 0;
  freshArc.width = 0;
  freshArc.durationMs = 0;
  freshArc.measuredAt = 0;
  arcOpen = false;
  arcEntryAngle = 0;
  arcEntryAt = 0;
}


void fault(const __FlashStringHelper *reason) {
  if (state == FAULT) return;
  motorCoast(); laserOff(); calibrated = false;
  clearFreshArc();
  flicker.active = drift.active = false;
  enter(FAULT);
  Serial.print(F("FAULT,reason=")); Serial.println(reason);
  Serial.println(F("Motor off. Reference discarded. Inspect fault and explicitly send RECAL to recalibrate."));
}

// Stop using the previous measurement and explicitly exit the cone before
// the next full ON->OFF IR arc can be accepted.
void beginFreshSearch(const __FlashStringHelper *reason, bool retry = false) {
  if (retry && ++freshArcRetries > MAX_FRESH_ARC_RETRIES) {
    fault(F("FRESH_ARC_RETRY_LIMIT"));
    return;
  }
  motorCoast(); laserOff(); clearFreshArc();
  clearSince = 0;
  Serial.print(F("FRESH_ARC_REQUIRED,reason=")); Serial.print(reason);
  Serial.print(F(",retry=")); Serial.println(freshArcRetries);
  enter(WAIT_CLEAR);
}

void restartRun(bool forceRecalibration = false) {
  motorCoast(); laserOff(); clearFreshArc();
  fireAttempts = freshArcRetries = 0;
  flicker.active = drift.active = false;
  stuckSince = 0; consecutiveSoftLimit = 0; motorArmedUntil = 0;
  if (forceRecalibration) {
    // RECAL is the only command that deliberately discards a valid gyro bias.
    calibrated = false; everCalibrated = false;
    bias = 0; yaw = 0; yawRate = 0; badReads = 0;
    memset(ir, 0, sizeof(ir));
    enter(SEEK_TRIGGER);
    Serial.println(F("RECAL_REQUESTED,First qualified IR triggers 1000-sample gyro calibration."));
  } else if (calibrated && imuValid && state != FAULT) {
    // Preserve the calibration, gyro heading, sensor edge filters across runs.
    clearSince = 0;
    enter(WAIT_CLEAR);
    Serial.println(F("RUN_RESUME,Gyro bias retained; clear previous cone and await a new detection."));
  } else if (!everCalibrated && state != FAULT) {
    bias = 0; yaw = 0; yawRate = 0; badReads = 0;
    memset(ir, 0, sizeof(ir));
    enter(SEEK_TRIGGER);
    Serial.println(F("RUN_FIRST,Waiting for initial IR trigger to calibrate once."));
  } else {
    enter(FAULT);
    Serial.println(F("RUN_REFUSED,Reference invalid; use RECAL explicitly."));
  }
}

uint8_t occupied(uint16_t bits) {
  uint8_t result = 0;
  while (bits) { result += bits & 1; bits >>= 1; }
  return result;
}
uint8_t irMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i)
    if (ir[i].qualified) mask |= (1 << i);
  return mask;
}

bool mpuWrite(uint8_t address, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(address); Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuRead(float &rate, float &magnitude, float &temperature) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDRESS, (uint8_t)14, (uint8_t)true) != 14) return false;
  int16_t v[7];
  for (uint8_t i = 0; i < 7; ++i) {
    uint16_t h = Wire.read(), l = Wire.read();
    v[i] = (int16_t)((h << 8) | l);
  }
  const float ax = v[0] / ACC_LSB_G;
  const float ay = v[1] / ACC_LSB_G;
  const float az = v[2] / ACC_LSB_G;
  magnitude = sqrtf(ax*ax + ay*ay + az*az);
  temperature = v[3] / 340.0f + 36.53f;
  rate = v[6] / GYRO_LSB_DPS;
  return true;
}

void startCalibration() {
  motorCoast(); laserOff();
  calSamples = 0; calMean = 0; calM2 = 0;
  calAccelMin = 1e6f; calAccelMax = 0;
  calibrationStarted = millis();
  lastImuUs = micros();
  enter(CALIBRATING);
  Serial.print(F("CAL_START,samples=")); Serial.print(CAL_SAMPLES_REQUIRED);
  Serial.print(F(",sample_ms=")); Serial.println(GYRO_SAMPLE_MS);
}

void sampleCalibration(float r, float ag) {
  ++calSamples;
  const float delta = r - calMean;
  calMean += delta / calSamples;
  calM2 += delta * (r - calMean);
  if (ag < calAccelMin) calAccelMin = ag;
  if (ag > calAccelMax) calAccelMax = ag;
  if (calSamples < CAL_SAMPLES_REQUIRED) return;

  const float sd = sqrtf(calM2 / (calSamples - 1));
  if (sd > CAL_MAX_SD_DPS ||
      calAccelMin < 1.0f - CAL_ACCEL_G_TOLERANCE ||
      calAccelMax > 1.0f + CAL_ACCEL_G_TOLERANCE ||
      absolute(calMean) > 1.8f) {
    Serial.print(F("CAL_REJECT,sd=")); Serial.print(sd, 4);
    Serial.print(F(",mean_rate=")); Serial.print(calMean, 4);
    Serial.print(F(",min_g=")); Serial.print(calAccelMin, 3);
    Serial.print(F(",max_g=")); Serial.println(calAccelMax, 3);
    enter(WAIT_STILL);
    return;
  }
  bias = calMean; yaw = 0; yawRate = 0;
  calibrated = true; everCalibrated = true; lastImuUs = micros();
  clearSince = 0;
  Serial.print(F("CAL_OK,bias_dps=")); Serial.print(bias, 6);
  Serial.print(F(",noise_sd=")); Serial.print(sd, 5);
  Serial.print(F(",elapsed_ms=")); Serial.print(millis()-calibrationStarted);
  Serial.print(F(",temp_C=")); Serial.println(tempC, 2);
  Serial.println(F("CAL_ONCE,Calibration complete. Leave first detected cone, then wait for second IR trigger."));
  enter(WAIT_CLEAR);
}

void updateImu(uint32_t ms) {
  if ((uint32_t)(ms - lastImuMs) < GYRO_SAMPLE_MS) return;
  lastImuMs = ms;
  float r, ag, tc;
  const uint32_t nowUs = micros();
  if (!mpuRead(r, ag, tc)) {
    if (++badReads >= MAX_GYRO_READ_FAILURES) fault(F("MPU_READ_FAILURE"));
    return;
  }
  badReads = 0; imuValid = true;
  rawRate = r; accelG = ag; tempC = tc;
  if ((state == SEEK_TRIGGER || state == WAIT_STILL) &&
      (absolute(r) > PRECAL_MAX_BODY_RATE_DPS ||
       ag >= JOLT_ACCEL_G || ag <= JOLT_ACCEL_LOW_G)) {
    fault(F("PRECAL_EXCESSIVE_BODY_MOTION"));
  }
  if (state == MOTOR_TEST && !calibrated) {
    const float dtUncal = (uint32_t)(nowUs - lastImuUs) * 0.000001f;
    if (dtUncal < 0.120f) pulse.uncalibratedBodyDeg += r * dtUncal;
  }
  if (state == CALIBRATING) {
    sampleCalibration(r, ag);
  } else if (calibrated && state != FAULT) {
    const float dt = (uint32_t)(nowUs - lastImuUs) * 0.000001f;
    if (dt > 0.120f) fault(F("GYRO_SAMPLE_GAP"));
    else {
      yawRate = r - bias;
      yaw += yawRate * dt;
      if (absolute(r) >= JOLT_GYRO_DPS || ag >= JOLT_ACCEL_G || ag <= JOLT_ACCEL_LOW_G)
        fault(F("GYRO_SATURATION_OR_JOLT"));
    }
  }
  lastImuUs = nowUs;
}

void recordArc(float first, float last, uint32_t ms) {
  if (state != ACQUIRE_ARC || freshArc.valid) return;
  const float width = absolute(last - first);
  if (ms < MIN_ARC_MS || width < MIN_ARC_DEG || width > MAX_ARC_DEG ||
      last <= first) { // Reject reverse or partially-detected sweep edges
    Serial.print(F("REJECT_ARC,width_deg=")); Serial.print(width, 3);
    Serial.print(F(",duration_ms=")); Serial.println(ms);
    return;
  }
  freshArc.valid = true;
  freshArc.centre = (first + last) * 0.5f;
  freshArc.width = width;
  freshArc.durationMs = ms;
  freshArc.measuredAt = millis();
  Serial.print(F("FRESH_ARC,centre_yaw_deg=")); Serial.print(freshArc.centre, 3);
  Serial.print(F(",width_deg=")); Serial.print(width, 3);
  Serial.print(F(",duration_ms=")); Serial.println(ms);
  Serial.println(F("ARC_NOT_STORED,This centre applies only to the CURRENT aiming attempt."));
}

void updateIr(uint32_t ms) {
  if ((uint32_t)(ms-lastIrMs) < IR_SAMPLE_MS) return;
  lastIrMs = ms;
  for (uint8_t i = 0; i < IR_SENSOR_COUNT; ++i) {
    IrData &s = ir[i];
    const bool low = digitalRead(IR_PINS[i]) == LOW;
    ++s.total;
    if (low) ++s.low;
    if (!low && s.previousRaw && s.shortRun > 0 && s.shortRun <= 3) ++s.shortLowRuns;
    s.previousRaw = low;
    s.shortRun = low ? (s.shortRun < 255 ? s.shortRun + 1 : 255) : 0;
    s.bits = (uint16_t)((s.bits << 1) | (low ? 1 : 0));
    const uint8_t occupancy = occupied(s.bits);
    bool desired = s.qualified;
    if (!s.qualified && occupancy >= IR_ON_OCCUPANCY) desired = true;
    if (s.qualified && occupancy <= IR_OFF_OCCUPANCY) desired = false;
    if (desired == s.qualified) { s.pending = false; continue; }
    if (!s.pending || desired != s.pendingValue) {
      s.pending = true;
      s.pendingValue = desired;
      s.candidateAt = ms;
      s.candidateAngle = yaw;
    } else if ((uint32_t)(ms-s.candidateAt) >= (desired ? IR_ON_VERIFY_MS : IR_OFF_VERIFY_MS)) {
      s.qualified = desired;
      s.pending = false;
      ++s.transitions;
      if (desired) {
        s.acceptedAt = s.candidateAt;
        s.entryAngle = s.candidateAngle;
        if (state == ACQUIRE_ARC && i == PRIMARY_SENSOR) {
          arcOpen = true; arcEntryAngle = s.candidateAngle;
          arcEntryAt = s.candidateAt;
        }
      } else {
        if (state == ACQUIRE_ARC && i == PRIMARY_SENSOR && arcOpen) {
          arcOpen = false;
          recordArc(arcEntryAngle, s.candidateAngle,
                    (uint32_t)(s.candidateAt - arcEntryAt));
        }
      }
    }
  }
}

void updateRpm(uint32_t ms) {
  if ((uint32_t)(ms - lastRpmMs) < 100) return;
  const int32_t n = countsNow();
  if (lastRpmMs != 0) {
    const float dt = (ms-lastRpmMs) / 1000.0f;
    wheelRpm = (n-lastRpmCounts) * (60.0f/ENCODER_COUNTS_PER_REV) / dt;
    wheelRpmFiltered = wheelRpmFiltered * 0.65f + wheelRpm * 0.35f;
    if (absolute(wheelRpm) >= TRIP_ENCODER_RPM) fault(F("WHEEL_RPM_HARD_LIMIT"));
  }
  lastRpmMs = ms; lastRpmCounts = n;
}

void requestMotor(int16_t wheelTorque) {
  if (state == FAULT || state == STOPPED || state == FINISHED || state == CALIBRATING ||
      state == FIRE || state == POST_FIRE || state == BOOT_DELAY || state == MOTOR_TEST) {
    pwmRequested = 0;
    return;
  }
  int16_t output = constrain(wheelTorque, -(int16_t)AUTO_MAX_PWM, (int16_t)AUTO_MAX_PWM);
  if (output != 0 && abs(output) < AUTO_MIN_PWM) output = (output < 0 ? -AUTO_MIN_PWM : AUTO_MIN_PWM);
  // Never demand additional speed in the direction where wheel momentum is saturated.
  if (wheelRpmFiltered > WHEEL_SOFT_LIMIT_RPM && output > 0) output = 0;
  if (wheelRpmFiltered < -WHEEL_SOFT_LIMIT_RPM && output < 0) output = 0;
  pwmRequested = output;
}

void requestYawRate(float demandedRate, bool uncalibrated = false) {
  const float observed = uncalibrated ? rawRate : yawRate;
  const float delta = demandedRate - observed;
  if (absolute(demandedRate) < 0.01f && absolute(observed) < REST_RATE_DPS) {
    requestMotor(0);
    return;
  }
  const float torque = clipf(delta*MOTOR_RATE_KP, -AUTO_MAX_PWM, AUTO_MAX_PWM);
  requestMotor((int16_t)(torque * BODY_ACCEL_SIGN_FOR_MOTOR_FORWARD));
}

void updateMotor(uint32_t ms) {
  if ((uint32_t)(ms-lastMotorMs) < MOTOR_CONTROL_MS) return;
  lastMotorMs = ms;
  if (state == FAULT || state == STOPPED || state == FINISHED || state == CALIBRATING ||
      state == FIRE || state == POST_FIRE || state == BOOT_DELAY || state == MOTOR_TEST) {
    if (state != MOTOR_TEST && pwmApplied != 0) motorCoast();
    return;
  }
  int16_t target = pwmRequested;
  // Brief zero-current interval before reversing torque direction.
  if (pwmApplied && target && ((pwmApplied < 0) != (target < 0))) {
    pwmApplied = 0;
    motorSetHardware(0);
    motorZeroAt = ms;
    return;
  }
  if ((uint32_t)(ms-motorZeroAt) < MOTOR_ZERO_CROSS_MS) target = 0;
  const int16_t diff = target - pwmApplied;
  if (diff > MOTOR_PWM_SLEW_PER_TICK) pwmApplied += MOTOR_PWM_SLEW_PER_TICK;
  else if (diff < -(int16_t)MOTOR_PWM_SLEW_PER_TICK) pwmApplied -= MOTOR_PWM_SLEW_PER_TICK;
  else pwmApplied = target;
  motorSetHardware(pwmApplied);
  if (abs(pwmApplied) >= 65 && absolute(wheelRpmFiltered) < 1.5f) {
    if (!stuckSince) stuckSince = ms;
    else if ((uint32_t)(ms-stuckSince) > 1600) fault(F("ENCODER_STALLED_OR_UNPLUGGED"));
  } else stuckSince = 0;
}

bool arcIsFresh(uint32_t ms) {
  return freshArc.valid && (uint32_t)(ms - freshArc.measuredAt) <= FRESH_ARC_MAX_AGE_MS;
}

bool aimAligned() {
  if (!calibrated || !imuValid || !arcIsFresh(millis())) return false;
  const float e = wrap180(freshArc.centre - yaw);
  return absolute(e) <= ALIGN_TOL_DEG && absolute(yawRate) <= REST_RATE_DPS &&
         absolute(wheelRpmFiltered) <= FIRE_MAX_WHEEL_RPM && ir[PRIMARY_SENSOR].qualified;
}

void printArc() {
  Serial.print(F("ARC_STATUS,valid=")); Serial.print(freshArc.valid ? 1 : 0);
  if (!freshArc.valid) { Serial.println(F(",message=No current arc; no target map exists.")); return; }
  Serial.print(F(",centre_yaw_deg=")); Serial.print(freshArc.centre, 3);
  Serial.print(F(",width_deg=")); Serial.print(freshArc.width, 3);
  Serial.print(F(",age_ms=")); Serial.print(millis()-freshArc.measuredAt);
  Serial.print(F(",usable=")); Serial.println(arcIsFresh(millis()) ? 1 : 0);
}

void updateController(uint32_t ms) {
  if (state == BOOT_DELAY) {
    if ((uint32_t)(ms-stateStarted) >= BOOT_GRACE_MS) {
      if (AUTORUN_ON_BOOT) restartRun(); else enter(STOPPED);
    }
    return;
  }
  if (state == SEEK_TRIGGER) {
    requestYawRate(SEEK_RATE_DPS, true);
    if (irMask() != 0) {
      motorCoast(); clearSince = 0;
      enter(WAIT_STILL);
      Serial.println(F("FIRST_IR_CONFIRMED,Stopping chassis before gyro bias calibration."));
    } else if ((uint32_t)(ms-stateStarted) > SEEK_TIMEOUT_MS)
      fault(F("SEEK_TIMEOUT_NO_IR_OR_INSUFFICIENT_WHEEL_MOMENTUM"));
  } else if (state == WAIT_STILL) {
    // Body-rate damping does not require gyro calibration; it uses raw rate.
    // The flywheel need not be fully stopped, but the body must be still.
    requestYawRate(0, true);
    if (absolute(rawRate) < REST_RATE_DPS &&
        accelG >= 1.0f-CAL_ACCEL_G_TOLERANCE &&
        accelG <= 1.0f+CAL_ACCEL_G_TOLERANCE &&
        absolute(wheelRpmFiltered) < FIRE_MAX_WHEEL_RPM) {
      if (!clearSince) clearSince = ms;
      if ((uint32_t)(ms-clearSince) >= FIRST_IR_STILL_MS) startCalibration();
    } else clearSince = 0;
    if ((uint32_t)(ms-stateStarted) > STILL_TIMEOUT_MS)
      fault(F("COULD_NOT_SETTLE_BODY_FOR_CALIBRATION"));
  } else if (state == CALIBRATING) {
    if ((uint32_t)(ms-stateStarted) > CAL_TIMEOUT_MS)
      fault(F("CALIBRATION_TIMEOUT"));
  } else if (state == WAIT_CLEAR) {
    // Never count the original calibration cone, or the just-fired cone.
    // All sensors must be clear for STARTUP_CLEAR_MS (200ms).
    requestYawRate(SCAN_RATE_DPS);
    if (irMask() == 0 && !ir[PRIMARY_SENSOR].pending) {
      if (!clearSince) clearSince = ms;
      if ((uint32_t)(ms-clearSince) >= STARTUP_CLEAR_MS) {
        clearFreshArc();
        scanStartYaw = yaw;
        enter(SEEK_NEXT_IR);
        Serial.println(F("IR_CLEAR_CONFIRMED,Now waiting for NEW primary IR trigger."));
      }
    } else clearSince = 0;
    if ((uint32_t)(ms-stateStarted) > SEEK_TIMEOUT_MS)
      fault(F("EXISTING_IR_CONE_NEVER_CLEARED"));
  } else if (state == SEEK_NEXT_IR) {
    requestYawRate(SCAN_RATE_DPS);
    if (ir[PRIMARY_SENSOR].qualified) {
      // Capture the original filtered ON edge, not just the confirmation time.
      arcOpen = true;
      arcEntryAngle = ir[PRIMARY_SENSOR].entryAngle;
      arcEntryAt = ir[PRIMARY_SENSOR].acceptedAt;
      enter(ACQUIRE_ARC);
      Serial.print(F("NEXT_IR_TRIGGER,entry_angle_deg=")); Serial.print(arcEntryAngle,3);
      Serial.print(F(",entry_ms=")); Serial.println(arcEntryAt);
      Serial.println(F("ARC_STARTED,Continue scan through verified IR OFF edge."));
      return;
    }
    if (yaw-scanStartYaw < -12.0f) {
      fault(F("SCAN_WRONG_MOTOR_POLARITY")); return;
    }
    if ((uint32_t)(ms-stateStarted) > SURVEY_TIMEOUT_MS || yaw-scanStartYaw > 400.0f)
      fault(F("NEXT_IR_NOT_FOUND_OR_WHEEL_MOMENTUM_LIMIT"));
  } else if (state == ACQUIRE_ARC) {
    requestYawRate(SCAN_RATE_DPS);
    const float travelled = yaw - scanStartYaw;
    if (travelled < -12.0f) { fault(F("SCAN_WRONG_MOTOR_POLARITY")); return; }
    if (freshArc.valid) {
      motorCoast();
      Serial.println(F("ARC_SEARCH_COMPLETE,Fresh cone centre calculated."));
      enter(AIM);
      return;
    }
    if ((uint32_t)(ms-stateStarted) > SURVEY_TIMEOUT_MS || travelled > 400.0f)
      fault(F("FRESH_ARC_NOT_COMPLETED_OR_WHEEL_MOMENTUM_LIMIT"));
  } else if (state == AIM) {
    if (!arcIsFresh(ms)) {
      beginFreshSearch(F("AIM_ARC_EXPIRED"), true); return;
    }
    const float error = wrap180(freshArc.centre - yaw);
    const float wanted = clipf(error*AIM_POS_KP, -AIM_RATE_LIMIT_DPS, AIM_RATE_LIMIT_DPS);
    requestYawRate(wanted);
    if (aimAligned()) {
      motorCoast(); alignedSince = ms;
      enter(SETTLE);
    }
    if ((uint32_t)(ms-stateStarted) > AIM_TIMEOUT_MS)
      fault(F("AIM_TIMEOUT_WRONG_SIGN_OR_INSUFFICIENT_TORQUE"));
  } else if (state == SETTLE) {
    // Always coast while holding alignment; if it drifts, resume closed-loop aiming.
    pwmRequested = 0;
    if (!arcIsFresh(ms)) { beginFreshSearch(F("SETTLE_ARC_EXPIRED"), true); return; }
    if (!aimAligned()) { enter(AIM); return; }
    if ((uint32_t)(ms-alignedSince) >= SETTLE_FOR_FIRE_MS) {
      fireStarted = ms;
#if ENABLE_AUTO_LASER
      digitalWrite(LASER_PIN, HIGH);
      laserIsOn = true;
      Serial.println(F("FIRE_START,LASER_ON"));
#else
      Serial.println(F("FIRE_START,SIMULATED_LASER_DISABLED"));
#endif
      enter(FIRE);
    }
  } else if (state == FIRE) {
    pwmRequested = 0;
    if (!aimAligned()) {
      laserOff();
      ++fireAttempts;
      Serial.println(F("FIRE_ABORT,ALIGNMENT_OR_ARC_VALIDITY_LOST"));
      if (fireAttempts >= MAX_FIRE_ATTEMPTS) {
        fault(F("REPEATED_FIRE_ALIGNMENT_LOSS"));
      } else beginFreshSearch(F("FIRE_ABORT_REMEASURE_ARC"), true);
      return;
    }
    if ((uint32_t)(ms-fireStarted) >= LASER_FIRE_MS) {
      laserOff();
      Serial.println(F("FIRE_COMPLETE,Fresh IR arc was used for this engagement."));
      enter(POST_FIRE);
    }
  } else if (state == POST_FIRE) {
    if ((uint32_t)(ms-stateStarted) >= POST_FIRE_MS) {
      fireAttempts = 0;
      freshArcRetries = 0;
      if (AUTO_REPEAT) {
        Serial.println(F("SHOT_COMPLETE,Calibration retained. Exit current arc and seek next trigger."));
        beginFreshSearch(F("AFTER_FIRE_EXIT_CONE_AND_SEEK"));
      }
      else {
        motorCoast(); laserOff(); clearFreshArc();
        Serial.println(F("ENGAGEMENT_COMPLETE,Arc discarded; send RUN for another run or RESCAN to reuse gyro calibration."));
        enter(FINISHED);
      }
    }
  }
}

void printStatus() {
  Serial.print(F("STATUS,state=")); Serial.print(stateName());
  Serial.print(F(",yaw=")); Serial.print(yaw,2);
  Serial.print(F(",dps=")); Serial.print(yawRate,2);
  Serial.print(F(",bias=")); Serial.print(bias,5);
  Serial.print(F(",raw_dps=")); Serial.print(rawRate,2);
  Serial.print(F(",wheel_rpm=")); Serial.print(wheelRpmFiltered,2);
  Serial.print(F(",pwm=")); Serial.print(pwmApplied);
  Serial.print(F(",ir_mask=")); Serial.print(irMask(), BIN);
  Serial.print(F(",fresh_arc=")); Serial.print(arcIsFresh(millis()) ? 1 : 0);
  Serial.print(F(",arc_error_deg="));
  if (arcIsFresh(millis())) Serial.print(wrap180(freshArc.centre-yaw), 2);
  else Serial.print(F("NA"));
  Serial.print(F(",accel_g=")); Serial.print(accelG,3);
  Serial.print(F(",calibrated=")); Serial.print(calibrated?1:0);
  Serial.print(F(",ever_calibrated=")); Serial.println(everCalibrated?1:0);
}

void startTimed(TimedStats &t, uint32_t secs) {
  memset(&t, 0, sizeof(t));
  t.active = true;
  t.started = t.lastPrint = millis();
  t.duration = secs*1000UL;
  t.yawStart = t.yawMin = t.yawMax = yaw;
  t.temperatureStart = tempC;
  for (uint8_t i=0; i<IR_SENSOR_COUNT; i++) {
    t.samples[i]=ir[i].total;
    t.low[i]=ir[i].low;
    t.changes[i]=ir[i].transitions;
    t.shorts[i]=ir[i].shortLowRuns;
  }
}
void updateTimed(uint32_t ms) {
  if (flicker.active && (uint32_t)(ms-flicker.started) >= flicker.duration) {
    flicker.active = false;
    for (uint8_t i=0; i<IR_SENSOR_COUNT; ++i) {
      const uint32_t samples=ir[i].total-flicker.samples[i];
      const uint32_t low=ir[i].low-flicker.low[i];
      Serial.print(F("RESULT_FLICKER,sensor=")); Serial.print(i);
      Serial.print(F(",raw_low_pct=")); Serial.print(samples ? 100.0f*low/samples : 0.0f,2);
      Serial.print(F(",qualified_edges=")); Serial.print(ir[i].transitions-flicker.changes[i]);
      Serial.print(F(",short_low_runs=")); Serial.println(ir[i].shortLowRuns-flicker.shorts[i]);
    }
  }
  if (drift.active) {
    if (yaw < drift.yawMin) drift.yawMin = yaw;
    if (yaw > drift.yawMax) drift.yawMax = yaw;
    if (absolute(yawRate) > drift.maxRate) drift.maxRate = absolute(yawRate);
    if ((uint32_t)(ms-drift.lastPrint) >= 1000) {
      drift.lastPrint = ms;
      Serial.print(F("DRIFT_SAMPLE,net_deg=")); Serial.print(yaw-drift.yawStart,5);
      Serial.print(F(",temp_C=")); Serial.println(tempC,2);
    }
    if ((uint32_t)(ms-drift.started) >= drift.duration) {
      drift.active = false;
      Serial.print(F("RESULT_DRIFT,net_deg=")); Serial.print(yaw-drift.yawStart,5);
      Serial.print(F(",range_deg=")); Serial.print(drift.yawMax-drift.yawMin,5);
      Serial.print(F(",peak_dps=")); Serial.print(drift.maxRate,3);
      Serial.print(F(",delta_temp_C=")); Serial.println(tempC-drift.temperatureStart,3);
    }
  }
}

void updatePulse(uint32_t ms) {
  if (state != MOTOR_TEST) return;
  if (!pulse.coasting && (int32_t)(ms-pulse.driveEnd) >= 0) {
    motorCoast(); pulse.coasting = true;
    Serial.println(F("MOTOR_COAST"));
  }
  const float rpm = absolute(wheelRpmFiltered);
  if (rpm > pulse.peakRpm) pulse.peakRpm = rpm;
  if ((uint32_t)(ms-pulse.lastLog) >= 100) {
    pulse.lastLog = ms;
    if (!pulse.coasting && ms+350 >= pulse.driveEnd) {
      pulse.sumEndRpm += rpm;
      ++pulse.endRpmSamples;
    }
    Serial.print(F("PULSE_DATA,elapsed_ms=")); Serial.print(ms-pulse.began);
    Serial.print(F(",yaw=")); Serial.print(yaw,3);
    Serial.print(F(",rate=")); Serial.print(yawRate,3);
    Serial.print(F(",wheel_rpm=")); Serial.print(wheelRpmFiltered,2);
    Serial.print(F(",counts=")); Serial.println(countsNow()-pulse.initialCounts);
  }
  if ((int32_t)(ms-pulse.end) >= 0) {
    motorCoast();
    const float wheelDeg=(countsNow()-pulse.initialCounts)*(360.0f/ENCODER_COUNTS_PER_REV);
    const float bodyDeg=calibrated ? yaw-pulse.initialYaw : pulse.uncalibratedBodyDeg;
    Serial.print(F("RESULT_PULSE,pwm=")); Serial.print(pulse.pwm);
    Serial.print(F(",drive_ms=")); Serial.print(pulse.duration);
    Serial.print(F(",wheel_delta_deg=")); Serial.print(wheelDeg,3);
    Serial.print(F(",body_delta_deg=")); Serial.print(bodyDeg,3);
    Serial.print(F(",gyro_calibrated=")); Serial.print(calibrated ? 1 : 0);
    Serial.print(F(",K_PROVISIONAL="));
    if (absolute(wheelDeg)>1.0f) Serial.print(-bodyDeg/wheelDeg,5);
    else Serial.print(F("NA"));
    Serial.print(F(",peak_rpm=")); Serial.print(pulse.peakRpm,2);
    Serial.print(F(",end_drive_mean_rpm="));
    if (pulse.endRpmSamples) Serial.print(pulse.sumEndRpm/pulse.endRpmSamples,2);
    else Serial.print(F("NA"));
    Serial.print(F(",end_rpm=")); Serial.println(wheelRpmFiltered,2);
    Serial.println(F("TEST_NOTE,Not a valid K or no-load RPM unless both bodies settle / RPM reaches plateau."));
    clearFreshArc();  // motor test does not force recalibration
    enter(STOPPED);
  }
}

void help() {
  Serial.println(F("HELP,115200 Newline | RUN STOP RECAL STATUS ARC RESCAN TRACE ON/OFF"));
  Serial.println(F("HELP,FLICKER 20 | DRIFT 60 (STOPPED only)"));
  Serial.println(F("HELP,ARM MOTOR | PULSE +/-65 150 (STOPPED; direction + / -)"));
  Serial.println(F("INFO,Motor D6 PWM / D7 digital, encoder D9 D10, gyro A4 A5"));
  Serial.println(F("INFO,First IR calibrates once; second IR starts first arc; then loop after every shot."));
  Serial.println(F("INFO,After FIRE: exit cone -> await new IR -> fresh arc. RUN preserves calibration."));
  Serial.println(F("INFO,RECAL explicitly recalibrates; no map/EEPROM."));
  Serial.print(F("INFO,Auto laser simulation by default. AUTO_REPEAT="));
  Serial.println(AUTO_REPEAT ? F("true") : F("false"));
}
char input[52];
uint8_t used = 0;
void serialCommand(char *buf) {
  char *cmd = strtok(buf, " \t");
  if (!cmd) return;
  for (char *p=cmd;*p;p++) if (*p>='a'&&*p<='z') *p -= 32;
  char *a=strtok(NULL," \t"), *b=strtok(NULL," \t");
  if (!strcmp(cmd,"HELP") || !strcmp(cmd,"?")) help();
  else if (!strcmp(cmd,"STOP") || !strcmp(cmd,"X")) {
    motorCoast(); laserOff(); flicker.active=drift.active=false;
    enter(STOPPED); Serial.println(F("STOPPED,No motor or laser drive."));
  } else if (!strcmp(cmd,"RUN") || !strcmp(cmd,"R")) restartRun();
  else if (!strcmp(cmd,"RECAL") || !strcmp(cmd,"RESET")) restartRun(true);
  else if (!strcmp(cmd,"STATUS")) printStatus();
  else if (!strcmp(cmd,"ARC")) printArc();
  else if (!strcmp(cmd,"RESCAN")) {
    if (!calibrated || !imuValid || state==FAULT || state==MOTOR_TEST ||
        state==CALIBRATING || state==FIRE || state==POST_FIRE)
      Serial.println(F("ERR,RESCAN needs a valid calibrated gyro and safe state. Use RECAL if gyro reference is lost."));
    else { freshArcRetries = 0; beginFreshSearch(F("SERIAL_RESCAN")); }
  } else if (!strcmp(cmd,"TRACE")) {
    traceOn = a && (a[0]=='O'||a[0]=='o') && (a[1]=='N'||a[1]=='n');
    Serial.println(traceOn?F("TRACE_ON"):F("TRACE_OFF"));
  } else if (!strcmp(cmd,"FLICKER") || !strcmp(cmd,"DRIFT")) {
    int secs=a?atoi(a):0;
    if (secs<5||secs>180) Serial.println(F("ERR,Duration 5..180 seconds"));
    else if (!strcmp(cmd,"DRIFT") && (state!=STOPPED || !calibrated))
      Serial.println(F("ERR,DRIFT requires stopped chassis and valid calibration"));
    else if (!strcmp(cmd,"FLICKER")) {
      startTimed(flicker,secs); Serial.println(F("FLICKER_START"));
    } else { startTimed(drift,secs); Serial.println(F("DRIFT_START,Do not move")); }
  } else if (!strcmp(cmd,"ARM") && a && (!strcmp(a,"MOTOR")||!strcmp(a,"motor"))) {
    if (state!=STOPPED) Serial.println(F("ERR,STOP first"));
    else { motorArmedUntil=millis()+15000UL; Serial.println(F("ARMED_MOTOR,15s")); }
  } else if (!strcmp(cmd,"PULSE")) {
    const int pwm=a?atoi(a):0, duration=b?atoi(b):0;
    if (state!=STOPPED || (int32_t)(motorArmedUntil-millis())<=0 ||
        pwm==0 || abs(pwm)>MAX_TEST_PWM || duration<40 || duration>MAX_TEST_DRIVE_MS)
      Serial.println(F("ERR,STOP; ARM MOTOR then PULSE -180..-1 or 1..180, ms 40..1200"));
    else {
      motorArmedUntil=0; memset(&pulse,0,sizeof(pulse));
      pulse.pwm=pwm; pulse.duration=duration;
      pulse.began=pulse.lastLog=millis();
      pulse.driveEnd=pulse.began+duration;
      pulse.end=pulse.driveEnd+COAST_RECORD_MS;
      pulse.initialYaw=yaw; pulse.initialCounts=countsNow();
      enter(MOTOR_TEST);
      motorSetHardware(pwm);
      pwmApplied=pwm;
      Serial.println(F("PULSE_START"));
    }
  } else Serial.println(F("ERR,Unknown command. HELP"));
}
void readCommands() {
  while (Serial.available()) {
    const char c=Serial.read();
    if (c=='\n'||c=='\r') {
      if (used) { input[used]=0; serialCommand(input); used=0; }
    } else if (used<sizeof(input)-1) input[used++]=c;
    else used=0;
  }
}

void sentinelSetup() {
  Serial.begin(115200);
  for (uint8_t i=0; i<IR_SENSOR_COUNT; ++i) pinMode(IR_PINS[i],INPUT_PULLUP);
  pinMode(LASER_PIN,OUTPUT); pinMode(STATUS_LED,OUTPUT);
  pinMode(MOTOR_IN1,OUTPUT); pinMode(MOTOR_IN2,OUTPUT);
  pinMode(ENCODER_A,INPUT_PULLUP); pinMode(ENCODER_B,INPUT_PULLUP);
  motorCoast(); laserOff();
  encoderPrevA = digitalRead(ENCODER_A)==HIGH;
  PCICR |= _BV(PCIE0); PCMSK0 |= _BV(PCINT1);
  Wire.begin(); Wire.setClock(100000L);
  delay(100);
  if (!mpuWrite(0x6B,0x00) || !mpuWrite(0x1A,0x03) ||
      !mpuWrite(0x1B,0x00) || !mpuWrite(0x1C,0x00) ||
      !mpuWrite(0x19,0x04)) { // 200 Hz nominal MPU output for 5ms sampling
    Serial.println(F("FATAL,MPU_NOT_FOUND_0x68"));
    enter(FAULT); return;
  }
  imuValid=true;
  enter(BOOT_DELAY);
  Serial.println(F("MQ_SENTINEL,CONTINUOUS_SINGLE_CALIBRATION,115200"));
  Serial.print(F("SENSORS,number=")); Serial.print(IR_SENSOR_COUNT);
  Serial.print(F(",primary_pin=")); Serial.println(IR_PINS[PRIMARY_SENSOR]);
  Serial.print(F("LASER_AUTO_ENABLED,")); Serial.println(ENABLE_AUTO_LASER);
  Serial.println(F("CHECK: PMODE=HIGH; motor sign; encoder counts/rev; secure flywheel!"));
  Serial.println(F("CALIBRATE_ON_FIRST_IR,Then require a second trigger to start the first arc."));
  Serial.println(F("NO_MAP,After each shot leave beam and reacquire; never recalibrate automatically."));
  help();
  lastImuUs=micros();
}

void sentinelLoop() {
  const uint32_t ms=millis();
  readCommands();
  if (state!=FAULT) updateImu(ms);
  updateIr(ms);
  updateRpm(ms);
  if (state!=FAULT && state!=STOPPED && state!=MOTOR_TEST)
    updateController(ms);
  updateMotor(ms);
  updatePulse(ms);
  updateTimed(ms);
  if (traceOn && (uint32_t)(ms-traceAt)>=250) { traceAt=ms; printStatus(); }
  digitalWrite(STATUS_LED, state==FAULT ? ((ms/120)%2) : (state==FINISHED || state==FIRE));
}
