/*
 MQ SENTINEL — MPU-6050 + IR SENSOR-ONLY DIAGNOSTICS
 Arduino Nano V3 (ATmega328P), 115200 baud, newline line ending.
 No motor actuation. No EEPROM. No IRremote or extra libraries.

 Configure these four feature switches before uploading:
   1 = hardware connected, 0 = entirely omit from the sketch.
 The laser is DISABLED by default, and additionally requires ARM.

 MPU6050   SDA=A4, SCL=A5, GND=GND; regulated supply to module VCC.
           Sensor I2C logic is 3.3 V: use a bidirectional level shifter
           between a 5 V Nano A4/A5 and MPU SDA/SCL (unless confirmed
           onboard level shifting for this exact breakout).
 IR        left D2 / centre D3 / right D4, active LOW.
 Motor     driver inputs D6 and D7 are held LOW (coast), NEVER driven.
 Laser     diode enable D8, active HIGH (verify with actual interface).
 Encoder   A D9 / B D10, optional; counts FLYWHEEL, NOT BODY YAW.

 Mount MPU rigidly on satellite BODY, not on reaction wheel, with its
 configured gyro yaw axis parallel to the satellite suspension axis.
 Sensor heading 0 is an arbitrary boot-relative direction, not north.
*/

#define USE_MPU6050 1
#define USE_IR_SENSORS 1
#define USE_ENCODER 1
#define ENABLE_LASER 0   // CHANGE TO 1 ONLY FOR SUPERVISED LASER TESTS

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

const uint8_t PIN_IR_L = 2;
const uint8_t PIN_IR_C = 3;
const uint8_t PIN_IR_R = 4;
const uint8_t PIN_MOTOR_IN1 = 6;
const uint8_t PIN_MOTOR_IN2 = 7;
const uint8_t PIN_LASER = 8;
const uint8_t PIN_ENCODER_A = 9;
const uint8_t PIN_ENCODER_B = 10;
const uint8_t PIN_LED = 13;

// Tune after empirical tests rather than guessing receiver sensitivity.
const uint8_t MPU_ADDR = 0x68; // 0x69 if AD0 tied to 3.3 V
const uint8_t YAW_GYRO_AXIS = 2; // 0=X, 1=Y, 2=Z; select BODY yaw axis
const float YAW_SIGN = +1.0f;    // reverse to -1 if observed sign is wrong
const unsigned long IMU_PERIOD_US = 10000UL; // 100 Hz
const unsigned long IR_ACTIVE_HOLD_MS = 75UL; // time since a LOW/edge
const unsigned long IR_STABLE_MS = 250UL;
const unsigned long RATE_STABLE_MS = 300UL;
const float RATE_STABLE_LIMIT_DPS = 0.65f;
const unsigned long LASER_DURATION_MS = 2000UL;
const unsigned long DRIFT_DURATION_MS = 60000UL;
const float GYRO_LSB_PER_DPS = 131.0f; // full-scale +/-250 dps
const float ACC_LSB_PER_G = 16384.0f;  // full-scale +/-2 g
const float GYRO_EMA_ALPHA = 0.15f;
const float ENCODER_COUNTS_PER_REV_APPROX = 700.0f; // MUST MEASURE: decoder is x4

// -------- INTERRUPT-DRIVEN IR TRANSITIONS (D2-D4, PCINT2) ----------
volatile uint16_t isrIrFalling[3] = {0,0,0};
volatile uint8_t isrIrPrevious = 0x1C;

#if USE_IR_SENSORS
ISR(PCINT2_vect) {
  uint8_t nowPins = PIND & 0x1C;
  uint8_t falls = (isrIrPrevious & (uint8_t)~nowPins) & 0x1C;
  if (falls & _BV(PD2)) ++isrIrFalling[0];
  if (falls & _BV(PD3)) ++isrIrFalling[1];
  if (falls & _BV(PD4)) ++isrIrFalling[2];
  isrIrPrevious = nowPins;
}
#endif

// -------- PASSIVE ENCODER (D9/D10, PCINT0) ----------
volatile long isrEncCount = 0;
volatile uint8_t isrEncPrev = 0;
#if USE_ENCODER
const int8_t QUAD_STEPS[16] = {
  0,-1, 1, 0,
  1, 0, 0,-1,
 -1, 0, 0, 1,
  0, 1,-1, 0
};
ISR(PCINT0_vect) {
  uint8_t state = (PINB >> 1) & 0x03; // PB1=D9, PB2=D10
  isrEncCount += QUAD_STEPS[(isrEncPrev << 2) | state];
  isrEncPrev = state;
}
#endif

// -------- SENSOR AND ORIENTATION STATE ----------
bool imuOK = false;
float biasRaw[3] = {0,0,0};
float gyroDps[3] = {0,0,0};
float accelG[3] = {0,0,0};
float tempC = 0.0f;
float yawIntegrated = 0.0f; // CONTINUOUS integration; not constrained to 0-360
float yawCorrection = 0.0f; // IR-referenced correction: effective heading = sum
float filteredYawRate = 0.0f;
unsigned long lastImuUs = 0;
unsigned long lastImuAttemptUs = 0;
unsigned long lastImuRecoveryMs = 0;
uint16_t imuReadFailures = 0;

uint8_t irRawMask = 0;     // L C R = 100,010,001; 1 = active
uint8_t irMask = 0;        // 75-ms qualifying window; may combine signals
unsigned long lastIRSeenMs[3] = {0,0,0};
uint16_t prevIrEdges[3] = {0,0,0};
uint16_t irEdgeDeltas[3] = {0,0,0};
unsigned long irMaskSinceMs = 0;
unsigned long centreAcceptedSinceMs = 0; // valid 010<->111 transitions do NOT reset stability
unsigned long lastMaskSampleMs = 0;
unsigned long rateStableSinceMs = 0;
uint8_t previousIrMask = 0;
uint32_t maskHistogram[8] = {0,0,0,0,0,0,0,0};
unsigned long lastHistMs = 0;

long encCount = 0;
long lastEncCount = 0;
float encRpm = 0;
unsigned long lastEncMs = 0;

float landmarks[3] = {0,0,0};
bool landmarkValid[3] = {false,false,false};

uint8_t testMode = 0;
unsigned long testStartMs = 0;
float testStartHeading = 0.0f;
unsigned long lastPrintMs = 0;
unsigned long reportPeriodMs = 200;
uint8_t previousAlignment = 0;
unsigned long alignmentFirstMs = 0;
bool laserArmed = false;
bool laserActive = false;
unsigned long laserStartedMs = 0;
uint16_t completedDwell = 0;
uint16_t rejectedDwell = 0;

char inputLine[54];
uint8_t lineLength = 0;

// -------- SUPPORT ----------
float heading() { return yawIntegrated + yawCorrection; }
float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}
float absf(float v) { return v < 0 ? -v : v; }

void printMask(uint8_t mask) {
  Serial.print((mask & 4) ? '1' : '0');
  Serial.print((mask & 2) ? '1' : '0');
  Serial.print((mask & 1) ? '1' : '0');
}

bool imuWrite(uint8_t reg, uint8_t val) {
#if USE_MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
#else
  (void)reg; (void)val;
  return false;
#endif
}

bool imuRead14(int16_t *out) {
#if USE_MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write((uint8_t)0x3B);   // ACCEL_XOUT_H..GYRO_ZOUT_L
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) {
    uint16_t hi = (uint8_t)Wire.read();
    uint16_t lo = (uint8_t)Wire.read();
    out[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
#else
  (void)out;
  return false;
#endif
}

bool initImu() {
#if USE_MPU6050
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  // Wake, gyro PLL, 100 Hz sample, ~44 Hz DLPF, +/-250 dps, +/-2g.
  if (!imuWrite(0x6B, 0x01)) return false; // PWR_MGMT_1
  delay(50);
  if (!imuWrite(0x1A, 0x03)) return false; // CONFIG: DLPF 3
  if (!imuWrite(0x19, 0x09)) return false; // SMPLRT_DIV=9 => 100 Hz
  if (!imuWrite(0x1B, 0x00)) return false; // GYRO_CONFIG
  if (!imuWrite(0x1C, 0x00)) return false; // ACCEL_CONFIG
  int16_t data[7];
  if (!imuRead14(data)) return false;
  lastImuUs = micros();
  lastImuAttemptUs = lastImuUs;
  return true;
#else
  return false;
#endif
}

// Keep the satellite STILL when calling CAL: static gyro bias not true yaw.
bool calibrateGyro() {
#if USE_MPU6050
  if (!imuOK) { Serial.println(F("ERR: MPU-6050 unavailable")); return false; }
  Serial.println(F("CAL: DO NOT MOVE satellite; sampling 300 points (~3 s)."));
  double sum[3] = {0,0,0};
  double sq[3] = {0,0,0};
  int16_t r[7];
  for (uint16_t i = 0; i < 300; i++) {
    if (!imuRead14(r)) {
      Serial.println(F("CAL failed: I2C read"));
      return false;
    }
    for (uint8_t a = 0; a < 3; ++a) {
      double v = r[4+a];
      sum[a] += v;
      sq[a] += v*v;
    }
    delay(10);
  }
  double maxSigma = 0;
  for (uint8_t a = 0; a < 3; a++) {
    double mean = sum[a] / 300.0;
    double var = sq[a] / 300.0 - mean * mean;
    if (var < 0.0) var = 0.0;
    double sigma = sqrt(var) / GYRO_LSB_PER_DPS;
    if (sigma > maxSigma) maxSigma = sigma;
  }
  if (maxSigma > 0.8) {
    Serial.print(F("CAL rejected (moving/vibration), max sigma dps="));
    Serial.println(maxSigma, 3);
    return false;
  }
  for (uint8_t a = 0; a < 3; a++) biasRaw[a] = (float)(sum[a] / 300.0);
  filteredYawRate = 0;
  lastImuUs = micros();
  Serial.print(F("CAL OK; bias dps x,y,z="));
  for (uint8_t a = 0; a < 3; a++) {
    Serial.print(biasRaw[a] / GYRO_LSB_PER_DPS, 4);
    if (a != 2) Serial.print(',');
  }
  Serial.println();
  return true;
#else
  Serial.println(F("ERR: USE_MPU6050=0"));
  return false;
#endif
}

void pollImu() {
#if USE_MPU6050
  unsigned long nowUs = micros();
  if ((unsigned long)(nowUs - lastImuAttemptUs) < IMU_PERIOD_US) return;
  lastImuAttemptUs = nowUs;
  if (!imuOK) {
    if (millis() - lastImuRecoveryMs >= 1000UL) {
      lastImuRecoveryMs = millis();
      imuOK = initImu();
      if (imuOK) Serial.println(F("IMU reconnected: run CAL while stationary"));
    }
    return;
  }
  int16_t r[7];
  if (!imuRead14(r)) {
    imuOK = false; // No blind integration or laser after I2C failure
    ++imuReadFailures;
    Serial.println(F("WARN: IMU read failed; angle invalid until recovered/CAL"));
    return;
  }
  for (uint8_t a = 0; a < 3; a++) {
    accelG[a] = (float)r[a] / ACC_LSB_PER_G;
    gyroDps[a] = ((float)r[a+4] - biasRaw[a]) / GYRO_LSB_PER_DPS;
  }
  tempC = (float)r[3] / 340.0f + 36.53f;
  float rawRate = YAW_SIGN * gyroDps[YAW_GYRO_AXIS];
  filteredYawRate += GYRO_EMA_ALPHA * (rawRate - filteredYawRate);
  float dt = (unsigned long)(nowUs - lastImuUs) * 0.000001f;
  if (dt > 0.0f && dt < 0.10f) yawIntegrated += rawRate * dt;
  lastImuUs = nowUs;
#endif
}

void pollIR() {
#if USE_IR_SENSORS
  const unsigned long now = millis();
  uint16_t ticks[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i=0;i<3;i++) ticks[i]=isrIrFalling[i];
  }
  uint8_t d = PIND;
  uint8_t raw = 0;
  const uint8_t bitMasks[3] = {4,2,1};
  const uint8_t hwBits[3] = {_BV(PD2),_BV(PD3),_BV(PD4)};
  for (uint8_t i=0; i<3; ++i) {
    irEdgeDeltas[i] = (uint16_t)(ticks[i] - prevIrEdges[i]);
    prevIrEdges[i] = ticks[i];
    bool active = ((d & hwBits[i]) == 0);
    if (active) raw |= bitMasks[i];
    if (active || irEdgeDeltas[i] != 0) lastIRSeenMs[i] = now;
  }
  irRawMask = raw;
  irMask = 0;
  for (uint8_t i=0; i<3; ++i) {
    if (lastIRSeenMs[i] != 0 && (unsigned long)(now - lastIRSeenMs[i]) <= IR_ACTIVE_HOLD_MS)
      irMask |= bitMasks[i];
  }
  if (irMask != previousIrMask) {
    previousIrMask = irMask;
    irMaskSinceMs = now;
  }
  // Previous V8.x logs showed valid centre masks oscillating 010 <-> 111.
  // Stable qualification must not reset when the mask changes within that set.
  if (irMask == 0x02 || irMask == 0x07) {
    if (centreAcceptedSinceMs == 0) centreAcceptedSinceMs = now;
  } else centreAcceptedSinceMs = 0;
#else
  irRawMask=0; irMask=0;
#endif
}

void pollEncoder() {
#if USE_ENCODER
  const unsigned long now = millis();
  if (now - lastEncMs < 200) return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { encCount = isrEncCount; }
  unsigned long elapsed = now - lastEncMs;
  if (elapsed > 0) encRpm = ((encCount - lastEncCount)*60000.0f) /
                       ((float)elapsed * ENCODER_COUNTS_PER_REV_APPROX);
  lastEncCount = encCount;
  lastEncMs = now;
#endif
}

bool alignedIR() {
#if USE_IR_SENSORS
  // The team's observed valid masks. 111 is AMBIGUOUS, report separately.
  return (irMask == 0x02 || irMask == 0x07);
#else
  return false;
#endif
}

bool rateStable() {
#if USE_MPU6050
  return imuOK && absf(filteredYawRate) < RATE_STABLE_LIMIT_DPS &&
         rateStableSinceMs != 0 && millis() - rateStableSinceMs >= RATE_STABLE_MS;
#else
  return false;
#endif
}

bool centreStable() {
#if USE_IR_SENSORS
  return alignedIR() && centreAcceptedSinceMs != 0 &&
         millis() - centreAcceptedSinceMs >= IR_STABLE_MS;
#else
  return false;
#endif
}

bool qualified() { return centreStable() && rateStable(); }

void updateRateStability() {
#if USE_MPU6050
  if (!imuOK || absf(filteredYawRate) >= RATE_STABLE_LIMIT_DPS) rateStableSinceMs=0;
  else if (rateStableSinceMs==0) rateStableSinceMs=millis();
#endif
}

void laserOff() {
  digitalWrite(PIN_LASER, LOW);
  laserActive = false;
}

void laserOn() {
#if ENABLE_LASER
  digitalWrite(PIN_LASER, HIGH);
  laserActive = true;
#endif
}

void stopLaser(const __FlashStringHelper *reason) {
  if (laserActive) {
    Serial.print(F("LASER OFF: "));
    Serial.println(reason);
  }
  laserOff();
  laserArmed = false;
}

void setMode(uint8_t next) {
  if (next > 15) { Serial.println(F("ERR: use mode 0..15")); return; }
  stopLaser(F("mode changed"));
  testMode=next;
  testStartMs=millis();
  testStartHeading=heading();
  lastPrintMs=0;
  previousAlignment=0;
  alignmentFirstMs=0;
  completedDwell=0;
  rejectedDwell=0;
  for (uint8_t i=0;i<8;i++) maskHistogram[i]=0;
  lastHistMs=millis();
  lastMaskSampleMs=millis();
  Serial.print(F("TEST=")); Serial.println(testMode);
  if (next == 10 || next == 11) {
#if ENABLE_LASER
    Serial.println(F("LASER mode: starts DISARMED. ARM for one supervised shot."));
#else
    Serial.println(F("LASER COMPILE-TIME DISABLED (#define ENABLE_LASER 0)."));
#endif
  }
  if (next==3) Serial.println(F("DRIFT 60s: KEEP BODY STATIONARY; compare to start."));
  if (next==12) Serial.println(F("ms,yaw_deg,rate_dps,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,temp_C,IRraw,IRheld,qualified,enc_count"));
}

int landmarkIndex(const char *s) {
  if (s && s[0] && !s[1]) {
    if (s[0]=='A'||s[0]=='a'||s[0]=='1') return 0;
    if (s[0]=='B'||s[0]=='b'||s[0]=='2') return 1;
    if (s[0]=='C'||s[0]=='c'||s[0]=='3') return 2;
  }
  return -1;
}

void listLandmarks() {
  Serial.println(F("Stored BODY headings; RAM ONLY, 0 is arbitrary:"));
  for (uint8_t i=0;i<3;i++) {
    Serial.print((char)('A'+i)); Serial.print(F(": "));
    if (!landmarkValid[i]) Serial.println(F("not set"));
    else {
      Serial.print(landmarks[i],2);
      Serial.print(F(" deg; shortest return error "));
      Serial.print(wrap180(landmarks[i]-heading()),2);
      Serial.println(F(" deg"));
    }
  }
}

void printStatus() {
  Serial.print(F("STATUS test=")); Serial.print(testMode);
  Serial.print(F(" ms=")); Serial.print(millis());
  Serial.print(F(" MPU=")); Serial.print(imuOK ? F("OK") : F("OFF/FAULT"));
  Serial.print(F(" yaw=")); Serial.print(heading(),2);
  Serial.print(F(" rate=")); Serial.print(filteredYawRate,3);
  Serial.print(F(" rawIR=")); printMask(irRawMask);
  Serial.print(F(" heldIR=")); printMask(irMask);
  Serial.print(F(" qual=")); Serial.print(qualified() ? 1 : 0);
  Serial.print(F(" laser=")); Serial.print(laserActive ? F("ON") : F("OFF"));
  Serial.print(F(" enc=")); Serial.println(encCount);
}

void printHelp() {
  Serial.println(F("MQ SENTINEL SENSOR-ONLY TEST SUITE; motor ALWAYS OFF"));
  Serial.println(F("MODE <n> or just <n>; 115200 baud; select Newline in monitor"));
  Serial.println(F(" 0=status/manual menu       1=IMU raw accel/gyro/temp"));
  Serial.println(F(" 2=body yaw and yaw rate    3=stationary 60s drift"));
  Serial.println(F(" 4=manual turn/step error   5=IR raw+interrupt edges"));
  Serial.println(F(" 6=IR-mask histogram        7=IR+gyro sweep/centre events"));
  Serial.println(F(" 8=relative target memory   9=dry-run 2s hold (NO LASER)"));
  Serial.println(F("10=manual gyro-only laser  11=qualified IR+gyro laser"));
  Serial.println(F("12=CSV full sensor logger  13=passive wheel encoder"));
  Serial.println(F("14=pure maths simulator    15=landmark reacquire/error"));
  Serial.println(F("HELP STATUS CAL ZERO LIST MARK A/B/C ERROR A/B/C"));
  Serial.println(F("SYNC A/B/C (requires stable centre & gyro), CLEAR A/B/C"));
  Serial.println(F("RATE <milliseconds> (50..2000), SIM <now> <goal>"));
  Serial.println(F("ARM DISARM FIRE (laser requires ENABLE_LASER=1)"));
  Serial.println(F("IR order = Left Centre Right, 010/111 are team valid masks."));
  Serial.println(F("111 can be optical overlap, NOT independent proof of bullseye."));
}

// -------- COMMAND LINE: no String allocations, safe for Nano RAM ----------
void processLine(char *line) {
  char *cmd = strtok(line," \t,");
  if (!cmd) return;
  // A bare integer selects a runtime test.
  if (cmd[0]>='0' && cmd[0]<='9') {
    char *end; long n=strtol(cmd,&end,10);
    if (*end=='\0' && n>=0 && n<=15) { setMode((uint8_t)n); return; }
  }
  if (!strcasecmp(cmd,"MODE") || !strcasecmp(cmd,"TEST")) {
    char *a=strtok(NULL," \t,"); if (a) setMode((uint8_t)atoi(a));
    else Serial.println(F("Usage: MODE 0..15"));
  } else if (!strcasecmp(cmd,"HELP") || !strcasecmp(cmd,"?")) {
    printHelp();
  } else if (!strcasecmp(cmd,"STATUS")) {
    printStatus();
  } else if (!strcasecmp(cmd,"CAL")) {
    stopLaser(F("gyro calibration")); calibrateGyro();
  } else if (!strcasecmp(cmd,"ZERO")) {
    stopLaser(F("heading reset")); yawIntegrated=0; yawCorrection=0;
    for(uint8_t i=0;i<3;i++) landmarkValid[i]=false;
    testStartHeading=0;
    Serial.println(F("Yaw ZERO and all landmarks cleared (reference changed)."));
  } else if (!strcasecmp(cmd,"LIST")) {
    listLandmarks();
  } else if (!strcasecmp(cmd,"MARK") || !strcasecmp(cmd,"ERROR") ||
             !strcasecmp(cmd,"SYNC") || !strcasecmp(cmd,"CLEAR")) {
    // strtok invalidates earlier tokens only at delimiter, cmd itself remains.
    char *a=strtok(NULL," \t,");
    int i=landmarkIndex(a);
    if (i<0) { Serial.println(F("Use A, B or C (or 1,2,3)")); return; }
    if (!strcasecmp(cmd,"CLEAR")) {
      landmarkValid[i]=false; Serial.println(F("Landmark cleared"));
    } else if (!strcasecmp(cmd,"MARK")) {
      if (!imuOK) { Serial.println(F("ERR: gyro not online")); return; }
      landmarks[i]=heading(); landmarkValid[i]=true;
      Serial.print(F("Stored landmark ")); Serial.print((char)('A'+i));
      Serial.print(F(" at ")); Serial.println(landmarks[i],2);
    } else if (!landmarkValid[i]) {
      Serial.println(F("ERR: first MARK that landmark"));
    } else if (!strcasecmp(cmd,"ERROR")) {
      Serial.print(F("Shortest signed turn towards "));
      Serial.print((char)('A'+i)); Serial.print(F(": "));
      Serial.print(wrap180(landmarks[i]-heading()),2);
      Serial.println(F(" deg (manually turn; no motor commands)"));
    } else if (!strcasecmp(cmd,"SYNC")) {
      // A confirmed optical landmark can correct gyro bias/angle drift,
      // but only IF operator knows the same physical beacon is active.
      if (!imuOK || !qualified()) {
        Serial.println(F("SYNC REJECTED: need stable 010/111 and low yaw rate"));
        return;
      }
      float residual=wrap180(landmarks[i]-heading());
      yawCorrection+=residual;
      Serial.print(F("SYNC applied ")); Serial.print(residual,3);
      Serial.println(F(" deg. Verify SAME beacon as saved landmark!"));
    }
  } else if (!strcasecmp(cmd,"RATE")) {
    char *a=strtok(NULL," \t,");
    if (!a) { Serial.println(F("Usage: RATE 50..2000")); return; }
    long ms=atol(a); if(ms<50)ms=50; if(ms>2000)ms=2000;
    reportPeriodMs=(unsigned long)ms;
    Serial.print(F("Report period ms=")); Serial.println(reportPeriodMs);
  } else if (!strcasecmp(cmd,"SIM")) {
    char *a=strtok(NULL," \t,"); char *b=strtok(NULL," \t,");
    if(!a || !b) { Serial.println(F("Usage: SIM 350 10")); return; }
    float current=atof(a), goal=atof(b);
    Serial.print(F("SIM now=")); Serial.print(current,2);
    Serial.print(F(" goal=")); Serial.print(goal,2);
    Serial.print(F(" shortest_error=")); Serial.println(wrap180(goal-current),2);
  } else if (!strcasecmp(cmd,"DISARM")) {
    stopLaser(F("DISARM")); Serial.println(F("DISARMED"));
  } else if (!strcasecmp(cmd,"ARM")) {
#if ENABLE_LASER
    if ((testMode==10 || testMode==11) && imuOK) {
      laserArmed=true;
      Serial.println(F("ARMED for ONE 2-second shot only; never aim at eyes"));
    } else Serial.println(F("ARM only allowed in mode 10/11 with healthy gyro"));
#else
    Serial.println(F("LASER DISABLED AT COMPILE TIME; ENABLE_LASER=0"));
#endif
  } else if (!strcasecmp(cmd,"FIRE")) {
#if ENABLE_LASER
    if (testMode!=10 || !laserArmed || !rateStable() || laserActive) {
      Serial.println(F("FIRE REJECTED: require TEST 10, ARM, gyro still"));
    } else {
      laserStartedMs=millis(); laserOn();
      Serial.println(F("LASER ON: timed 2s manual gyro-only test"));
    }
#else
    Serial.println(F("LASER DISABLED AT COMPILE TIME"));
#endif
  } else {
    Serial.println(F("Unknown command; HELP for menu"));
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c=(char)Serial.read();
    if (c=='\n' || c=='\r') {
      if (lineLength) { inputLine[lineLength]='\0'; processLine(inputLine); lineLength=0; }
    } else if (c>=32 && c<=126) {
      if (lineLength < sizeof(inputLine)-1) inputLine[lineLength++]=c;
      else { lineLength=0; Serial.println(F("ERR: command too long")); }
    }
  }
}

// -------- PERIODIC DISPLAY / TEST LOGIC ----------
void handleLaserAndDryDwell() {
  const unsigned long now=millis();
  bool dwellMode=(testMode==9 || testMode==11);
  if (testMode==10 && laserActive) {
    if (!imuOK || !rateStable()) {
      ++rejectedDwell; stopLaser(F("gyro moving/fault during shot"));
    } else if (now-laserStartedMs>=LASER_DURATION_MS) {
      ++completedDwell; stopLaser(F("2s complete"));
    }
  }
  if (!dwellMode) return;

  if (laserActive) {
    // One invalid sample aborts; never keep firing if target is lost.
    if (!qualified()) {
      ++rejectedDwell; stopLaser(F("lost centre/rate; abort"));
      alignmentFirstMs=0;
    } else if (now-laserStartedMs>=LASER_DURATION_MS) {
      ++completedDwell; stopLaser(F("stable 2s complete"));
      alignmentFirstMs=0;
    }
    return;
  }

  if (!qualified()) {
    if (alignmentFirstMs!=0) {
      ++rejectedDwell;
      Serial.println(F("DRY HOLD RESET: centre/rate lost"));
    }
    alignmentFirstMs=0;
    return;
  }
  if (testMode==9) { // completely laser-free 2-second dry run
    if (!alignmentFirstMs) alignmentFirstMs=now;
    if (now-alignmentFirstMs>=LASER_DURATION_MS) {
      ++completedDwell;
      Serial.print(F("DRY SUCCESS #")); Serial.print(completedDwell);
      Serial.print(F(" heading=")); Serial.println(heading(),2);
      alignmentFirstMs=now; // may count continuous holds; stationary only
    }
  } else { // TEST 11: one-shot laser, only after explicit ARM
#if ENABLE_LASER
    if (laserArmed) {
      laserStartedMs=now;
      laserOn();
      Serial.print(F("LASER ON: qualified alignment mask=")); printMask(irMask);
      Serial.println(F("; 2s watchdog active"));
    }
#endif
  }
}

void runTest() {
  unsigned long now=millis();
  if (testMode==9 || testMode==10 || testMode==11) handleLaserAndDryDwell();
  if (testMode==0) return;  // no unsolicited status stream
  if (testMode==3 && now-testStartMs>=DRIFT_DURATION_MS) {
    Serial.print(F("DRIFT RESULT 60s: delta_deg="));
    Serial.print(heading()-testStartHeading,4);
    Serial.print(F(" mean_dps="));
    Serial.println((heading()-testStartHeading) / 60.0f,5);
    setMode(0); return;
  }
  if (testMode==6 && now-lastMaskSampleMs>=20UL) {
    ++maskHistogram[irMask & 7]; // ~50 samples per second, independently of print rate
    lastMaskSampleMs=now;
  }
  if (now-lastPrintMs<reportPeriodMs) return;
  lastPrintMs=now;

  switch (testMode) {
    case 1:
      Serial.print(F("IMU gx,gy,gz dps="));
      Serial.print(gyroDps[0],3); Serial.print(',');
      Serial.print(gyroDps[1],3); Serial.print(',');
      Serial.print(gyroDps[2],3);
      Serial.print(F(" accel_g="));
      Serial.print(accelG[0],3); Serial.print(',');
      Serial.print(accelG[1],3); Serial.print(',');
      Serial.print(accelG[2],3);
      Serial.print(F(" tempC=")); Serial.println(tempC,2);
      break;
    case 2: case 3: case 4:
      Serial.print(F("yaw_deg=")); Serial.print(heading(),3);
      Serial.print(F(" yaw_rate_dps=")); Serial.print(filteredYawRate,3);
      Serial.print(F(" delta_from_start="));
      Serial.print(heading()-testStartHeading,3);
      Serial.print(F(" still=")); Serial.println(rateStable()?1:0);
      break;
    case 5: {
      uint16_t count[3];
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i=0;i<3;i++) count[i]=isrIrFalling[i];
      }
      Serial.print(F("IR raw=")); printMask(irRawMask);
      Serial.print(F(" held=")); printMask(irMask);
      Serial.print(F(" ISR_falls_L,C,R_total="));
      Serial.print(count[0]); Serial.print(',');
      Serial.print(count[1]); Serial.print(',');
      Serial.println(count[2]);
      break;
    }
    case 6: {
      // Mask histogram is sampled at ~50 Hz, independent of serial baud.
      Serial.print(F("MASK=")); printMask(irMask);
      Serial.print(F(" 000/001/010/011/100/101/110/111 ="));
      for(uint8_t i=0;i<8;i++) {
        Serial.print(maskHistogram[i]); if(i<7)Serial.print('/');
      }
      Serial.println();
      break;
    }
    case 7:
      Serial.print(F("IR_YAW mask=")); printMask(irMask);
      Serial.print(F(" raw=")); printMask(irRawMask);
      Serial.print(F(" yaw=")); Serial.print(heading(),2);
      Serial.print(F(" rate=")); Serial.print(filteredYawRate,2);
      Serial.print(F(" centreStable=")); Serial.print(centreStable()?1:0);
      Serial.print(F(" qualified=")); Serial.println(qualified()?1:0);
      if (qualified() && !previousAlignment) {
        Serial.print(F(">>> CENTRE LOCK EVENT at yaw="));
        Serial.print(heading(),3);
        Serial.println(F(" (laser centre NOT independently verified)"));
      }
      previousAlignment=qualified()?1:0;
      break;
    case 8: case 15:
      Serial.print(F("current_yaw=")); Serial.print(heading(),3);
      Serial.print(F(" IR=")); printMask(irMask);
      Serial.print(F(" stable=")); Serial.print(qualified()?1:0);
      Serial.print(F(" return_errors A/B/C="));
      for(uint8_t i=0;i<3;i++) {
        if(landmarkValid[i]) Serial.print(wrap180(landmarks[i]-heading()),2);
        else Serial.print(F("NA"));
        if(i<2)Serial.print('/');
      }
      Serial.println();
      break;
    case 9:
      Serial.print(F("DRY_HOLD qualified=")); Serial.print(qualified()?1:0);
      Serial.print(F(" heldIR=")); printMask(irMask);
      Serial.print(F(" elapsed_ms="));
      Serial.print(alignmentFirstMs ? now-alignmentFirstMs : 0);
      Serial.print(F(" done=")); Serial.print(completedDwell);
      Serial.print(F(" reset=")); Serial.println(rejectedDwell);
      break;
    case 10: case 11:
      Serial.print(F("LASER_TEST armed=")); Serial.print(laserArmed?1:0);
      Serial.print(F(" on=")); Serial.print(laserActive?1:0);
      Serial.print(F(" IR=")); printMask(irMask);
      Serial.print(F(" yaw=")); Serial.print(heading(),2);
      Serial.print(F(" rate=")); Serial.print(filteredYawRate,2);
      Serial.print(F(" done/abort=")); Serial.print(completedDwell);
      Serial.print('/'); Serial.println(rejectedDwell);
      break;
    case 12:
      Serial.print(now); Serial.print(',');
      Serial.print(heading(),3); Serial.print(',');
      Serial.print(filteredYawRate,3); Serial.print(',');
      for(uint8_t i=0;i<3;i++){Serial.print(gyroDps[i],3);Serial.print(',');}
      for(uint8_t i=0;i<3;i++){Serial.print(accelG[i],3);Serial.print(',');}
      Serial.print(tempC,2); Serial.print(',');
      printMask(irRawMask); Serial.print(',');
      printMask(irMask); Serial.print(',');
      Serial.print(qualified()?1:0); Serial.print(',');
      Serial.println(encCount);
      break;
    case 13:
      Serial.print(F("ENC passive count=")); Serial.print(encCount);
      Serial.print(F(" wheel_rpm~=")); Serial.print(encRpm,2);
      Serial.print(F(" body_yaw=")); Serial.print(heading(),2);
      Serial.println(F(" (encoder = wheel motion, NOT body yaw)"));
      break;
    case 14:
      Serial.println(F("SIM: no sensors used. Type SIM 350 10, SIM 10 350, etc."));
      break;
  }
}

void setup() {
  // Prior firmware uses DRV8874 IN1=D6 and IN2=D7. Keep both LOW (coast).
  pinMode(PIN_MOTOR_IN1, OUTPUT); digitalWrite(PIN_MOTOR_IN1, LOW);
  pinMode(PIN_MOTOR_IN2, OUTPUT); digitalWrite(PIN_MOTOR_IN2, LOW);
  pinMode(PIN_LASER, OUTPUT); digitalWrite(PIN_LASER, LOW);
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  Serial.begin(115200);
  delay(400);
  Serial.println(F("MQ Sentinel diagnostic firmware. NO MOTOR ACTUATION."));
#if USE_IR_SENSORS
  pinMode(PIN_IR_L, INPUT_PULLUP);
  pinMode(PIN_IR_C, INPUT_PULLUP);
  pinMode(PIN_IR_R, INPUT_PULLUP);
  isrIrPrevious=PIND & 0x1C;
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT18)|_BV(PCINT19)|_BV(PCINT20);
#endif
#if USE_ENCODER
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  isrEncPrev=(PINB>>1)&3;
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT1)|_BV(PCINT2);
#endif
#if USE_MPU6050
  Wire.begin(); Wire.setClock(100000UL);
  imuOK=initImu();
  if (imuOK) calibrateGyro();
  else Serial.println(F("MPU NOT FOUND: check I2C level shifter, A4/A5, VCC, AD0"));
#else
  Serial.println(F("USE_MPU6050=0: pure IR/encoder/maths tests still available"));
#endif
  irMaskSinceMs=millis();
  lastEncMs=millis();
  lastIRSeenMs[0]=lastIRSeenMs[1]=lastIRSeenMs[2]=0;
  testStartMs=millis();
  printHelp();
}

void loop() {
  // Run sensor updates regardless of whether the serial monitor is attached.
  pollImu();
  pollIR();
  pollEncoder();
  updateRateStability();
  readSerialCommands();
  runTest();
  // Redundant safety output: when NOT in laser mode always force D8 LOW.
  if(testMode != 10 && testMode != 11) laserOff();
}
