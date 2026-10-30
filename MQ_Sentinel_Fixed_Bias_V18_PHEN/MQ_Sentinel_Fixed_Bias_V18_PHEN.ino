/*
  MQ SENTINEL V18 — reference X gyro bias, no moving-platform calibration, frequent micro pulses.
  Classic Nano ATmega328P; DRV8874 driver mode selected below, nSLEEP HIGH.
  D5 IN1, D6 IN2, D4 eye-safe driven laser, D3 encoder A, D2 encoder B,
  D11 active-LOW 38 kHz demodulator, A4/A5 MPU6050, D13 lock indicator.

  BOOT -> SELFTEST (laser self-test + fixed stored X gyro bias) -> CLEAR -> SEEK -> SLOW_SCAN (full IR entry/exit)
  -> ARC_BRAKE -> AIM -> FINE -> FIRE (up to 2000 ms while aligned)
  -> EXIT_ARC -> CLEAR -> SEEK; repeat. Legacy two-pass states retained
  as optional fallback if DIRECT_FULL_ARC=0.

  Important: reaction-wheel torque comes from acceleration, NOT steady RPM.
  BODY_SIGN and ENCODER_SIGN must be verified on the physical suspended rig.
  Soft and hard wheel RPM thresholds depend on correct encoder scaling.
  No stationary calibration after restart. Encoder diagnosis still prevents blind wheel overspeed; t=retry.
  DRV8874 PMODE configuration is compile-time selectable; do not guess.
  First test without laser connected and with mechanically contained flywheel.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// ------------------------- HARDWARE ---------------------------------------
// 0: DRV8874 PMODE HIGH, independent IN1/IN2 PWM, HIGH/HIGH = brake.
// 1: DRV8874 PMODE LOW, IN1=EN PWM and IN2=PH direction, EN LOW = brake.
// Select to match the ACTUAL PMODE level BEFORE nSLEEP goes HIGH.
#ifndef DRIVER_PHASE_ENABLE
#define DRIVER_PHASE_ENABLE 1
#endif
#define DIRECT_FULL_ARC 1
const uint8_t PIN_IN1=5, PIN_IN2=6, PIN_LASER=4;
const uint8_t PIN_ENC_A=3, PIN_ENC_B=2, PIN_IR=11, PIN_LED=13, MPU=0x68;
#define LASER_ENABLED 1         // D4 HIGH only if validated lock, eye-safe module
const int8_t BODY_SIGN=-1;     // +IN1 produces negative body angular accel: VERIFY
const int8_t ENCODER_SIGN=+1;  // +IN1 produces positive encoder counts: VERIFY
const uint8_t YAW_AXIS=0; // 0=X, 1=Y, 2=Z; sensor's X is chassis yaw
const float GYRO_X_BIAS=24.3267f; // user measured RAW gyro-X counts
const float PRIOR_X_BIAS_DPS=GYRO_X_BIAS/131.0f; // 0.18570 deg/s, fixed at EVERY boot
const float PRIOR_Y_BIAS_DPS=84.4403f/131.0f;
const float PRIOR_Z_BIAS_DPS=-0.12317f;
// Stationary reference from six measurements. Dynamic re-calibration is OFF.
// Using this fixed bias will cause some temperature-related yaw drift;
// never mistake genuine chassis movement for a new gyro offset.
#define USE_STORED_X_BIAS_NO_RECAL 1
const float X_BIAS_REFERENCE_TOL_DPS=0.30f;
const uint16_t GYRO_WARNING_INTERVAL_MS=1000, FAULT_RETRY_MS=2000;
const float MAX_INTEGRATED_GYRO_DPS=235.0f;
const uint16_t LASER_CAL_SELFTEST_MS=350, FIRE_DURATION_MS=2000;
const uint16_t EXIT_ARC_TIMEOUT_MS=8500, EXIT_CLEAR_MS=250;
const float EXIT_RATE_DPS=4.0f;
const uint16_t ENCODER_START_LIMIT_MS=1100;
const int16_t ENCODER_START_PWM=185;
const uint8_t ENCODER_REQUIRED_EDGES=3;
const uint16_t ENCODER_STALE_MS=2200;
const uint16_t FINE_BRAKE_START_MS=35, FINE_BRAKE_ON_MS=16;
const uint16_t BRAKE_PERIOD_MS=70, BRAKE_ON_MS=17, AIM_BRAKE_PERIOD_MS=75;
const uint16_t BRAKE_PREP_MS=5, BRAKE_RELEASE_MS=15;
const float BRAKE_MIN_WHEEL_RPM=12.0f, BRAKE_MIN_BODY_RATE=0.75f;
const uint16_t BRAKE_ENCODER_FRESH_MS=350;

// -------------------------- SENSORS ---------------------------------------
const uint16_t GYRO_INTERVAL_MS=5;
const float GYRO_COUNTS_PER_DPS=131.0f, ACC_COUNTS_PER_G=16384.0f;
const float RATE_LPF=0.16f;
const uint32_t BOOT_WAIT_MS=2000UL;
const uint32_t IR_MIN_PULSE_US=100UL, IR_MAX_PULSE_US=35000UL;
const uint8_t IR_PULSES_REQUIRED=2;
// Debounced IR arc and live lock have separate freshness windows.
// A demodulator may pulse LOW or remain LOW temporarily. An indefinitely
// LOW input is not proof of correct optical alignment. Allow up to 9s
// because genuine slow passes can stay LOW for several seconds.
const uint16_t IR_GROUP_MS=180, IR_LOSS_MS=210, IR_RECENT_MS=260;
const uint16_t IR_MAX_CONTINUOUS_LOW_MS=9000;
const uint16_t IR_LOCK_LOW_MIN_MS=7;
const uint16_t IR_LOW_STEADY_MS=9, IR_CLEAR_MS=220;
const uint16_t IR_POLL_MS=5, IR_PATH_MS=30;
const uint16_t ARC_MIN_MS=65, ARC_MAX_MS=8500;
const float ARC_MIN_DEG=0.5f, ARC_MAX_DEG=85.0f;
const float ARC_MIN_EFFICIENCY=0.48f, ARC_MAX_BACKTRACK_DEG=2.5f;
const float ARC_MAX_SWEEP_RATE=85.0f;

// ---------------------- MOTOR (7.5 V SUPPLY) -------------------------------
// PWM=255 means 100% of the existing 7.5 V motor supply (NOT 12 V).
// Absolute wheel RPM depends on load, motor, encoder scaling and supply sag.
// The 250 RPM cutoff is inherited from V8: VERIFY mechanical safety before use.
const int8_t SEEK_MOTOR_SIGN=-1;              // existing V8 IN2 direction
const float MOTOR_PWM_LIMIT=255.0f;           // maximum available 7.5 V duty
const float SEEK_WHEEL_RPM=225.0f;            // HIGH target; full PWM until taper
const float SEEK_PWM=255.0f;                  // 100% duty when below taper threshold
const float SEEK_TAPER_START_RPM=SEEK_WHEEL_RPM-10.0f;      // back off before overspeed limit
const float WHEEL_HARD_RPM=250.0f;            // keep V8 safety cutoff unchanged

// At first IR we stop the BODY using gyroscope feedback, rather than
// demanding that the flywheel immediately change from 225 to 70 RPM.
// Controlled second pass runs in the original seek direction.
const float FAST_BRAKE_MAX_PWM=175.0f;
const float FAST_BRAKE_KP=14.0f, FAST_BRAKE_MIN_PWM=30.0f;
const float FAST_BRAKE_RATE_OK_DPS=1.0f;
const uint16_t FAST_BRAKE_STILL_MS=200, FAST_BRAKE_TIMEOUT_MS=6500;
const float BACKOUT_DPS=3.0f, BACKOUT_PWM_MAX=125.0f;
const float BACKOUT_MARGIN_DEG=1.8f;
const uint16_t BACKOUT_TIMEOUT_MS=18000;
const float SCAN_BODY_DPS=2.8f, SCAN_PWM_MAX=115.0f;
const float RATE_KP=15.0f, RATE_FRICTION_PWM=30.0f;
const float SLOW_SCAN_MAX_RATE_DPS=9.0f;
const uint16_t PRE_SCAN_STILL_MS=220, PRE_SCAN_TIMEOUT_MS=5500;
const uint16_t SLOW_SCAN_TIMEOUT_MS=19000;
const float ARC_BRAKE_MAX_PWM=125.0f;
const uint16_t ARC_BRAKE_STILL_MS=220, ARC_BRAKE_TIMEOUT_MS=6000;
const float STOPPED_DPS=0.75f;
const uint8_t MAX_SLOW_SCAN_RETRIES=2;

const float MOTOR_SLEW_PER_SEC=750.0f;  // ~0.34 s to full seek PWM
const float BRAKE_SLEW_PER_SEC=900.0f;  // gyro-controlled braking / backout
const uint16_t MOTOR_PERIOD_MS=5, REVERSE_DEADTIME_MS=24;
const uint16_t RPM_SAMPLE_MS=50;
// Encoder has shown 0 RPM even under full PWM in the 29 Oct hanging log.
// Without edges, RPM limits cannot protect the flywheel. Stop rather than
// allowing indefinite blind full duty. Diagnose D3/D2/encoder power first.
const uint16_t ENCODER_MISSING_TIMEOUT_MS=2200;
const int16_t ENCODER_WATCH_PWM=120;
const float WHEEL_UNWIND_TRIGGER_RPM=150.0f;
const float UNWIND_PWM_LIMIT=160.0f, UNWIND_RPM_FINISH=18.0f;
const uint16_t NO_YAW_UNWIND_MS=1500;
const float NO_YAW_THRESHOLD_DPS=1.1f;
const float COUNTS_PER_REV=700.0f;

// ------------------------- AIMING / LOCK -----------------------------------
// Slower gyro-based AIM than SEEK. Fine and FIRE use only micro-pulses.
const float COARSE_RATE_LIMIT_DPS=6.0f;
const float COARSE_KP_POS=1.3f, COARSE_KP_RATE=6.0f;
const float COARSE_FRICTION_PWM=38.0f, COARSE_MAX_PWM=115.0f;
const float BRAKE_ACCEL_DPS2=12.0f;  // cautious provisional DECEL estimate; measure on rig
const float AIM_CURVE_WIDTH_DEG=10.0f;
const float FINE_ENTER_DEG=3.2f, FINE_REENTER_DEG=4.4f;
const float FINE_ENTER_DPS=8.0f;
// More frequent, smaller pulses than V12 (145/45ms, PWM max 74).
const uint16_t FINE_PERIOD_MS=85, FINE_PULSE_MS=22;
const float FINE_MIN_PWM=90.0f, FINE_MAX_PWM=135.0f;
const float FINE_KP_POS=29.0f, FINE_KD_RATE=10.0f;
const float FINE_MIN_ERROR=0.22f;
// Positive lock needs BOTH gyro-centred and recently genuine active-LOW IR.
// Entry tolerances are stricter than the FIRE hold tolerances (hysteresis).
const float FIRE_ANGLE_TOL_DEG=2.20f, FIRE_RATE_TOL_DPS=3.20f;
const float FIRE_ANGLE_MIN_TOL_DEG=0.85f;
const float FIRE_HOLD_EXTRA_DEG=0.55f, FIRE_HOLD_RATE_DPS=4.3f;
const uint16_t FIRE_CONFIRM_MS=85, FIRE_IR_HOLD_MS=90;
const uint16_t NO_IR_AT_CENTRE_RESCAN_MS=3500;
const float MAX_FIRE_WHEEL_RPM=185.0f;
const uint16_t PRINT_INTERVAL_MS=250;

// -------------------------- STATE ------------------------------------------
enum State:uint8_t {
  BOOT, SELFTEST, CLEAR, SEEK, FAST_BRAKE, BACKOUT, SETTLE,
  SLOW_SCAN, ARC_BRAKE, AIM, FINE, FIRE, EXIT_ARC, UNWIND, ENC_DIAG, HOLD
};
State state=BOOT;
uint32_t enteredMs=0, clearSince=0, lastPrintMs=0, lastGyroMs=0, lastGyroUs=0;
uint32_t lastMotorMs=0, reverseUntil=0, slowYawSince=0, unwindSince=0;
uint32_t fineStartedMs=0, stableSince=0, noIrAt=0, lastLockIrAt=0;
uint32_t laserCalStartedMs=0, exitClearSince=0, motorBrakeReadyAt=0;
uint32_t gyroWarnAt=0, faultSince=0;
uint32_t brakeReleaseUntil=0, seekEnteredEdges=0, diagEnteredEdges=0;
uint16_t brakeActivations=0, printedBrakeActivations=0;
bool laserCalActive=false, encoderProven=false;
uint32_t phaseStillAt=0, backoutClearAt=0;
float firstIrYaw=0, firstSweepSign=1.0f, backoutTargetYaw=0;
uint8_t slowScanRetries=0;
bool slowArcStarted=false;
uint8_t stopCode=0, failedI2C=0;
uint16_t irOnCount=0, irOffCount=0, validArcs=0, rejectedArcs=0, fires=0;
float gyroBias=PRIOR_X_BIAS_DPS, yaw=0, yawRate=0, yawRateFiltered=0, rawRate=0, accelG=1, tempC=0;
bool calibrated=false, fallbackBias=true, laserHigh=false, irStuckReported=false;
uint16_t gyroSkippedSamples=0;
float wheelRPM=0, rpmInstant=0;
int32_t prevTicks=0;
uint32_t lastRPMMs=0;
volatile int32_t encoderTicks=0;
volatile uint32_t encoderRisingEdges=0;
uint32_t previousEncoderEdges=0, lastEncoderEdgeAt=0, lastPrintedEncoderEdges=0;
enum MotorMode:uint8_t { MODE_COAST, MODE_DRIVE, MODE_BRAKE };
MotorMode motorMode=MODE_COAST;
int16_t pwmApplied=0;
float pwmGoal=0, pwmActual=0;
float arcEntry=0, arcExit=0, arcCentre=0, arcWidth=0;
float arcMin=0, arcMax=0, arcPath=0, lastArcPathYaw=0, arcMaxRate=0;
uint32_t arcStarted=0, lastArcPathMs=0;
bool arcValid=false;

// D11 = PB3 / PCINT3, tracks short LOW pulses without 2ms polling losses.
volatile bool isLow=false;
volatile uint32_t lowStartUs=0, lastLowSeenUs=0;
volatile uint16_t validPulseTotal=0;
ISR(PCINT0_vect) {
  const bool low=(PINB & _BV(PB3))==0;
  if (low==isLow) return;
  const uint32_t t=micros();
  isLow=low;
  if (low) { lowStartUs=t; lastLowSeenUs=t; }
  else {
    const uint32_t width=t-lowStartUs;
    if(width>=IR_MIN_PULSE_US && width<=IR_MAX_PULSE_US) ++validPulseTotal;
  }
}

struct IrState {
  bool low=false, active=false;
  uint16_t previousPulses=0, groupPulses=0, pulsesDisplayed=0;
  uint32_t groupStart=0, lastSignal=0, lastSample=0;
  float firstYaw=0, lastYaw=0;
} ir;

float absf(float x){return x<0?-x:x;}
float clampf(float x,float low,float high){return x<low?low:(x>high?high:x);}
float signf(float x){return x>0?1.0f:(x<0?-1.0f:0.0f);}
float smoothstep(float x){x=clampf(x,0,1);return x*x*(3.0f-2.0f*x);}
// A new state may be entered *after* loop() snapshots `now`. Unsigned
// subtraction of that older timestamp used to cause an instant timeout.
// Signed half-range comparison is safe for these short (< 30s) intervals
// including across millis() wraparound.
bool timeElapsed(uint32_t now,uint32_t since,uint32_t waitMs){
  return (int32_t)(now-since)>=0 && (uint32_t)(now-since)>=waitMs;
}
const char* stateName(){
  switch(state){
    case BOOT:return "BOOT";case SELFTEST:return "TEST";case CLEAR:return "CLEAR";
    case SEEK:return "SEEK";case FAST_BRAKE:return "BRAKE";
    case BACKOUT:return "BACKOUT";case SETTLE:return "SETTLE";
    case SLOW_SCAN:return "SCAN";case ARC_BRAKE:return "ARC_BRK";
    case AIM:return "AIM";
    case FINE:return "FINE";case FIRE:return "FIRE";
    case EXIT_ARC:return "EXIT";
    case UNWIND:return "UNWIND";case ENC_DIAG:return "ENC_DIAG";
    default:return "HOLD";
  }
}
void laserOff(){digitalWrite(PIN_LASER,LOW);laserHigh=false;}
void laserOn(){
#if LASER_ENABLED
  // Print BEFORE changing the pin. A reset after LASER_ARM but before the
  // second message strongly suggests power/wiring trouble at D4 activation.
  Serial.print(F("LASER_ARM,angle_error="));Serial.print(arcCentre-yaw,2);
  Serial.print(F(",rate="));Serial.println(yawRateFiltered,2);
  Serial.flush();
  digitalWrite(PIN_LASER,HIGH);
  laserHigh=true;
  Serial.println(F("LASER_D4_HIGH,OUTPUT_COMMAND_CONFIRMED"));
#else
  Serial.println(F("LASER_DISABLED_IN_BUILD"));
#endif
}
// Correct motor bridge output for the selected physical DRV8874 PMODE.
void motorHardware(int16_t p){
#if DRIVER_PHASE_ENABLE
  // PH/EN: IN1 is enable PWM; IN2 is phase. With EN=0 both outputs brake.
  if(p==0){digitalWrite(PIN_IN1,LOW);digitalWrite(PIN_IN2,LOW);motorMode=MODE_COAST;}
  else{digitalWrite(PIN_IN2,p>0?HIGH:LOW);analogWrite(PIN_IN1,abs(p));motorMode=MODE_DRIVE;}
#else
  if(p>0){digitalWrite(PIN_IN2,LOW);analogWrite(PIN_IN1,p);motorMode=MODE_DRIVE;}
  else if(p<0){digitalWrite(PIN_IN1,LOW);analogWrite(PIN_IN2,-p);motorMode=MODE_DRIVE;}
  else{digitalWrite(PIN_IN1,LOW);digitalWrite(PIN_IN2,LOW);motorMode=MODE_COAST;}
#endif
}
void motorElectricalBrake(){
  if(motorMode!=MODE_BRAKE)++brakeActivations;
#if DRIVER_PHASE_ENABLE
  // IMPORTANT: HIGH/HIGH is DRIVE in PH/EN, not brake.
  digitalWrite(PIN_IN1,LOW);digitalWrite(PIN_IN2,LOW);
#else
  digitalWrite(PIN_IN1,HIGH);digitalWrite(PIN_IN2,HIGH);
#endif
  motorMode=MODE_BRAKE;pwmGoal=pwmActual=0;pwmApplied=0;
}
void motorOff(){pwmGoal=pwmActual=0;pwmApplied=0;motorHardware(0);}
void serviceStartupLaser(uint32_t now){
  if(laserCalActive && (state!=SELFTEST || timeElapsed(now,laserCalStartedMs,LASER_CAL_SELFTEST_MS))){
    laserCalActive=false;laserOff();Serial.println(F("SELFTEST_LASER_OFF"));
  }
}
uint32_t edgeCountNow(); // predeclare encoder atomic helper
void enter(State s){
  state=s;enteredMs=millis();
  // Do not unnecessarily erase confirmation when AIM -> FINE while already
  // centred; both states are allowed to complete laser confirmation.
  if(s!=AIM && s!=FINE)stableSince=0;
  if(s==SEEK){lastEncoderEdgeAt=enteredMs;seekEnteredEdges=edgeCountNow();}
  if(s==ENC_DIAG)diagEnteredEdges=edgeCountNow();
  Serial.print(F("STATE,"));Serial.print(stateName());
  Serial.print(F(",yaw="));Serial.print(yaw,2);
  Serial.print(F(",ir="));Serial.println(ir.active?1:0);
}
// Serious hardware faults still remove power. Unlike V12, they are not
// permanent: a valid sensor stream/acceptable wheel speed allows restart.
// Normal motion, shaking and gyro rates above 90 dps are NOT faults.
void safeStop(uint8_t c){
  if(state==HOLD)return;
  stopCode=c;laserOff();motorOff();arcValid=false;slowArcStarted=false;
  faultSince=millis();enter(HOLD);
  Serial.print(F("HARDWARE_HOLD,code="));Serial.println(c);
  Serial.println(F("MOTOR_LASER_OFF; retry when gyro/I2C valid and wheel within limit"));
}
void beginClear(){
  laserOff();arcValid=false;slowArcStarted=false;
  clearSince=0;slowYawSince=0;phaseStillAt=0;
  enter(CLEAR);
}
void encoderRise(){encoderTicks+=digitalRead(PIN_ENC_B)?-1:+1; ++encoderRisingEdges;}
int32_t countNow(){int32_t c;ATOMIC_BLOCK(ATOMIC_RESTORESTATE){c=encoderTicks;}return c;}
uint32_t edgeCountNow(){uint32_t c;ATOMIC_BLOCK(ATOMIC_RESTORESTATE){c=encoderRisingEdges;}return c;}

// ----------------------------- MPU6050 -------------------------------------
bool mpuWrite(uint8_t r,uint8_t v){
  Wire.beginTransmission(MPU);Wire.write(r);Wire.write(v);
  return Wire.endTransmission()==0;
}
bool mpuRead(float &dps,float &g){
  Wire.beginTransmission(MPU);Wire.write((uint8_t)0x3B);
  if(Wire.endTransmission(false)!=0)return false;
  if(Wire.requestFrom((uint8_t)MPU,(uint8_t)14,(uint8_t)true)!=14)return false;
  int16_t v[7];
  for(uint8_t i=0;i<7;i++){uint16_t h=Wire.read(),l=Wire.read();v[i]=(int16_t)((h<<8)|l);}
  const float x=v[0]/ACC_COUNTS_PER_G,y=v[1]/ACC_COUNTS_PER_G,z=v[2]/ACC_COUNTS_PER_G;
  g=sqrtf(x*x+y*y+z*z);
  tempC=v[3]/340.0f+36.53f;
  // v[4]=gyro X (registers 0x43/0x44); v[5]=Y; v[6]=Z.
  dps=v[4+YAW_AXIS]/GYRO_COUNTS_PER_DPS;
  return true;
}
void startStartupSelfTest(){
  // No moving-platform calibration: the six-test X reference is usable immediately.
  // SELFTEST is retained solely for the 350 ms visible laser self-test.
  motorOff();laserOff();
  gyroBias=PRIOR_X_BIAS_DPS;calibrated=true;fallbackBias=true;
  yaw=0;yawRate=0;yawRateFiltered=0;lastGyroUs=micros();
  enter(SELFTEST);
  Serial.print(F("GYRO_X_FIXED_BIAS,raw="));Serial.print(GYRO_X_BIAS,4);
  Serial.print(F(",dps="));Serial.println(gyroBias,5);
  Serial.println(F("NO_STATIONARY_CAL_REQUIRED; moving_chassis_will_not_fail_CAL"));
  laserCalStartedMs=millis();laserCalActive=true;
  digitalWrite(PIN_LASER,HIGH);laserHigh=true;
  Serial.println(F("SELFTEST_LASER_ON,350ms"));
}
void gyroUpdate(uint32_t now){
  if(now-lastGyroMs<GYRO_INTERVAL_MS)return;
  lastGyroMs=now;
  const uint32_t us=micros();
  float r,g;
  if(!mpuRead(r,g)){
    // Never continue an active laser with missing gyro data.
    if(state==FIRE){laserOff();beginClear();}
    if(++failedI2C>=4 && state!=HOLD)safeStop(1);
    lastGyroUs=us;
    return;
  }
  failedI2C=0;rawRate=r;accelG=g;
  // X-axis bias is always the stored six-test reference. We never attempt to
  // interpret motion while hanging as a new zero-rate calibration.
  if(!calibrated){lastGyroUs=us;return;}
  if(state==SELFTEST){lastGyroUs=us;return;}
  const float dt=(uint32_t)(us-lastGyroUs)*0.000001f;
  lastGyroUs=us;
  // A long loop/serial stall is not a broken gyro; discard the bad integration
  // interval, retain the valid reference, and invalidate a possible laser lock.
  if(dt>0.12f || dt<=0.0f){
    ++gyroSkippedSamples;
    if(state==FIRE || state==AIM || state==FINE || state==ARC_BRAKE || state==SLOW_SCAN){
      laserOff();beginClear(); // can't aim from an invalid integrated angle
    }
    if(timeElapsed(now,gyroWarnAt,GYRO_WARNING_INTERVAL_MS)){
      gyroWarnAt=now;
      Serial.print(F("GYRO_SAMPLE_GAP_MS,"));Serial.println(dt*1000.0f,1);
    }
    return;
  }
  // Body motion and varying accelerometer magnitude are expected after the
  // Sentinel is hung. Do NOT treat them as an MCU fault or re-enter SELFTEST.
  // Gyro saturation invalidates precision targeting but does not latch HOLD.
  if(absf(r)>=MAX_INTEGRATED_GYRO_DPS){
    ++gyroSkippedSamples;
    if(state==FIRE || state==AIM || state==FINE || state==ARC_BRAKE || state==SLOW_SCAN){
      laserOff();beginClear(); // can't aim from an invalid integrated angle
    }
    if(timeElapsed(now,gyroWarnAt,GYRO_WARNING_INTERVAL_MS)){
      gyroWarnAt=now;Serial.println(F("GYRO_X_SATURATED: tracking unreliable; no new calibration"));
    }
    return;
  }
  yawRate=r-gyroBias;
  if(absf(yawRate)<0.07f && absf(wheelRPM)<1.0f && pwmApplied==0)yawRate=0;
  yaw+=yawRate*dt;
  yawRateFiltered+=RATE_LPF*(yawRate-yawRateFiltered);
  if(state==SLOW_SCAN && slowArcStarted){
    if(yaw<arcMin)arcMin=yaw;
    if(yaw>arcMax)arcMax=yaw;
    if(absf(yawRate)>arcMaxRate)arcMaxRate=absf(yawRate);
    if(now-lastArcPathMs>=IR_PATH_MS){
      arcPath+=absf(yaw-lastArcPathYaw);
      lastArcPathYaw=yaw;lastArcPathMs=now;
    }
  }
}

// ------------------------ IR EDGE TRACKER ---------------------------------
// Actual live IR: either recently completed valid bursts or a measured,
// presently LOW input (with a maximum continuous-LOW sanity duration).
// Using only lastLowSeenUs rejects a genuine sustained LOW input because the
// timestamp updates on falling edges, not while the pin remains LOW.
bool irRecent(uint32_t now){
  if(!ir.active)return false;
  uint32_t started, lastEdge;
  bool low;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    started=lowStartUs;lastEdge=lastLowSeenUs;low=isLow;
  }
  const uint32_t us=micros();
  const uint32_t lowMs=(uint32_t)(us-started)/1000UL;
  // After accepting a full ON/OFF arc, sustained LOW is valid while angle
  // remains centred; a timer must not force a legitimate locked laser OFF.
  const bool validatedTarget=arcValid &&
      (state==AIM||state==FINE||state==FIRE);
  const bool validLevel=low && lowMs>=IR_LOCK_LOW_MIN_MS &&
      (lowMs<=IR_MAX_CONTINUOUS_LOW_MS || validatedTarget);
  // ir.lastSignal only tracks actual pulse completions or a bounded LOW.
  const bool pulseFresh=!low &&
      (uint32_t)(us-lastEdge)<=IR_RECENT_MS*1000UL &&
      (uint32_t)(now-ir.lastSignal)<=IR_RECENT_MS;
  return validLevel || pulseFresh;
}
void initialiseArc(float entry,uint32_t ms);
void firstIrEncounter(float entry){
  // A target event during the bounded encoder-start test is not sufficient
  // permission to enter sustained scanning without wheel-speed feedback.
  if(!encoderProven){
    Serial.println(F("IR_SEEN_DURING_ENCODER_START,waiting_for_D3_edges"));
    return;
  }
  firstIrYaw=entry;
  // Choose the sign of real observed body rotation, not the flywheel sign.
  firstSweepSign=absf(yawRateFiltered)>0.6f ? signf(yawRateFiltered)
                 : (float)(SEEK_MOTOR_SIGN*BODY_SIGN);
  slowScanRetries=0;phaseStillAt=0;backoutClearAt=0;
  // Do not suddenly cut PWM: motorUpdate slews through zero with deadtime.
  // This is controlled active braking, not an abrupt coast/reversal.
  laserOff();
#if DIRECT_FULL_ARC
  // First fresh IR entry = first edge; follow through until confirmed IR exit.
  initialiseArc(entry,millis());
  enter(SLOW_SCAN);
  Serial.println(F("DIRECT_FULL_ARC,first_IR_entry=arc_entry"));
#else
  enter(FAST_BRAKE);
#endif
  Serial.print(F("FAST_IR,yaw="));Serial.print(firstIrYaw,2);
  Serial.print(F(",body_dps="));Serial.print(yawRateFiltered,2);
  Serial.print(F(",direction="));Serial.println(firstSweepSign,0);
}
void startBackout(){
  laserOff();arcValid=false;slowArcStarted=false;
  phaseStillAt=0;backoutClearAt=0;
  backoutTargetYaw=firstIrYaw-firstSweepSign*BACKOUT_MARGIN_DEG;
  enter(BACKOUT);
  Serial.print(F("BACKOUT_TARGET,yaw="));Serial.println(backoutTargetYaw,2);
}
void initialiseArc(float entry,uint32_t ms){
  arcEntry=entry;arcStarted=ms;
  arcExit=entry;arcMin=arcMax=entry;arcPath=arcMaxRate=0;
  lastArcPathYaw=entry;lastArcPathMs=millis();
  slowArcStarted=true;
  Serial.print(F("SLOW_ARC_ENTRY,deg="));Serial.println(entry,2);
}
void rescanOrSeek(){
  laserOff();arcValid=false;slowArcStarted=false;
  if(++slowScanRetries<=MAX_SLOW_SCAN_RETRIES){
    Serial.print(F("SLOW_PASS_RETRY,"));Serial.println(slowScanRetries);
    startBackout();
  }else{
    Serial.println(F("SLOW_PASS_ABORT,full_seek_again"));
    slowScanRetries=0;beginClear();
  }
}
void finishArc(float exit,uint32_t ms){
  slowArcStarted=false;
  arcExit=exit;arcWidth=absf(arcExit-arcEntry);
  arcPath+=absf(arcExit-lastArcPathYaw);
  const float efficiency=arcWidth/fmaxf(arcWidth,arcPath);
  const float lo=fminf(arcEntry,arcExit),hi=fmaxf(arcEntry,arcExit);
  const uint32_t duration=ms-arcStarted;
  uint8_t bad=0;
  if(duration<ARC_MIN_MS)bad=1;
  else if(arcWidth<ARC_MIN_DEG)bad=2;
  else if(arcWidth>ARC_MAX_DEG)bad=3;
  else if(arcMin<lo-ARC_MAX_BACKTRACK_DEG||arcMax>hi+ARC_MAX_BACKTRACK_DEG)bad=4;
  else if(efficiency<ARC_MIN_EFFICIENCY)bad=5;
  else if(arcMaxRate>ARC_MAX_SWEEP_RATE)bad=6;
  else if(duration>ARC_MAX_MS)bad=7;
  if(bad){
    ++rejectedArcs;
    Serial.print(F("ARC_BAD,reason="));Serial.print(bad);
    Serial.print(F(",span="));Serial.print(arcWidth,2);
    Serial.print(F(",eff="));Serial.println(efficiency,2);
    rescanOrSeek();
  }else{
    ++validArcs;arcCentre=(arcEntry+arcExit)*0.5f;arcValid=true;
    phaseStillAt=0;
    Serial.print(F("ARC_OK,entry="));Serial.print(arcEntry,2);
    Serial.print(F(",exit="));Serial.print(arcExit,2);
    Serial.print(F(",centre="));Serial.print(arcCentre,2);
    Serial.print(F(",width="));Serial.println(arcWidth,2);
    enter(ARC_BRAKE);
  }
}

void irUpdate(uint32_t now){
  if(now-ir.lastSample<IR_POLL_MS)return;
  ir.lastSample=now;
  uint16_t total;
  uint32_t lowAt;
  bool low;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    total=validPulseTotal;lowAt=lowStartUs;low=isLow;
  }
  ir.low=low;
  const uint16_t newPulses=(uint16_t)(total-ir.previousPulses);
  ir.previousPulses=total;
  if(newPulses){
    ir.pulsesDisplayed+=newPulses;
    if(!ir.groupStart||now-ir.groupStart>IR_GROUP_MS){
      ir.groupStart=now;ir.groupPulses=0;ir.firstYaw=yaw;
    }
    const uint32_t sum=(uint32_t)ir.groupPulses+newPulses;
    ir.groupPulses=(sum>65535?65535:(uint16_t)sum);
    ir.lastSignal=now;ir.lastYaw=yaw;
  }
  const uint32_t lowDurationMs=(uint32_t)(micros()-lowAt)/1000UL;
  const bool validatedTarget=arcValid &&
      (state==AIM||state==FINE||state==FIRE);
  const bool stuckLow=low && lowDurationMs>IR_MAX_CONTINUOUS_LOW_MS &&
                      !validatedTarget;
  if(stuckLow && !irStuckReported){
    irStuckReported=true;
    Serial.print(F("IR_STUCK_LOW,ms="));Serial.print(lowDurationMs);
    Serial.println(F(",check_receiver_supply_or_carrier_saturation"));
  }
  if(!low)irStuckReported=false;
  const bool steadyLow=low && lowDurationMs>=IR_LOW_STEADY_MS &&
                       !stuckLow;
  if(steadyLow){
    if(!ir.groupStart||now-ir.groupStart>IR_GROUP_MS){
      ir.groupStart=now;ir.groupPulses=0;ir.firstYaw=yaw;
    }
    ir.lastSignal=now;ir.lastYaw=yaw;
  }
  // A physically stuck-LOW wire/receiver cannot keep the IR gate HIGH forever.
  // Invalidate its arc and require it to return HIGH before re-arming.
  if(stuckLow && ir.active){
    ir.active=false;ir.groupPulses=0;ir.groupStart=0;
    Serial.println(F("IR_INVALID,continuous_LOW_timeout"));
    if(state==SLOW_SCAN && slowArcStarted)rescanOrSeek();
  }
  if(!ir.active){
    if(!stuckLow && ((ir.groupPulses>=IR_PULSES_REQUIRED &&
         now-ir.groupStart<=IR_GROUP_MS && ir.groupStart!=0)||steadyLow)){
      ir.active=true;++irOnCount;
      Serial.print(F("IR_ON,edge="));Serial.print(ir.firstYaw,2);
      Serial.print(F(",pulses="));Serial.println(ir.groupPulses);
      if(state==SEEK)firstIrEncounter(ir.firstYaw);
      else if(state==SLOW_SCAN && !slowArcStarted)
        initialiseArc(ir.firstYaw,ir.groupStart);
    }
  }else if(!low && timeElapsed(now,ir.lastSignal,IR_LOSS_MS)){
    ir.active=false;++irOffCount;
    Serial.print(F("IR_OFF,edge="));Serial.println(ir.lastYaw,2);
    ir.groupPulses=0;ir.groupStart=0;
    if(state==SLOW_SCAN && slowArcStarted)finishArc(ir.lastYaw,ir.lastSignal);
  }
  if(!ir.active&&ir.groupStart&&now-ir.groupStart>IR_GROUP_MS){
    ir.groupStart=0;ir.groupPulses=0;
  }
}

// --------------------------- MOTOR CONTROL --------------------------------
void rpmUpdate(uint32_t now){
  if(now-lastRPMMs<RPM_SAMPLE_MS)return;
  int32_t ticks;uint32_t edges;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ticks=encoderTicks;edges=encoderRisingEdges;}
  if(edges!=previousEncoderEdges){
    lastEncoderEdgeAt=now;
    previousEncoderEdges=edges;
  }
  if(state==SEEK && !encoderProven &&
     (uint32_t)(edges-seekEnteredEdges)>=ENCODER_REQUIRED_EDGES){
    encoderProven=true;
    Serial.println(F("ENCODER_VERIFIED,3_edges"));
  }
  if(state==SEEK && !encoderProven &&
     timeElapsed(now,enteredMs,ENCODER_START_LIMIT_MS)){
    Serial.print(F("ENCODER_NO_PULSES,edges="));Serial.print(edges);
    Serial.print(F(",pwm="));Serial.print(pwmApplied);
    Serial.println(F(",inspect_D3_A_D2_B_and_encoder_power"));
    // Not a permanent fault: allow only manual bounded retry.
    motorOff();laserOff();enter(ENC_DIAG);
    Serial.println(F("ENC_DIAG,NO_EDGES,command_t_to_retry; check_PMODE_SLP_VM_D5_D6"));
  }
  if(encoderProven && state==SEEK && abs(pwmApplied)>=ENCODER_WATCH_PWM &&
     timeElapsed(now,lastEncoderEdgeAt,ENCODER_STALE_MS)){
    motorOff();laserOff();enter(ENC_DIAG);
    Serial.println(F("ENCODER_LOST,command_t_to_retry"));
  }
  if(lastRPMMs){
    rpmInstant=(ticks-prevTicks)*60000.0f/(COUNTS_PER_REV*(now-lastRPMMs));
    wheelRPM+=0.35f*(rpmInstant-wheelRPM);
    if(absf(wheelRPM)>WHEEL_HARD_RPM||absf(rpmInstant)>WHEEL_HARD_RPM+8.0f)safeStop(2);
  }
  lastRPMMs=now;prevTicks=ticks;
}
float cruisePWM(){
  if(!encoderProven)return SEEK_MOTOR_SIGN*ENCODER_START_PWM;
  // SEEK/CLEAR: request the FULL available 7.5 V drive until near the
  // inherited hard-RPM cutoff. Never infer a 12 V motor supply from PWM.
  const float rpm=wheelRPM*SEEK_MOTOR_SIGN*ENCODER_SIGN;
  if(rpm>=WHEEL_HARD_RPM)return 0;
  float out=SEEK_PWM;
  if(rpm>SEEK_TAPER_START_RPM){
    const float k=clampf((WHEEL_HARD_RPM-rpm)/
        (WHEEL_HARD_RPM-SEEK_TAPER_START_RPM),0,1);
    out*=k;
  }
  return SEEK_MOTOR_SIGN*clampf(out,0,MOTOR_PWM_LIMIT);
}

// Body-yaw rate regulator for the slow second pass and reverse backout.
// The PWM direction makes wheel torque oppose/sign-match BODY_SIGN as needed.
float ratePWM(float targetDps,float maxPwm){
  const float error=targetDps-yawRateFiltered;
  const float ff=RATE_FRICTION_PWM*smoothstep(absf(error)/2.0f);
  return BODY_SIGN*clampf(RATE_KP*error+signf(error)*ff,-maxPwm,maxPwm);
}
// Brake measured chassis motion (not wheel RPM). Wheel may continue spinning.
float bodyBrakePWM(float maxPwm){
  const float absRate=absf(yawRateFiltered);
  if(absRate<0.30f)return 0;
  const float level=clampf(FAST_BRAKE_KP*absRate+FAST_BRAKE_MIN_PWM,
                             0,maxPwm);
  return BODY_SIGN*(-signf(yawRateFiltered))*level;
}

float coarsePWM(){
  const float err=arcCentre-yaw;
  const float curve=COARSE_RATE_LIMIT_DPS*smoothstep(absf(err)/AIM_CURVE_WIDTH_DEG);
  const float braking=0.65f*sqrtf(2.0f*BRAKE_ACCEL_DPS2*absf(err));
  const float requested=signf(err)*fminf(curve,braking);
  const float rateError=requested-yawRateFiltered;
  const float friction=COARSE_FRICTION_PWM*smoothstep(absf(rateError)/4.0f);
  const float cmd=COARSE_KP_RATE*rateError+signf(rateError)*friction;
  return clampf(cmd*BODY_SIGN,-COARSE_MAX_PWM,COARSE_MAX_PWM);
}
float finePWM(uint32_t now){
  const float err=arcCentre-yaw;
  const float control=FINE_KP_POS*err-FINE_KD_RATE*yawRateFiltered;
  if(absf(err)<FINE_MIN_ERROR && absf(yawRateFiltered)<0.50f)return 0;
  const uint16_t phase=(uint16_t)((now-fineStartedMs)%FINE_PERIOD_MS);
  if(phase>=FINE_PULSE_MS)return 0;
  // Short rectangular pulse; PWM rises promptly, unlike V12's low-power cosine.
  return BODY_SIGN*signf(control)*clampf(absf(control),FINE_MIN_PWM,FINE_MAX_PWM);
}
float electricalBrakeBodySign(){
  // Slowing a spinning wheel produces body torque opposite its wheel-accel torque.
  return -BODY_SIGN*ENCODER_SIGN*signf(wheelRPM);
}
bool brakeOpposesYaw(uint32_t now){
  return encoderProven && lastEncoderEdgeAt &&
    timeElapsed(now,lastEncoderEdgeAt,0) &&
    (uint32_t)(now-lastEncoderEdgeAt)<=BRAKE_ENCODER_FRESH_MS &&
    absf(wheelRPM)>=BRAKE_MIN_WHEEL_RPM &&
    absf(yawRateFiltered)>=BRAKE_MIN_BODY_RATE &&
    electricalBrakeBodySign()*yawRateFiltered<0;
}
bool brakeHelpsAim(uint32_t now){
  if(!arcValid || !brakeOpposesYaw(now))return false;
  const float e=arcCentre-yaw, v=yawRateFiltered;
  if(e*v < -0.12f)return true; // moving away
  const float stopDist=v*v/(2.0f*BRAKE_ACCEL_DPS2);
  return e*v>0.12f && stopDist>=0.65f*absf(e);
}
bool brakeWindow(uint32_t now){
  if(state==FAST_BRAKE || state==SETTLE || state==ARC_BRAKE){
    return brakeOpposesYaw(now) &&
      (uint32_t)(now-enteredMs)%BRAKE_PERIOD_MS<BRAKE_ON_MS;
  }
  if(state==AIM)return brakeHelpsAim(now) &&
    (uint32_t)(now-enteredMs)%AIM_BRAKE_PERIOD_MS<BRAKE_ON_MS;
  if(state==FINE || state==FIRE){
    const uint16_t phase=(uint32_t)(now-fineStartedMs)%FINE_PERIOD_MS;
    return brakeHelpsAim(now) && phase>=FINE_BRAKE_START_MS &&
      phase<FINE_BRAKE_START_MS+FINE_BRAKE_ON_MS;
  }
  return false;
}
void motorUpdate(uint32_t now){
  if(now-lastMotorMs<MOTOR_PERIOD_MS)return;
  const float dt=(now-lastMotorMs)*0.001f;
  lastMotorMs=now;
  if(state==BOOT||state==SELFTEST||state==HOLD||state==ENC_DIAG||state==CLEAR){
    if(pwmApplied || motorMode!=MODE_COAST)motorOff();
    return;
  }
  // Only electrically brake if fresh, signed encoder feedback proves direction.
  if(brakeWindow(now)){
    if(motorMode==MODE_DRIVE){
      motorOff();motorBrakeReadyAt=now+BRAKE_PREP_MS;return;
    }
    if(timeElapsed(now,motorBrakeReadyAt,0))motorElectricalBrake();
    else motorOff();
    return;
  }
  if(motorMode==MODE_BRAKE){
    motorOff();brakeReleaseUntil=now+BRAKE_RELEASE_MS;return;
  }
  float goal=0;
  if(state==SEEK)goal=cruisePWM();
  else if(state==FAST_BRAKE)goal=bodyBrakePWM(FAST_BRAKE_MAX_PWM);
  else if(state==BACKOUT)goal=ratePWM(-firstSweepSign*BACKOUT_DPS,BACKOUT_PWM_MAX);
  else if(state==SETTLE)goal=bodyBrakePWM(ARC_BRAKE_MAX_PWM);
  else if(state==SLOW_SCAN)goal=ratePWM(firstSweepSign*SCAN_BODY_DPS,SCAN_PWM_MAX);
  else if(state==ARC_BRAKE)goal=bodyBrakePWM(ARC_BRAKE_MAX_PWM);
  else if(state==EXIT_ARC)goal=ratePWM(firstSweepSign*EXIT_RATE_DPS,SCAN_PWM_MAX);
  else if(state==UNWIND){
    goal=clampf(-wheelRPM*ENCODER_SIGN*1.30f,-UNWIND_PWM_LIMIT,UNWIND_PWM_LIMIT);
  }
  else if(state==AIM)goal=coarsePWM();
  else if(state==FINE)goal=finePWM(now);
  else if(state==FIRE){
    // Don't kick the wheel while already essentially settled; tiny pulses
    // are reserved for correcting genuine position/rate drift.
    goal=(absf(arcCentre-yaw)>0.65f || absf(yawRateFiltered)>0.95f)
      ? finePWM(now):0;
  }
  // No sudden discontinuities outside fine mode. Fine pulse envelope supplies
  // its own rise/fall ramp and must not be stretched by large slew memory.
  const bool micro=(state==FINE||state==FIRE);
  const float rate=(micro?22000.0f:
      (state==FAST_BRAKE||state==BACKOUT||state==SETTLE||
       state==SLOW_SCAN||state==ARC_BRAKE ? BRAKE_SLEW_PER_SEC:MOTOR_SLEW_PER_SEC));
  const float step=rate*dt;
  pwmGoal=clampf(goal,-MOTOR_PWM_LIMIT,MOTOR_PWM_LIMIT);
  float next=pwmActual+clampf(pwmGoal-pwmActual,-step,step);
  if(pwmActual*next<0)next=0;
  if(pwmApplied!=0&&next==0&&pwmGoal!=0&&signf(pwmApplied)!=signf(pwmGoal))
    reverseUntil=now+REVERSE_DEADTIME_MS;
  if((int32_t)(now-reverseUntil)<0 || !timeElapsed(now,brakeReleaseUntil,0))next=0;
  pwmActual=next;
  pwmApplied=(int16_t)(next+(next>=0?0.5f:-0.5f));
  motorHardware(pwmApplied);
}

// ------------------------------- LOCK -------------------------------------
float fireTolerance(){
  // Width-proportional lock window, tight enough to avoid the IR cone edge.
  return clampf(arcWidth*0.42f,FIRE_ANGLE_MIN_TOL_DEG,FIRE_ANGLE_TOL_DEG);
}
// Entry gate: must be accurate and receive recent light.
bool centred(uint32_t now){
  return calibrated && encoderProven && arcValid && irRecent(now) &&
    absf(arcCentre-yaw)<=fireTolerance() &&
    absf(yawRateFiltered)<=FIRE_RATE_TOL_DPS &&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}
// Hold gate uses small yaw/rate hysteresis so jitter doesn't flicker D4.
// IR loss has a very short grace, never bypassing a known invalid/stuck input.
bool maintainLock(uint32_t now){
  const bool fresh=irRecent(now);
  if(fresh)lastLockIrAt=now;
  const bool shortGap=!irStuckReported && ir.active && lastLockIrAt &&
      timeElapsed(now,lastLockIrAt,0) &&
      (uint32_t)(now-lastLockIrAt)<=FIRE_IR_HOLD_MS;
  return calibrated && encoderProven && arcValid && (fresh||shortGap) &&
    absf(arcCentre-yaw)<=fireTolerance()+FIRE_HOLD_EXTRA_DEG &&
    absf(yawRateFiltered)<=FIRE_HOLD_RATE_DPS &&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}
const char* gate(uint32_t now){
  if(!encoderProven)return "ENC";
  if(!arcValid)return "NOARC";
  if(irStuckReported)return "STUCK";
  if(!irRecent(now))return "NOIR";
  if(absf(arcCentre-yaw)>fireTolerance())return "ANGLE";
  if(absf(yawRateFiltered)>FIRE_RATE_TOL_DPS)return "RATE";
  if(absf(wheelRPM)>MAX_FIRE_WHEEL_RPM)return "WHEEL";
  if(state!=AIM && state!=FINE && state!=FIRE)return "STATE";
  if(stableSince && (uint32_t)(now-stableSince)<FIRE_CONFIRM_MS)return "WAIT";
  return "READY";
}
// Works from either AIM or FINE. A transient READY at the end of AIM no longer
// gets discarded when the state transitions to FINE in the same loop.
bool updateLaserArming(uint32_t now){
  if(!centred(now)){stableSince=0;return false;}
  if(!stableSince){stableSince=now;Serial.println(F("LOCK_CANDIDATE,gyro_and_IR_agree"));}
  if(timeElapsed(now,stableSince,FIRE_CONFIRM_MS)){
    lastLockIrAt=now;
    laserOn();
    ++fires;
    enter(FIRE);
    Serial.println(F("LASER_ON,centred_with_confirmed_active_LOW_IR"));
    return true;
  }
  return false;
}
void sequence(uint32_t now){
  // Gyro rates over 90 dps are real motion on this suspended rig, not failures.
  // Precision-lock thresholds continue to prevent firing while moving fast.
  switch(state){
    case BOOT:
      if(timeElapsed(now,enteredMs,BOOT_WAIT_MS))startStartupSelfTest();
      break;
    case SELFTEST:
      // The only boot SELFTEST is the short self-test. Uses your known X bias.
      if(timeElapsed(now,enteredMs,LASER_CAL_SELFTEST_MS+15)){
        laserCalActive=false;laserOff();
        // Define relative yaw from the end of the startup test, even if the
        // chassis moved during it. This is NOT a gyro bias calibration.
        yaw=0;yawRate=0;yawRateFiltered=0;lastGyroUs=micros();
        beginClear();
        Serial.println(F("SELFTEST_DONE,source=STORED_X_REFERENCE,stationary_calibration=SKIPPED"));
      }
      break;
    case CLEAR:
      // Do not deadlock if the receiver starts inside an IR cone. SEEK only
      // starts measuring after a *fresh* IR_ON transition.
      if(timeElapsed(now,enteredMs,EXIT_CLEAR_MS))enter(SEEK);
      break;
    case SEEK:
      // Full available 7.5 V PWM builds wheel acceleration quickly.
      // The first confirmed IR event causes gyro braking, not arc measurement.
      if(absf(wheelRPM)>WHEEL_UNWIND_TRIGGER_RPM &&
         absf(yawRateFiltered)<NO_YAW_THRESHOLD_DPS){
        if(!slowYawSince)slowYawSince=now;
        if(now-slowYawSince>NO_YAW_UNWIND_MS){
          slowYawSince=0;unwindSince=0;enter(UNWIND);
          Serial.println(F("MOMENTUM_RESET,body_stalled_at_speed"));
        }
      }else slowYawSince=0;
      break;
    case FAST_BRAKE:
      // Stop satellite yaw by applying torque against current angular rate.
      if(absf(yawRateFiltered)<FAST_BRAKE_RATE_OK_DPS){
        if(!phaseStillAt)phaseStillAt=now;
        if(now-phaseStillAt>=FAST_BRAKE_STILL_MS){
          Serial.print(F("FAST_BRAKE_DONE,yaw="));Serial.println(yaw,2);
          startBackout();
        }
      }else phaseStillAt=0;
      if(timeElapsed(now,enteredMs,FAST_BRAKE_TIMEOUT_MS+1)){
        Serial.println(F("FAST_BRAKE_TIMEOUT: resuming clearance, not latching HOLD"));
        beginClear();
      }
      break;
    case BACKOUT:{
      // Return PAST the IR entry edge, on the original approach side.
      // We require BOTH position and the sensor being fully clear.
      const bool pastEdge=firstSweepSign*(yaw-backoutTargetYaw)<=0;
      if(pastEdge && !ir.active){
        if(!backoutClearAt)backoutClearAt=now;
        if(now-backoutClearAt>=IR_CLEAR_MS){
          phaseStillAt=0;enter(SETTLE);
          Serial.println(F("BACKOUT_CLEAR,ready_for_second_pass"));
        }
      }else backoutClearAt=0;
      if(timeElapsed(now,enteredMs,BACKOUT_TIMEOUT_MS+1)){
        Serial.println(F("BACKOUT_TIMEOUT: retry SEEK"));beginClear();
      }
      break;
    }
    case SETTLE:
      // Brake the reverse movement before heading towards beacon again.
      if(absf(yawRateFiltered)<STOPPED_DPS){
        if(!phaseStillAt)phaseStillAt=now;
        if(now-phaseStillAt>=PRE_SCAN_STILL_MS){
          slowArcStarted=false;
          phaseStillAt=0;
          enter(SLOW_SCAN);
          Serial.print(F("SLOW_PASS_START,target_body_dps="));
          Serial.println(firstSweepSign*SCAN_BODY_DPS,2);
        }
      }else phaseStillAt=0;
      if(timeElapsed(now,enteredMs,PRE_SCAN_TIMEOUT_MS+1)){
        Serial.println(F("SETTLE_TIMEOUT: retry SEEK"));beginClear();
      }
      break;
    case SLOW_SCAN:
      // Closed-loop gyro yaw speed, not a fixed flywheel RPM.
      if(absf(yawRateFiltered)>SLOW_SCAN_MAX_RATE_DPS){
        Serial.println(F("SLOW_SCAN_TOO_FAST: reject arc and retry"));rescanOrSeek();break;
      }
      if(timeElapsed(now,enteredMs,SLOW_SCAN_TIMEOUT_MS+1)){
        ++rejectedArcs;
        Serial.println(F("ARC_BAD,reason=slow_pass_timeout"));
        rescanOrSeek();
      }
      break;
    case ARC_BRAKE:
      if(absf(yawRateFiltered)<STOPPED_DPS){
        if(!phaseStillAt)phaseStillAt=now;
        if(now-phaseStillAt>=ARC_BRAKE_STILL_MS){
          enter(AIM);
          Serial.print(F("ARC_BRAKE_DONE,centre="));Serial.println(arcCentre,2);
        }
      }else phaseStillAt=0;
      if(timeElapsed(now,enteredMs,ARC_BRAKE_TIMEOUT_MS+1)){
        Serial.println(F("ARC_BRAKE_TIMEOUT: return SEEK; no permanent halt"));
        beginClear();
      }
      break;
    case UNWIND:
      if(absf(wheelRPM)<UNWIND_RPM_FINISH){
        if(!unwindSince)unwindSince=now;
        if(now-unwindSince>=300)beginClear();
      }else unwindSince=0;
      break;
    case AIM:
      if(updateLaserArming(now))break;
      if(absf(arcCentre-yaw)<=FINE_ENTER_DEG&&absf(yawRateFiltered)<=FINE_ENTER_DPS){
        fineStartedMs=now;enter(FINE);
      }
      break;
    case FINE:
      if(absf(arcCentre-yaw)>FINE_REENTER_DEG){enter(AIM);break;}
      if(updateLaserArming(now))break;
      if(arcValid&&absf(arcCentre-yaw)<fireTolerance()&&!irRecent(now)){
        if(!noIrAt)noIrAt=now;
        if(now-noIrAt>NO_IR_AT_CENTRE_RESCAN_MS){
          Serial.println(F("REACQUIRE,centre_without_recent_IR"));
          noIrAt=0;rescanOrSeek();
        }
      }else noIrAt=0;
      break;
    case FIRE:
      if(!maintainLock(now)){
        laserOff();fineStartedMs=now;enter(FINE);
        Serial.println(F("LASER_OFF,IR_OR_STABLE_ALIGNMENT_LOST"));
      }else if(timeElapsed(now,enteredMs,FIRE_DURATION_MS)){
        laserOff();exitClearSince=0;enter(EXIT_ARC);
        Serial.println(F("FIRE_COMPLETE,2000ms,EXIT_ARC"));
      }
      break;
    case EXIT_ARC:
      if(!ir.active && !ir.low){
        if(!exitClearSince)exitClearSince=now;
        if(timeElapsed(now,exitClearSince,IR_CLEAR_MS)){
          Serial.println(F("EXIT_CONFIRMED,RETURN_SEEK"));beginClear();
        }
      }else exitClearSince=0;
      if(state==EXIT_ARC && timeElapsed(now,enteredMs,EXIT_ARC_TIMEOUT_MS)){
        Serial.println(F("EXIT_TIMEOUT,RETURN_SEEK"));beginClear();
      }
      break;
    case ENC_DIAG:break;
    case HOLD:
      // Hardware fault hold ONLY: real I2C failure or measured wheel overspeed.
      // Normal rotation, acceleration and calibration never enter HOLD.
      laserOff();
      if(failedI2C==0 && absf(wheelRPM)<WHEEL_UNWIND_TRIGGER_RPM &&
         timeElapsed(now,enteredMs,FAULT_RETRY_MS)){
        Serial.print(F("HARDWARE_FAULT_CLEARED,previous_code="));Serial.println(stopCode);
        encoderProven=false;beginClear();
      }
      break;
  }
}

// ----------------------------- TELEMETRY ----------------------------------
float estimatedStopDeg(){
  // Diagnostic estimate only: BRAKE_ACCEL_DPS2 must be measured on the rig.
  return yawRateFiltered*yawRateFiltered/(2.0f*BRAKE_ACCEL_DPS2);
}
void header(){
  Serial.println(F(" Time | State  | Yaw(deg) | Rate(dps) | IR | Raw | Puls | PWM  | Mode | RPM   | Centre | Error | Stop | Enc | Gate  | LZR | Lowms | Brk | Proof"));
  Serial.println(F("------+--------+----------+-----------+----+-----+------+-------+-------+--------+-------+------+-----+-------+-----+------"));
}
void printFloat(float x,int width,int decimals){char buf[20];dtostrf(x,width,decimals,buf);Serial.print(buf);}
void telemetry(uint32_t now){
  if(now-lastPrintMs<PRINT_INTERVAL_MS)return;
  lastPrintMs=now;
  static uint8_t lines=0;
  if((lines++%10)==0)header();
  printFloat(now*0.001f,5,1);Serial.print(F(" | "));
  char st[8];strncpy(st,stateName(),7);st[7]=0;
  Serial.print(st);for(uint8_t i=strlen(st);i<6;i++)Serial.print(' ');
  Serial.print(F(" | "));printFloat(yaw,8,2);
  Serial.print(F(" | "));printFloat(yawRateFiltered,9,2);
  Serial.print(F(" |  "));Serial.print(ir.active?1:0);
  Serial.print(F(" |  "));Serial.print(ir.low?1:0);
  Serial.print(F(" | "));printFloat(ir.pulsesDisplayed,4,0);ir.pulsesDisplayed=0;
  Serial.print(F(" | "));printFloat(pwmApplied,5,0);
  Serial.print(F(" | "));Serial.print(motorMode==MODE_BRAKE?F("BRK"):
    (motorMode==MODE_DRIVE?F("DRV"):F("OFF")));
  Serial.print(F(" | "));printFloat(wheelRPM,5,1);
  Serial.print(F(" | "));
  if(arcValid)printFloat(arcCentre,6,2);else Serial.print(F("    --"));
  Serial.print(F(" | "));
  if(arcValid)printFloat(arcCentre-yaw,5,2);else Serial.print(F("   --"));
  Serial.print(F(" | "));printFloat(estimatedStopDeg(),4,1);
  const uint32_t edges=edgeCountNow();
  const uint32_t edgeDelta=edges-lastPrintedEncoderEdges;
  lastPrintedEncoderEdges=edges;
  Serial.print(F(" | "));Serial.print(edgeDelta);
  Serial.print(F(" | "));Serial.print(gate(now));
  Serial.print(F(" |  "));Serial.print(laserHigh?1:0);
  uint32_t lowUs;
  bool low;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){lowUs=lowStartUs;low=isLow;}
  Serial.print(F(" | "));
  if(low)Serial.print((uint32_t)(micros()-lowUs)/1000UL);
  else Serial.print(0);
  Serial.print(F(" | "));Serial.print((uint16_t)(brakeActivations-printedBrakeActivations));
  printedBrakeActivations=brakeActivations;
  Serial.print(F(" | "));Serial.println(encoderProven?1:0);
}
void setup(){
  pinMode(PIN_IN1,OUTPUT);pinMode(PIN_IN2,OUTPUT);pinMode(PIN_LASER,OUTPUT);
  pinMode(PIN_LED,OUTPUT);pinMode(PIN_ENC_A,INPUT_PULLUP);
  pinMode(PIN_ENC_B,INPUT_PULLUP);pinMode(PIN_IR,INPUT_PULLUP);
  motorOff();laserOff();Serial.begin(115200);
  Wire.begin();Wire.setClock(100000UL);
  delay(100);
  if(!mpuWrite(0x6B,0x00)||!mpuWrite(0x1A,0x03)||
     !mpuWrite(0x1B,0x00)||!mpuWrite(0x1C,0x00)||!mpuWrite(0x19,0x04)){
    safeStop(1);
  }
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encoderRise,RISING);
  isLow=(PINB&_BV(PB3))==0;
  if(isLow){lowStartUs=micros();lastLowSeenUs=micros();}
  PCMSK0|=_BV(PCINT3);PCICR|=_BV(PCIE0);
  enteredMs=millis();lastGyroUs=micros();
  Serial.println(F("MQ SENTINEL V18 | FIXED X BIAS | NO STATIONARY RECAL | 22ms/85ms | 2s FIRE"));
#if DRIVER_PHASE_ENABLE
  Serial.println(F("DRV=PH/EN PMODE_LOW: D5=EN(PWM),D6=PH; HIGH/HIGH is DRIVE"));
#else
  Serial.println(F("DRV=PWM PMODE_HIGH: D5=IN1,D6=IN2; HIGH/HIGH is BRAKE"));
#endif
  Serial.println(F("Encoder proof: startup PWM185 up to 1100ms; if no 3 D3 edges ENC_DIAG; t=retry"));
  Serial.println(F("NO MOVING_PLATFORM_CAL or gyro-motion HOLD. Hardware hold only 1=I2C,2=RPM (auto-retry if safe)."));
  Serial.println(F("D5/D6 MOTOR | D4 LASER | D3/D2 ENCODER | D11 IR | A4/A5 GYRO"));
  Serial.println(F("NOTICE: an MCU brownout/reset during spinning restarts firmware; stored bias cannot fix supply drops."));
  Serial.print(F("LOCK_TOL_MAX="));Serial.print(FIRE_ANGLE_TOL_DEG,2);
  Serial.print(F(",RATE_MAX="));Serial.print(FIRE_RATE_TOL_DPS,2);
  Serial.print(F(",CONFIRM_MS="));Serial.print(FIRE_CONFIRM_MS);
  Serial.print(F(",IR_AGE_MS="));Serial.print(IR_RECENT_MS);
  Serial.print(F(",STUCK_LOW_MS="));Serial.println(IR_MAX_CONTINUOUS_LOW_MS);
  Serial.print(F("X_BIAS_RAW_REFERENCE="));Serial.println(GYRO_X_BIAS,4);
  Serial.println(F("GYRO_MODE=STORED_X_BIAS; MOVEMENT_AT_BOOT_OK; NO_RECAL"));
  Serial.print(F("SEEK_FULL_PWM="));Serial.print(SEEK_PWM,0);
  Serial.print(F(",SCAN_BODY_RATE_DPS="));Serial.print(SCAN_BODY_DPS,1);
  Serial.print(F(",MAX_PWM="));Serial.print(MOTOR_PWM_LIMIT,0);
  Serial.print(F(",YAW_AXIS="));Serial.print(YAW_AXIS==0?'X':(YAW_AXIS==1?'Y':'Z'));
  Serial.print(F(",LASER_ENABLED="));Serial.println(LASER_ENABLED);
  Serial.print(F("ENCODER_START_LIMIT_MS="));Serial.print(ENCODER_START_LIMIT_MS);
  Serial.println(F(",ENC_DIAG_ON_MISSING_EDGES"));
}
void loop(){
  const uint32_t now=millis();
  // Continue polling all sensors. Ordinary rotation never enters HOLD.
  // Hardware holds auto-retry when sensors and wheel speed are safe.
  gyroUpdate(now);
  serviceStartupLaser(millis());
  irUpdate(now);
  rpmUpdate(now);
  while(Serial.available()){
    const char c=(char)Serial.read();
    if(c=='t' && state==ENC_DIAG && absf(wheelRPM)<8.0f){
      encoderProven=false;Serial.println(F("MANUAL_ENCODER_RETEST"));beginClear();
    }
  }
  sequence(now);
  motorUpdate(now);
  digitalWrite(PIN_LED,state==HOLD?((now/125)&1):laserHigh);
  telemetry(now);
}
