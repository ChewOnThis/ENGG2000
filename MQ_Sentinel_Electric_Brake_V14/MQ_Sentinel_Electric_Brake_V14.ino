/*
  MQ SENTINEL V14 — V13 + drive/brake PWM motor stage + yaw-drift fixes.
  V14 changes vs V13:
   1. motorHardware(): drive/brake PWM. The held-HIGH pin sets direction, the
      other pin is inverted PWM (low = drive, high = brake). p=0 = full BRAKE
      (not coast). motorMode is kept consistent (V13 would lock after 1st brake).
   2. CAL verifies gravity lies along YAW_AXIS (stop code 10 = AXIS_MISMATCH).
   2b. YAW_AXIS 0 = MPU X added; measured X bias (6 bench tests) loaded as prior.
   3. CAL uses the MEASURED bias when still (no 20% blend with a stale prior),
      rejects a moving rig (SD 0.30 dps) and retries up to 3 times.
   4. MPU sample-rate divider 0 (1 kHz) so every 5 ms poll reads a fresh sample.
   5. I2C timeout (when the core supports it) so a bus glitch cannot hang loop().
   6. Stray laserOn() removed from startCalibration() (laser stayed on in CAL).
  MQ SENTINEL V13 — encoder-verified electrical braking + reverse micro-corrections.
  Classic Nano ATmega328P; DRV8874 IN/IN with PMODE and nSLEEP HIGH.
  D5 IN1, D6 IN2, D4 eye-safe driven laser, D3 encoder A, D2 encoder B,
  D11 active-LOW 38 kHz demodulator, A4/A5 MPU6050, D13 lock indicator.

  BOOT -> CAL -> CLEAR -> SEEK (full 7.5 V PWM, subject to wheel cutoff)
  -> FAST_BRAKE at first IR -> BACKOUT to the approach side of IR cone
  -> SETTLE -> SLOW_SCAN (gyro rate regulation, true complete ON/OFF arc)
  -> ARC_BRAKE (IN1=IN2=HIGH pulses where torque opposes yaw)
  -> AIM (predictive stopping) -> FINE (drive micro-pulses + conditional brake pulses)
  -> FIRE (only fresh IR + measured arc centre + stable low gyro rate).

  Important: reaction-wheel torque comes from acceleration, NOT steady RPM.
  BODY_SIGN and ENCODER_SIGN must be verified on the physical suspended rig.
  Soft and hard wheel RPM thresholds depend on correct encoder scaling.
  STOP keeps gyro/IR/encoder telemetry alive. Code 9 detects absent encoder edges.
  CAL runs a brief D4 laser self-test. Use ONLY a driven eye-safe laser module
  aimed into a beam stop; disconnect its power for initial motor tests.
  Dynamic braking requires valid, correctly signed encoder feedback.
  Brake shorts the MOTOR terminals via the DRV8874, not the supply rails.
  First test with mechanically contained flywheel.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// ------------------------- HARDWARE ---------------------------------------
const uint8_t PIN_IN1=5, PIN_IN2=6, PIN_LASER=4;
const uint8_t PIN_ENC_A=3, PIN_ENC_B=2, PIN_IR=11, PIN_LED=13, MPU=0x68;
#define LASER_ENABLED 1         // D4 HIGH only if validated lock, eye-safe module
const int8_t BODY_SIGN=-1;     // +IN1 produces negative body angular accel: VERIFY
const int8_t ENCODER_SIGN=+1;  // +IN1 produces positive encoder counts: VERIFY
const uint8_t YAW_AXIS=0;      // 0=MPU X; 1=MPU Y; 2=MPU Z (X = bench-measured axis)
// Measured X gyro bias, 6 bench tests @ ~26.4 C: raw mean 24.3267 counts
// (min 23.9250, max 24.8417; spread 0.007 dps) = 0.18570 dps at 131 LSB/dps.
const float GYRO_X_BIAS=24.3267f;                    // raw counts, +/-250 dps range
const float PRIOR_X_BIAS_DPS=GYRO_X_BIAS/131.0f;
const float PRIOR_Y_BIAS_DPS=84.4403f/131.0f;
const float PRIOR_Z_BIAS_DPS=-0.12317f;
// User-requested short CAL-stage laser-output self-test. Never a bare diode.
#define LASER_CAL_SELFTEST_ENABLED 1
const uint16_t LASER_CAL_SELFTEST_MS=350;
// CAL checks that gravity lies mostly along YAW_AXIS (rig hanging level).
#define AXIS_CHECK_ENABLED 1
const float AXIS_MIN_GRAVITY_G=0.85f;
const float CAL_PRIOR_TOL_DPS=0.30f;  // sanity window only; measured bias wins
const uint8_t CAL_MAX_RETRIES=3;


// -------------------------- SENSORS ---------------------------------------
const uint16_t CAL_SAMPLES=1000, GYRO_INTERVAL_MS=5;
const float GYRO_COUNTS_PER_DPS=131.0f, ACC_COUNTS_PER_G=16384.0f;
const float CAL_MAX_SD_DPS=0.30f, CAL_ACC_TOL=0.15f;
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

// DRV8874 PWM/IN-IN mode truth table (PMODE HIGH):
// 0/0 COAST, 1/0 forward, 0/1 reverse, 1/1 LOW-SIDE BRAKE.
// Braking decelerates an ALREADY SPINNING wheel. It cannot choose the
// direction of wheel torque; it is safe for yaw control only if encoder
// direction confirms that braking opposes the measured chassis yaw.
const float ELECTRIC_BRAKE_MIN_WHEEL_RPM=12.0f;
const uint16_t ELECTRIC_BRAKE_ENCODER_FRESH_MS=400;
const uint16_t ELECTRIC_BRAKE_DEADTIME_MS=10;
const uint16_t FAST_BRAKE_PULSE_PERIOD_MS=115, FAST_BRAKE_PULSE_ON_MS=28;
const uint16_t ARC_BRAKE_PULSE_PERIOD_MS=105, ARC_BRAKE_PULSE_ON_MS=24;
const uint16_t AIM_BRAKE_PULSE_PERIOD_MS=115, AIM_BRAKE_PULSE_ON_MS=18;
const uint16_t FINE_BRAKE_PULSE_START_MS=56, FINE_BRAKE_PULSE_MS=22;
const float BRAKE_MIN_BODY_RATE_DPS=0.65f;
const float AIM_OVERSHOOT_BRAKE_RATIO=0.72f;


// ------------------------- AIMING / LOCK -----------------------------------
// Slower gyro-based AIM than SEEK. Fine and FIRE use only micro-pulses.
const float COARSE_RATE_LIMIT_DPS=6.0f;
const float COARSE_KP_POS=1.3f, COARSE_KP_RATE=6.0f;
const float COARSE_FRICTION_PWM=38.0f, COARSE_MAX_PWM=115.0f;
const float BRAKE_ACCEL_DPS2=12.0f;  // cautious provisional DECEL estimate; measure on rig
const float AIM_CURVE_WIDTH_DEG=10.0f;
const float FINE_ENTER_DEG=3.2f, FINE_REENTER_DEG=4.4f;
const float FINE_ENTER_DPS=8.0f;
const uint16_t FINE_PERIOD_MS=145, FINE_PULSE_MS=45;
const float FINE_MAX_PWM=74.0f, FINE_KP_POS=22.0f, FINE_KD_RATE=10.0f;
const float FINE_MIN_ERROR=0.18f;
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
  BOOT, CAL, CLEAR, SEEK, FAST_BRAKE, BACKOUT, SETTLE,
  SLOW_SCAN, ARC_BRAKE, AIM, FINE, FIRE, UNWIND, STOP
};
State state=BOOT;
uint32_t enteredMs=0, clearSince=0, lastPrintMs=0, lastGyroMs=0, lastGyroUs=0;
uint32_t lastMotorMs=0, reverseUntil=0, slowYawSince=0, unwindSince=0;
uint32_t fineStartedMs=0, stableSince=0, noIrAt=0, lastLockIrAt=0;
uint32_t phaseStillAt=0, backoutClearAt=0;
float firstIrYaw=0, firstSweepSign=1.0f, backoutTargetYaw=0;
uint8_t slowScanRetries=0;
bool slowArcStarted=false;
uint8_t stopCode=0, failedI2C=0;
uint16_t calN=0, irOnCount=0, irOffCount=0, validArcs=0, rejectedArcs=0, fires=0;
float calMean=0, calM2=0, calSD=0, calMinG=10, calMaxG=0;
float axG=0, ayG=0, azG=0, calAx=0, calAy=0, calAz=0;
uint8_t calRetries=0;
float gyroBias=0, yaw=0, yawRate=0, yawRateFiltered=0, rawRate=0, accelG=1, tempC=0;
bool calibrated=false, fallbackBias=false, laserHigh=false, irStuckReported=false;
bool laserSelfTestActive=false;
uint32_t laserSelfTestStarted=0;
enum MotorMode:uint8_t { MOTOR_IDLE, MOTOR_DRIVE, MOTOR_BRAKE };
MotorMode motorMode=MOTOR_IDLE;
uint32_t brakeReadyAt=0;
uint16_t brakeActivations=0, previousPrintedBrakeActivations=0;

float wheelRPM=0, rpmInstant=0;
int32_t prevTicks=0;
uint32_t lastRPMMs=0;
volatile int32_t encoderTicks=0;
volatile uint32_t encoderRisingEdges=0;
uint32_t previousEncoderEdges=0, lastEncoderEdgeAt=0, lastPrintedEncoderEdges=0;
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
    case BOOT:return "BOOT";case CAL:return "CAL";case CLEAR:return "CLEAR";
    case SEEK:return "SEEK";case FAST_BRAKE:return "BRAKE";
    case BACKOUT:return "BACKOUT";case SETTLE:return "SETTLE";
    case SLOW_SCAN:return "SCAN";case ARC_BRAKE:return "ARC_BRK";
    case AIM:return "AIM";
    case FINE:return "FINE";case FIRE:return "FIRE";
    case UNWIND:return "UNWIND";default:return "STOP";
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
bool timeElapsed(uint32_t now,uint32_t since,uint32_t waitMs); // forward declaration
// DRIVE/BRAKE PWM: the held-HIGH pin sets direction; the other pin carries
// inverted PWM (LOW = drive, HIGH = brake). p=+255 full drive, p=0 full BRAKE.
// motorMode==MOTOR_IDLE means "zero command" (the H-bridge is BRAKING, not coasting).
void motorHardware(int16_t p){
  if(p>255)p=255; else if(p<-255)p=-255;
  if(p>0){analogWrite(PIN_IN1,255);analogWrite(PIN_IN2,255-p);motorMode=MOTOR_DRIVE;}
  else if(p<0){analogWrite(PIN_IN2,255);analogWrite(PIN_IN1,255+p);motorMode=MOTOR_DRIVE;}
  else {analogWrite(PIN_IN1,255);analogWrite(PIN_IN2,255);motorMode=MOTOR_IDLE;}
}
// In DRV8874 PWM (IN/IN) mode, HIGH/HIGH = low-side short braking.
// Use only with verified encoder direction and controlled pulse timing.
void motorElectricalBrake(){
  if(motorMode!=MOTOR_BRAKE)++brakeActivations;
  digitalWrite(PIN_IN1,HIGH);
  digitalWrite(PIN_IN2,HIGH);
  motorMode=MOTOR_BRAKE;
}
// Safety stop now BRAKES (drive/brake PWM, zero command = both inputs HIGH).
// At high wheel RPM expect a hard stop. A reset still releases to coast,
// because the DRV8874 inputs float to pull-down while the MCU is resetting.
void motorOff(){pwmGoal=pwmActual=0;pwmApplied=0;motorHardware(0);}
void laserSelfTestStart(){
#if LASER_ENABLED && LASER_CAL_SELFTEST_ENABLED
  laserSelfTestStarted=millis();
  laserSelfTestActive=true;
  Serial.println(F("LASER_CAL_TEST_ON,D4_HIGH,350ms,beam_stop_required"));
  digitalWrite(PIN_LASER,HIGH);
  laserHigh=true;
#endif
}
void serviceLaserSelfTest(uint32_t now){
  if(laserSelfTestActive &&
     (state!=CAL || timeElapsed(now,laserSelfTestStarted,LASER_CAL_SELFTEST_MS))){
    laserOff();laserSelfTestActive=false;
    Serial.println(F("LASER_CAL_TEST_OFF,D4_LOW"));
  }
}
void enter(State s){
  state=s;enteredMs=millis();
  // Do not unnecessarily erase confirmation when AIM -> FINE while already
  // centred; both states are allowed to complete laser confirmation.
  if(s!=AIM && s!=FINE)stableSince=0;
  if(s==SEEK)lastEncoderEdgeAt=enteredMs;
  Serial.print(F("STATE,"));Serial.print(stateName());
  Serial.print(F(",yaw="));Serial.print(yaw,2);
  Serial.print(F(",ir="));Serial.println(ir.active?1:0);
}
void safeStop(uint8_t c){
  if(state==STOP)return;
  stopCode=c;laserSelfTestActive=false;laserOff();motorOff();enter(STOP);
  Serial.print(F("SAFETY_STOP,code="));Serial.println(c);
  Serial.println(F("DIAGNOSTICS_CONTINUE: motor and laser OFF; gyro and IR still sampled"));
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
  axG=x;ayG=y;azG=z;
  g=sqrtf(x*x+y*y+z*z);
  tempC=v[3]/340.0f+36.53f;
  dps=v[4+YAW_AXIS]/GYRO_COUNTS_PER_DPS;   // 4=gx 5=gy 6=gz
  return true;
}
void startCalibration(){
  motorOff();calibrated=false;calRetries=0;calAx=calAy=calAz=0;
  calN=0;calMean=calM2=calSD=0;calMinG=10;calMaxG=0;
  enter(CAL);
  laserSelfTestStart();  // Fixed-duration startup D4 output verification
}
void gyroUpdate(uint32_t now){
  if(now-lastGyroMs<GYRO_INTERVAL_MS)return;
  lastGyroMs=now;
  const uint32_t us=micros();
  float r,g;
  if(!mpuRead(r,g)){
    if(++failedI2C>=4)safeStop(1);
    return;
  }
  failedI2C=0;rawRate=r;accelG=g;
  if(state==CAL){
    ++calN;const float delta=r-calMean;
    calMean+=delta/calN;calM2+=delta*(r-calMean);
    calAx+=axG;calAy+=ayG;calAz+=azG;
    if(g<calMinG)calMinG=g;
    if(g>calMaxG)calMaxG=g;
    if(calN>=CAL_SAMPLES){
      calSD=sqrtf(calM2/(calN-1));
      const float prior=YAW_AXIS==0?PRIOR_X_BIAS_DPS:
          (YAW_AXIS==1?PRIOR_Y_BIAS_DPS:PRIOR_Z_BIAS_DPS);
#if AXIS_CHECK_ENABLED
      const float yawAxisG=(YAW_AXIS==0?calAx:(YAW_AXIS==1?calAy:calAz))/calN;
      if(absf(yawAxisG)<AXIS_MIN_GRAVITY_G){
        Serial.print(F("AXIS_MISMATCH,yaw_axis_g="));Serial.print(yawAxisG,3);
        Serial.print(F(",ax="));Serial.print(calAx/calN,3);
        Serial.print(F(",ay="));Serial.print(calAy/calN,3);
        Serial.print(F(",az="));Serial.print(calAz/calN,3);
        Serial.println(F(",set_YAW_AXIS_to_the_axis_reading_about_1g"));
        safeStop(10);
        return;
      }
#endif
      const bool still=calSD<CAL_MAX_SD_DPS&&absf(calMean-prior)<CAL_PRIOR_TOL_DPS&&
          calMinG>1.0f-CAL_ACC_TOL&&calMaxG<1.0f+CAL_ACC_TOL;
      if(!still && calRetries<CAL_MAX_RETRIES){
        ++calRetries;
        Serial.print(F("CAL_RETRY,n="));Serial.print(calRetries);
        Serial.print(F(",sd="));Serial.print(calSD,4);
        Serial.print(F(",mean="));Serial.println(calMean,4);
        calN=0;calMean=calM2=calSD=0;calMinG=10;calMaxG=0;calAx=calAy=calAz=0;
        lastGyroUs=us;
        return;
      }
      fallbackBias=!still;
      gyroBias=still?calMean:prior;
      laserSelfTestActive=false;laserOff();
      calibrated=true;yaw=yawRate=yawRateFiltered=0;lastGyroUs=us;
      Serial.print(F("CAL_DONE,bias_dps="));Serial.print(gyroBias,5);
      Serial.print(F(",sd="));Serial.print(calSD,4);
      Serial.print(F(",temp_C="));Serial.print(tempC,2);
      Serial.print(F(",source="));Serial.println(still?F("MEASURED"):F("KNOWN_FALLBACK"));
      beginClear();
    }
  }else if(calibrated){
    const float dt=(uint32_t)(us-lastGyroUs)*0.000001f;
    if(dt>0.12f||absf(r)>=235||g>1.75f||g<0.36f){safeStop(3);}
    else {
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
  }
  lastGyroUs=us;
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
void firstIrEncounter(float entry){
  firstIrYaw=entry;
  // Choose the sign of real observed body rotation, not the flywheel sign.
  firstSweepSign=absf(yawRateFiltered)>0.6f ? signf(yawRateFiltered)
                 : (float)(SEEK_MOTOR_SIGN*BODY_SIGN);
  slowScanRetries=0;phaseStillAt=0;backoutClearAt=0;
  // Do not suddenly cut PWM: motorUpdate slews through zero with deadtime.
  // This is controlled active braking, not an abrupt coast/reversal.
  laserOff();
  enter(FAST_BRAKE);
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
  if(state==SEEK && abs(pwmApplied)>=ENCODER_WATCH_PWM &&
     timeElapsed(now,lastEncoderEdgeAt,ENCODER_MISSING_TIMEOUT_MS)){
    Serial.print(F("ENCODER_NO_PULSES,edges="));Serial.print(edges);
    Serial.print(F(",pwm="));Serial.print(pwmApplied);
    Serial.println(F(",inspect_D3_A_D2_B_and_encoder_power"));
    safeStop(9); // speed cannot be trusted without encoder feedback
  }
  if(lastRPMMs){
    rpmInstant=(ticks-prevTicks)*60000.0f/(COUNTS_PER_REV*(now-lastRPMMs));
    wheelRPM+=0.35f*(rpmInstant-wheelRPM);
    if(absf(wheelRPM)>WHEEL_HARD_RPM||absf(rpmInstant)>WHEEL_HARD_RPM+8.0f)safeStop(2);
  }
  lastRPMMs=now;prevTicks=ticks;
}
float cruisePWM(){
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
// The encoder is essential: if the counter reads 0 despite motor motion,
// electric brake torque direction is UNKNOWN. Never guess the sign.
bool encoderValidForBrake(uint32_t now){
  return lastEncoderEdgeAt!=0 &&
      timeElapsed(now,lastEncoderEdgeAt,0) &&
      (uint32_t)(now-lastEncoderEdgeAt)<=ELECTRIC_BRAKE_ENCODER_FRESH_MS &&
      absf(wheelRPM)>=ELECTRIC_BRAKE_MIN_WHEEL_RPM;
}
float brakeBodyDirection(){
  // +IN1 wheel acceleration -> BODY_SIGN yaw acceleration;
  // wheel signed RPM = ENCODER_SIGN * wheel direction.
  // Electrical braking wheel torque has opposite sign to wheel rotation.
  return -BODY_SIGN*ENCODER_SIGN*signf(wheelRPM);
}
bool brakeOpposesBodyYaw(uint32_t now){
  return encoderValidForBrake(now) &&
      absf(yawRateFiltered)>=BRAKE_MIN_BODY_RATE_DPS &&
      brakeBodyDirection()*yawRateFiltered < 0;
}
bool brakeWouldHelpAim(uint32_t now){
  if(!arcValid || !brakeOpposesBodyYaw(now))return false;
  const float err=arcCentre-yaw, rate=yawRateFiltered;
  // Apply damping if the body is moving AWAY from the centre, or if its
  // computed stopping distance predicts an overshoot during approach.
  const bool away=err*rate < -0.2f;
  const bool toward=err*rate > 0.2f;
  const float predictedStop=rate*rate/(2.0f*BRAKE_ACCEL_DPS2);
  return away || (toward && absf(rate)>1.1f &&
      predictedStop>=AIM_OVERSHOOT_BRAKE_RATIO*absf(err));
}
bool pulseWindow(uint32_t now,uint16_t period,uint16_t on){
  if(!timeElapsed(now,enteredMs,0))return false;
  return (uint32_t)(now-enteredMs)%period<on;
}
bool useElectricalBrakeNow(uint32_t now){
  if(state==FAST_BRAKE || state==SETTLE || state==ARC_BRAKE){
    const uint16_t period=state==FAST_BRAKE?
          FAST_BRAKE_PULSE_PERIOD_MS:ARC_BRAKE_PULSE_PERIOD_MS;
    const uint16_t on=state==FAST_BRAKE?
          FAST_BRAKE_PULSE_ON_MS:ARC_BRAKE_PULSE_ON_MS;
    return brakeOpposesBodyYaw(now) && pulseWindow(now,period,on);
  }
  if(state==AIM){
    return brakeWouldHelpAim(now) &&
        pulseWindow(now,AIM_BRAKE_PULSE_PERIOD_MS,AIM_BRAKE_PULSE_ON_MS);
  }
  if(state==FINE || state==FIRE){
    const uint16_t phase=(uint32_t)(now-fineStartedMs)%FINE_PERIOD_MS;
    return brakeWouldHelpAim(now) &&
        phase>=FINE_BRAKE_PULSE_START_MS &&
        phase<FINE_BRAKE_PULSE_START_MS+FINE_BRAKE_PULSE_MS;
  }
  return false;
}

float finePWM(uint32_t now){
  // Raised-cosine pulse envelope: gradually ramps 0 -> peak -> 0.
  // Micro pulses repeat at 145ms intervals, with zero power between pulses.
  const float err=arcCentre-yaw;
  const float control=FINE_KP_POS*err-FINE_KD_RATE*yawRateFiltered;
  if(absf(err)<FINE_MIN_ERROR && absf(yawRateFiltered)<0.30f)return 0;
  const uint16_t phase=(uint16_t)((now-fineStartedMs)%FINE_PERIOD_MS);
  if(phase>=FINE_PULSE_MS)return 0;
  const float x=phase/(float)FINE_PULSE_MS;
  const float envelope=0.5f-0.5f*cosf(6.2831853f*x);
  const float amplitude=clampf(absf(control),0,FINE_MAX_PWM);
  return BODY_SIGN*signf(control)*amplitude*envelope;
}
void motorUpdate(uint32_t now){
  if(now-lastMotorMs<MOTOR_PERIOD_MS)return;
  const float dt=(now-lastMotorMs)*0.001f;
  lastMotorMs=now;
  if(state==BOOT||state==CAL||state==STOP){
    if(motorMode!=MOTOR_IDLE || pwmApplied)motorOff();
    return;
  }
  // Short HIGH/HIGH braking pulses in selected states. Switch through
  // COAST and a small deadtime before shorting the motor terminals.
  // If wheel direction is unknown, fall back to ordinary driven correction.
  if(useElectricalBrakeNow(now)){
    if(motorMode==MOTOR_DRIVE){
      motorHardware(0);pwmGoal=pwmActual=0;pwmApplied=0;
      brakeReadyAt=now+ELECTRIC_BRAKE_DEADTIME_MS;
      return;
    }
    if(timeElapsed(now,brakeReadyAt,0)){
      pwmGoal=pwmActual=0;pwmApplied=0;
      motorElectricalBrake();
    } else motorHardware(0);
    return;
  }
  if(motorMode==MOTOR_BRAKE){
    motorHardware(0);pwmGoal=pwmActual=0;pwmApplied=0;
    reverseUntil=now+REVERSE_DEADTIME_MS;
    return;
  }
  float goal=0;
  if(state==SEEK)goal=cruisePWM();
  else if(state==FAST_BRAKE)goal=bodyBrakePWM(FAST_BRAKE_MAX_PWM);
  else if(state==BACKOUT)goal=ratePWM(-firstSweepSign*BACKOUT_DPS,BACKOUT_PWM_MAX);
  else if(state==SETTLE)goal=bodyBrakePWM(ARC_BRAKE_MAX_PWM);
  else if(state==SLOW_SCAN)goal=ratePWM(firstSweepSign*SCAN_BODY_DPS,SCAN_PWM_MAX);
  else if(state==ARC_BRAKE)goal=bodyBrakePWM(ARC_BRAKE_MAX_PWM);
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
  const float rate=(micro?2500.0f:
      (state==FAST_BRAKE||state==BACKOUT||state==SETTLE||
       state==SLOW_SCAN||state==ARC_BRAKE ? BRAKE_SLEW_PER_SEC:MOTOR_SLEW_PER_SEC));
  const float step=rate*dt;
  pwmGoal=clampf(goal,-MOTOR_PWM_LIMIT,MOTOR_PWM_LIMIT);
  float next=pwmActual+clampf(pwmGoal-pwmActual,-step,step);
  if(pwmActual*next<0)next=0;
  if(pwmApplied!=0&&next==0&&pwmGoal!=0&&signf(pwmApplied)!=signf(pwmGoal))
    reverseUntil=now+REVERSE_DEADTIME_MS;
  if((int32_t)(now-reverseUntil)<0)next=0;
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
  return calibrated && arcValid && irRecent(now) &&
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
  return calibrated && arcValid && (fresh||shortGap) &&
    absf(arcCentre-yaw)<=fireTolerance()+FIRE_HOLD_EXTRA_DEG &&
    absf(yawRateFiltered)<=FIRE_HOLD_RATE_DPS &&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}
const char* gate(uint32_t now){
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
  switch(state){
    case BOOT:
      if(timeElapsed(now,enteredMs,BOOT_WAIT_MS))startCalibration();
      break;
    case CAL:break;
    case CLEAR:
      // Do not run the wheel while waiting to leave an existing IR beam.
      if(!ir.active){
        if(!clearSince)clearSince=now;
        if(now-clearSince>=IR_CLEAR_MS){clearSince=0;enter(SEEK);}
      }else clearSince=0;
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
      if(timeElapsed(now,enteredMs,FAST_BRAKE_TIMEOUT_MS+1))safeStop(4);
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
      if(timeElapsed(now,enteredMs,BACKOUT_TIMEOUT_MS+1))safeStop(5);
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
      if(timeElapsed(now,enteredMs,PRE_SCAN_TIMEOUT_MS+1))safeStop(6);
      break;
    case SLOW_SCAN:
      // Closed-loop gyro yaw speed, not a fixed flywheel RPM.
      if(absf(yawRateFiltered)>SLOW_SCAN_MAX_RATE_DPS){
        Serial.println(F("SLOW_SCAN_OVERSPEED"));safeStop(7);break;
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
      if(timeElapsed(now,enteredMs,ARC_BRAKE_TIMEOUT_MS+1))safeStop(8);
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
        if(timeElapsed(now,noIrAt,NO_IR_AT_CENTRE_RESCAN_MS+1)){
          Serial.println(F("REACQUIRE,centre_without_recent_IR"));
          noIrAt=0;rescanOrSeek();
        }
      }else noIrAt=0;
      break;
    case FIRE:
      if(!maintainLock(now)){
        laserOff();fineStartedMs=now;enter(FINE);
        Serial.println(F("LASER_OFF,IR_OR_STABLE_ALIGNMENT_LOST"));
      }
      break;
    case STOP:laserOff();break;
  }
}

// ----------------------------- TELEMETRY ----------------------------------
float estimatedStopDeg(){
  // Diagnostic estimate only: BRAKE_ACCEL_DPS2 must be measured on the rig.
  return yawRateFiltered*yawRateFiltered/(2.0f*BRAKE_ACCEL_DPS2);
}
void header(){
  Serial.println(F(" Time | State  | Yaw(deg) | Rate(dps) | IR | Raw | Puls | PWM  | Mode | RPM   | Centre | Error | Stop | Enc | Gate  | LZR | Lowms | Brk"));
  Serial.println(F("------+--------+----------+-----------+----+-----+------+-------+------+-------+--------+-------+------+-----+-------+-----+-------+----"));
}
void printFloat(float x,int width,int decimals){char buf[20];dtostrf(x,width,decimals,buf);Serial.print(buf);}
void calibrationPrint(){
  static uint8_t n=0;
  if((n++%10)==0){
    Serial.println(F(" Time | Samples | Mean(dps) | SD(dps) | Min(g) | Max(g)"));
    Serial.println(F("------+---------+-----------+---------+--------+-------"));
  }
  printFloat(millis()*0.001f,5,1);Serial.print(F(" | "));
  printFloat(calN,7,0);Serial.print(F(" | "));
  printFloat(calMean,9,4);Serial.print(F(" | "));
  printFloat(calN>1?sqrtf(calM2/(calN-1)):0,7,4);Serial.print(F(" | "));
  printFloat(calMinG,6,3);Serial.print(F(" | "));
  printFloat(calMaxG,6,3);Serial.println();
}
void telemetry(uint32_t now){
  if(now-lastPrintMs<PRINT_INTERVAL_MS)return;
  lastPrintMs=now;
  if(state==CAL){calibrationPrint();return;}
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
  Serial.print(F(" | "));
  Serial.print(motorMode==MOTOR_BRAKE?F("BRK"):
      (motorMode==MOTOR_DRIVE?F("DRV"):F("IDL")));
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
  Serial.print(F(" | "));
  Serial.println((uint16_t)(brakeActivations-previousPrintedBrakeActivations));
  previousPrintedBrakeActivations=brakeActivations;
}
void setup(){
  pinMode(PIN_IN1,OUTPUT);pinMode(PIN_IN2,OUTPUT);pinMode(PIN_LASER,OUTPUT);
  pinMode(PIN_LED,OUTPUT);pinMode(PIN_ENC_A,INPUT_PULLUP);
  pinMode(PIN_ENC_B,INPUT_PULLUP);pinMode(PIN_IR,INPUT_PULLUP);
  motorOff();laserOff();Serial.begin(115200);
  Wire.begin();Wire.setClock(100000UL);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000UL,true);   // a glitched bus must not hang loop()
#endif
  delay(100);
  if(!mpuWrite(0x6B,0x00)||!mpuWrite(0x1A,0x03)||
     !mpuWrite(0x1B,0x00)||!mpuWrite(0x1C,0x00)||!mpuWrite(0x19,0x00)){
    safeStop(1);
  }
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encoderRise,RISING);
  isLow=(PINB&_BV(PB3))==0;
  if(isLow){lowStartUs=micros();lastLowSeenUs=micros();}
  PCMSK0|=_BV(PCINT3);PCICR|=_BV(PCIE0);
  enteredMs=millis();lastGyroUs=micros();
  Serial.println(F("MQ SENTINEL V14 | DRIVE/BRAKE PWM | YAW-DRIFT FIXES | MICRO-PULSE RETURN"));
  Serial.println(F("MOTOR: 00=COAST 10=FORWARD 01=REVERSE 11=BRAKE (PMODE HIGH)"));
  Serial.println(F("DRIVE/BRAKE PWM: held-HIGH pin = direction, other pin inverted PWM; command 0 = BRAKE"));
  Serial.println(F("BRAKE NEEDS FRESH SIGNED ENCODER; 0 RPM MEANS BRAKE INHIBITED"));
  Serial.println(F("CAL LASER TEST: D4 HIGH for 350ms, then LOW; eye-safe module only"));
  Serial.println(F("STOP codes 1=I2C 2=RPM 3=GYRO 4=FAST_BRAKE_TIMEOUT 5=BACKOUT_TIMEOUT 6=SETTLE_TIMEOUT 7=SCAN_OVERSPEED 8=ARC_BRAKE_TIMEOUT 9=ENCODER_MISSING 10=AXIS_MISMATCH"));
  Serial.println(F("D5/D6 MOTOR | D4 LASER | D3/D2 ENCODER | D11 IR | A4/A5 GYRO"));
  Serial.print(F("LOCK_TOL_MAX="));Serial.print(FIRE_ANGLE_TOL_DEG,2);
  Serial.print(F(",RATE_MAX="));Serial.print(FIRE_RATE_TOL_DPS,2);
  Serial.print(F(",CONFIRM_MS="));Serial.print(FIRE_CONFIRM_MS);
  Serial.print(F(",IR_AGE_MS="));Serial.print(IR_RECENT_MS);
  Serial.print(F(",STUCK_LOW_MS="));Serial.println(IR_MAX_CONTINUOUS_LOW_MS);
  Serial.print(F("SEEK_FULL_PWM="));Serial.print(SEEK_PWM,0);
  Serial.print(F(",SCAN_BODY_RATE_DPS="));Serial.print(SCAN_BODY_DPS,1);
  Serial.print(F(",MAX_PWM="));Serial.print(MOTOR_PWM_LIMIT,0);
  Serial.print(F(",YAW_AXIS="));Serial.print(YAW_AXIS==0?'X':(YAW_AXIS==1?'Y':'Z'));
  Serial.print(F(",LASER_ENABLED="));Serial.println(LASER_ENABLED);
  Serial.print(F("ENCODER_WATCH_MS="));Serial.print(ENCODER_MISSING_TIMEOUT_MS);
  Serial.println(F(",STOP_CODE_9_ON_MISSING_EDGES"));
}
void loop(){
  const uint32_t now=millis();
  // Continue polling sensors after STOP so IR can expire and gyro yaw/rate
  // can keep changing. STOP still enforces motor=0 and D4 LOW. Do not enter
  // the state machine again until reset: safety stops are latched.
  gyroUpdate(now);
  serviceLaserSelfTest(millis());
  irUpdate(now);
  rpmUpdate(now);
  if(state!=STOP)sequence(now);
  motorUpdate(now);
  digitalWrite(PIN_LED,state==STOP?((now/125)&1):laserHigh);
  telemetry(now);
}
