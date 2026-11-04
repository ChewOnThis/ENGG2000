/* MQ SENTINEL V40 - early single-shot throughput + SEEK wind-up recovery.
   Based directly on the user's supplied V39. No motor gains/PWM speeds changed.

   After valid IR arc: EARLY firing can start as soon as the target comes into
   the measured angular region, normally around 0.4-1.2 seconds into AIM.
   By 2.65s, try a bounded fallback using the scanned midpoint even if IR has
   temporarily gone stale. The fallback is NOT allowed through gross pointing,
   gyro or wheel-speed faults. If aiming is invalid, it may skip the shot.

   One continuous nominal 2.5s laser command per target; never strobe or rearm.
   Major pointing/sensor faults may still interrupt a shot for safety. Timer2
   independently caps D4 at approximately 3.0s while interrupts are functioning.
   Seek reverses direction after sustained slow body yaw / apparent wind-up.

   Auto-run on RESET. D=dump+stop, S=stop, C=erase EEPROM history+stop.
   EEPROM V40 uses V39's same 45 x 22-byte acquisition records and CRC;
   V39/V38 records migrate without erasure. Header 5..6 add a persistent
   cumulative counter of detected SEEK slowdown reversals.
   Demonstration eye-safe laser only. Test first with D4 disconnected.
*/


#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

const uint8_t PIN_IN1=5, PIN_IN2=6, PIN_LASER=4;
const uint8_t PIN_ENC_A=3, PIN_ENC_B=2, PIN_IR=11, PIN_LED=13, MPU=0x68;
#define LIVE_TELEMETRY 0  // set 1 for 50 ms USB CSV; default low-overhead EEPROM run
const int8_t BODY_SIGN=-1, ENCODER_SIGN=-1, SEEK_MOTOR_SIGN=-1;
const uint8_t YAW_AXIS=0;
const float GYRO_X_BIAS=24.3267f;
const uint16_t GYRO_INTERVAL_MS=5, MOTOR_PERIOD_MS=5, RPM_SAMPLE_MS=50;
const float GYRO_COUNTS_PER_DPS=131.0f, ACC_COUNTS_PER_G=16384.0f;
const float RATE_LPF=0.16f, WHEEL_HARD_RPM=250.0f, COUNTS_PER_REV=700.0f;
const float MOTOR_PWM_LIMIT=255.0f;
const uint32_t BOOT_WAIT_MS=500UL;
const uint32_t IR_MIN_PULSE_US=100UL, IR_MAX_PULSE_US=35000UL;
const uint8_t IR_PULSES_REQUIRED=2;
const uint16_t IR_GROUP_MS=180, IR_LOSS_MS=210, IR_RECENT_MS=260;
const uint16_t IR_MAX_CONTINUOUS_LOW_MS=9000, IR_LOCK_LOW_MIN_MS=7;
const uint16_t IR_LOW_STEADY_MS=9, IR_CLEAR_MS=220, IR_POLL_MS=5;
const uint16_t ARC_MIN_MS=65, ARC_MAX_MS=8500;
const float ARC_MIN_DEG=0.5f, ARC_MAX_DEG=85.0f;
const float ARC_MIN_EFFICIENCY=0.48f, ARC_MAX_BACKTRACK_DEG=2.5f;
const float ARC_MAX_SWEEP_RATE=85.0f;

// V34 movement controls (adjust only after comparing EEPROM phase times).
const float SCAN_ENTRY_DPS=22.0f, SCAN_MIN_DPS=12.0f;
const float SCAN_SLOW_PER_DEG=1.2f, SCAN_RATE_PWM_LIMIT=95.0f;
const float AIM_MAX_DPS=12.0f, AIM_BRAKE_DECEL_DPS2=17.0f;
const float AIM_BRAKE_MARGIN_DEG=0.65f, AIM_RATE_PWM_LIMIT=120.0f;
const float FINE_ENTER_DEG=3.2f, FINE_REENTER_DEG=4.4f, FINE_MIN_ERROR=0.18f;
const float FINE_LOOKAHEAD_SEC=0.15f, FINE_GAIN_DEG_PER_DUTY_MS=0.122f;
const uint16_t FINE_GAP_MS=80, FINE_MIN_PULSE_MS=10, FINE_MAX_PULSE_MS=50;
const float FINE_RATE_ABORT_DPS=7.0f;
const float FIRE_PWM_SLEW_PER_SEC=850.0f;
const float HOLD_KI=0.8f, HOLD_BIAS_LIMIT=20.0f;
const float FIRE_ANGLE_TOL_DEG=2.20f, FIRE_RATE_TOL_DPS=3.20f;
const float FIRE_ANGLE_MIN_TOL_DEG=0.85f;
const float FIRE_HOLD_EXTRA_DEG=0.55f, FIRE_HOLD_RATE_DPS=4.3f;
const uint16_t FIRE_CONFIRM_MS=85, FIRE_IR_HOLD_MS=90, HOLD_RATE_CONFIRM_MS=40;
// Throughput caps are NOT changes to motor tuning/gains.
const uint16_t MAX_AIM_FINE_MS=3000;       // finish an attempt by 3s
const uint16_t EARLY_FIRE_AT_MS=350;       // allow early fire after settling begins
const uint16_t EARLY_FIRE_CONFIRM_MS=45;   // short evidence window, not zero
const uint16_t EXPECTED_FIRE_AT_MS=1050;   // permit broader entry after ~1s
const uint16_t FORCED_FIRE_AT_MS=2650;     // last-chance shot, within guard rails
const float EARLY_FIRE_EXTRA_DEG=0.90f;    // normal entry: arc-derived tolerance + margin
const float FORCED_FIRE_MAX_ERROR_DEG=5.8f;
const float FORCED_FIRE_MAX_BODY_RATE=40.0f;
const float FORCED_FIRE_MAX_WHEEL_RPM=230.0f;
const uint16_t FIRE_MIN_MS=2500, MAX_FIRE_EPISODE_MS=3000;
const uint16_t FIRE_QUALITY_CONFIRM_MS=150;  // legacy, no longer delays completion
const uint16_t FIRE_IR_GRACE_MS=900;          // temporary sensor gaps during shot
const uint16_t FIRE_BAD_ANGLE_GRACE_MS=150;
const float FIRE_ABORT_EXTRA_DEG=4.0f;       // severe pointing departure only
const float FIRE_ABORT_RATE_DPS=70.0f;       // severe rotation, not 10 deg/s wobble
const float FIRE_QUALITY_RATE_DPS=4.3f;
const uint16_t LASER_WATCHDOG_TICKS=188;     // Timer2 ~62.5Hz, ~3.008s
// SEEK: detect wind-up as yaw-rate collapse relative to the current sweep.
const uint16_t SEEK_SLOW_ARM_MS=650, SEEK_SLOW_CONFIRM_MS=375;
const float SEEK_SLOW_ABS_DPS=2.8f, SEEK_SLOW_RELATIVE=0.33f;
const float SEEK_BASELINE_MIN_DPS=9.0f;
const uint16_t NO_IR_AT_CENTRE_RESCAN_MS=3500;
const float MAX_FIRE_WHEEL_RPM=185.0f, HOLD_RATE_HARD_DPS=8.0f;
const float POINTING_RATE_TAU_SEC=0.150f;
const float SEARCH_SPEED_ENTER_DPS=60.0f, SEARCH_SPEED_EXIT_DPS=45.0f;
const float SEARCH_SPEED_TARGET_DPS=40.0f, SEARCH_LIMIT_SLEW=500.0f;
const float DEPARTURE_PWM_PER_SEC=250.0f, DEPARTURE_ACCEL_DPS2=15.0f;
const uint16_t WRONG_SEARCH_CONFIRM_MS=400, STARTUP_PROGRESS_WINDOW_MS=5000;
const float WRONG_SEARCH_DPS=2.0f, STARTUP_MIN_PROGRESS_DEG=4.0f;

// State values kept for compatibility with EEPROM V33 dump state IDs.
enum State:uint8_t {BOOT=0, CLEAR=2, SEEK=3, SLOW_SCAN=7,
                    AIM=9, FINE=10, FIRE=11, UNWIND=12, STOP=13};
State state=BOOT;
uint32_t enteredMs=0, clearSince=0, lastGyroMs=0, lastGyroUs=0;
#if LIVE_TELEMETRY
uint32_t lastPrintMs=0;
#endif
uint32_t lastMotorMs=0, stableSince=0, noIrAt=0, lastLockIrAt=0;
uint32_t fireStartedAt=0, rateViolationAt=0;
uint32_t trackingProgressAt=0, headroomAt=0, seekStallAt=0;
uint32_t wrongSearchAt=0, startupProgressAt=0, searchSpeedSafeAt=0;
uint32_t departureStillAt=0, holdBiasAt=0;
float startupProgressYaw=0, bestTrackingError=0;
int8_t seekDirection=SEEK_MOTOR_SIGN;
float seekLevel=100.0f, holdBias=0;
uint8_t seekRecoveryAttempts=0, scanFailCount=0;
bool controlledDeparture=false, searchDirectionWatchArmed=false;
float departureApplied=0, departureReference=0;
bool searchSpeedActive=false, searchSpeedRelease=false;
uint8_t gyroFaultMask=0,scanRejectReason=0;
uint16_t gyroFaultDtMs=0;
float gyroFaultRawDps=0,gyroFaultAccelG=0;
uint8_t stopCode=0, failedI2C=0;
float pointingRate=0;
bool pointingRateReady=false;
float gyroBias=0,yaw=0,yawRate=0,yawRateFiltered=0,rawRate=0,accelG=1;
bool calibrated=false,irStuckReported=false;
volatile bool laserHigh=false,laserWatchdogTripped=false;
volatile uint16_t laserWatchdogRemaining=0;
bool shotAttempted=false;
uint32_t targetAimStartedAt=0,firstFireAt=0,fireQualitySince=0;
uint32_t badFireAngleSince=0;
uint32_t seekSlowSince=0;
float seekPeakRate=0;
uint16_t seekStallReversals=0;
uint8_t cycleAbortReason=0;
float wheelRPM=0,rpmInstant=0;
int32_t prevTicks=0;
uint32_t lastRPMMs=0;
volatile int32_t encoderTicks=0;
int16_t pwmApplied=0;
float arcEntry=0,arcExit=0,arcCentre=0,arcWidth=0;
float arcMin=0,arcMax=0,arcPath=0,lastArcPathYaw=0,arcMaxRate=0;
uint32_t arcStarted=0;
bool arcValid=false,slowArcStarted=false;
int16_t finePulsePwm=0;
bool finePulseActive=false;
uint32_t finePulseAt=0,fineBrakeAt=0;
uint16_t finePulseDuration=0;
float finePulseDirection=0;
volatile bool isLow=false;
volatile uint32_t lowStartUs=0,lastLowSeenUs=0;
volatile uint16_t validPulseTotal=0;
ISR(PCINT0_vect) {
  const bool low=(PINB & _BV(PB3))==0;
  if(low==isLow)return;
  const uint32_t t=micros();
  isLow=low;
  if(low){lowStartUs=t;lastLowSeenUs=t;}
  else {const uint32_t width=t-lowStartUs;
    if(width>=IR_MIN_PULSE_US && width<=IR_MAX_PULSE_US)++validPulseTotal;}
}


struct IrState {
  bool low=false, active=false;
  uint16_t previousPulses=0, groupPulses=0;
  uint32_t groupStart=0, lastSignal=0, lastSample=0;
  float firstYaw=0, lastYaw=0;
} ir;

float absf(float x){return x<0?-x:x;}
float clampf(float x,float low,float high){return x<low?low:(x>high?high:x);}
float signf(float x){return x>0?1.0f:(x<0?-1.0f:0.0f);}

bool timeElapsed(uint32_t now,uint32_t since,uint32_t waitMs){
  return (int32_t)(now-since)>=0 && (uint32_t)(now-since)>=waitMs;
}
bool centred(uint32_t now);
void initialiseArc(float entry,uint32_t ms);
void controlledSearchRecovery(bool reverseDirection);
void restartSearch(bool reverseDirection);
void recoverStall();

// Legacy event calls become no-ops; the recorder below stores useful
// per-detection acquisition summaries instead of flooding EEPROM.
enum : uint8_t {EV_STATE=1,EV_ARC=2,EV_PULSE=3,EV_LASER_ON=5,
  EV_LASER_OFF=6,EV_FAULT=9,EV_LOCK_LOST=12,EV_COMPLETE=14,
  EV_RECOVER=15,EV_SEARCH_WRONG=16,EV_GYRO_DETAIL=17,
  EV_SCAN_REJECT=18,EV_SPEED_ON=19,EV_SPEED_OFF=20};
inline void recordEvent(uint8_t) {}

// Latest 45 acquisition records: 16-byte header + 45 x 22 bytes = 1006.
// Format: detected_ms, gap_10ms, scan_ms, aim_ms, fire_ms,
//         arc_width_cdeg, fire_error_cdeg, peak_wheel_rpm, outcome,
//         session_id, CRC8. The first gap of a session is from boot.
const uint8_t EEPROM_MAGIC=0xD8,EEPROM_VERSION=40, LOG_SLOTS=45;
enum Outcome:uint8_t {HIT_2500MS=1,SHOT_INTERRUPTED=2,AIM_TIMEOUT=3,
  SCAN_REJECT=4,TRACK_RECOVERY=5,SENSOR_STOP=6,USER_STOP=7,
  SCAN_TIMEOUT=8,FIRE_QUALITY_TIMEOUT=9};
// Abort causes packed in top nibble of outcome byte: 1=IR, 2=angle,
// 3=body rate, 4=wheel overspeed, 5=timer/watchdog, 6=sensor failure.
// Result itself remains the low nibble (1..9). No extra EEPROM bytes.
uint8_t logSequence=0, logSession=0,logCount=0;
uint8_t logQueue[4][22];
uint8_t logHead=0,logTail=0,logQueued=0,logByte=0;
uint16_t logDropped=0;
uint32_t logBootAt=0,lastDetectionAt=0,lastSuccessfulAt=0,cycleDetectionAt=0;
uint32_t cycleAimAt=0,cycleFirstFireAt=0;
uint16_t cycleGap10=0,cycleScanMs=0,cycleAimMs=0,cycleFireMs=0;
uint16_t cycleArcWidthCdeg=0,cycleScoreGap10=0;
int16_t cycleFireErrorCdeg=0;
uint8_t cyclePeakRPM=0;
bool cycleActive=false;

void recorderInit();
void recorderService();
void recorderCommands();
void beginAcquisition(uint32_t now);
void completeAcquisition(uint8_t outcome,uint32_t now);

// Timer2 CTC at ~62.5 Hz (16 MHz AVR, /1024, OCR2A=249).
// Independent of the main control loop; D4 is PD4 on the ATmega328P.
// Only reliable while MCU/interrupts are working: this is not a hardware kill switch.
ISR(TIMER2_COMPA_vect) {
  if(laserWatchdogRemaining>0){
    if(--laserWatchdogRemaining==0){
      PORTD &= (uint8_t)~_BV(PD4);
      laserHigh=false;
      laserWatchdogTripped=true;
    }
  }
}
void initLaserWatchdog(){
  TCCR2A=_BV(WGM21);
  TCCR2B=_BV(CS22)|_BV(CS21)|_BV(CS20);
  OCR2A=249;
  TCNT2=0;
  TIMSK2|=_BV(OCIE2A);
}

void laserOff(){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    PORTD &= (uint8_t)~_BV(PD4);
    laserHigh=false;
    laserWatchdogRemaining=0;
  }
}
void laserOn(){
  // A target gets precisely one laser activation. Never reset the 3 s
  // deadline by trying to fire repeatedly on the same acquisition.
  if(shotAttempted || !cycleActive || state==STOP)return;
  const uint32_t now=millis();
  shotAttempted=true;
  firstFireAt=now;
  fireQualitySince=0;
  badFireAngleSince=0;
  cycleAbortReason=0;
  cycleFirstFireAt=now;
  cycleAimMs=cycleAimAt?(uint16_t)min((uint32_t)65535,now-cycleAimAt):0;
  cycleFireErrorCdeg=(int16_t)clampf((arcCentre-yaw)*100.0f,-32768,32767);
  laserWatchdogTripped=false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    laserWatchdogRemaining=LASER_WATCHDOG_TICKS;
    PORTD|=_BV(PD4);
    laserHigh=true;
  }
}

void motorHardware(int16_t p){
  if(p>255)p=255;else if(p<-255)p=-255;
  if(p>0){analogWrite(PIN_IN1,255);analogWrite(PIN_IN2,255-p);}
  else if(p<0){analogWrite(PIN_IN2,255);analogWrite(PIN_IN1,255+p);}
  else{analogWrite(PIN_IN1,255);analogWrite(PIN_IN2,255);}
}
void motorOff(){pwmApplied=0;motorHardware(0);}

void enter(State s){
  state=s;enteredMs=millis();rateViolationAt=0;
  if(s!=CLEAR && s!=SEEK && s!=SLOW_SCAN)wrongSearchAt=0;
  if(s==FIRE){fireStartedAt=enteredMs;finePulseActive=false;}
  if(s==SEEK){seekStallAt=0;seekLevel=100;
    seekSlowSince=0;seekPeakRate=0;
    startupProgressAt=enteredMs;startupProgressYaw=yaw;}
  if(s!=AIM && s!=FINE && s!=FIRE){trackingProgressAt=0;headroomAt=0;holdBias=0;}
  holdBiasAt=enteredMs;
  finePulseActive=false;fineBrakeAt=enteredMs;
  if(s!=AIM && s!=FINE)stableSince=0;
  recordEvent(EV_STATE);
}
void safeStop(uint8_t c){
  if(state==STOP)return;
  stopCode=c;laserOff();motorOff();
  completeAcquisition(SENSOR_STOP,millis());
  enter(STOP);recordEvent(EV_FAULT);
}
void beginClear(){laserOff();arcValid=false;slowArcStarted=false;clearSince=0;enter(CLEAR);}

void encoderRise(){encoderTicks+=digitalRead(PIN_ENC_B)?-1:+1;}

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
  dps=v[4+YAW_AXIS]/GYRO_COUNTS_PER_DPS;   
  return true;
}
void startWithFixedBias(){
  motorOff();laserOff();
  gyroBias=GYRO_X_BIAS/GYRO_COUNTS_PER_DPS;
  calibrated=true;
  yaw=yawRate=yawRateFiltered=0;pointingRate=0;pointingRateReady=false;
  lastGyroUs=micros();lastGyroMs=millis();
  beginClear();
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
  if(calibrated){
    const float dt=(uint32_t)(us-lastGyroUs)*0.000001f;
    if(dt>0.12f||absf(r)>=235||g>1.75f||g<0.36f){
      if(state!=STOP){
        gyroFaultMask=(dt>0.12f?1:0)|(absf(r)>=235?2:0)|
          (g>1.75f?4:0)|(g<0.36f?8:0);
        gyroFaultDtMs=(uint16_t)clampf(dt*1000,0,32767);
        gyroFaultRawDps=r;gyroFaultAccelG=g;
        recordEvent(EV_GYRO_DETAIL);safeStop(3);
      }
    }
    else {
      yawRate=r-gyroBias;
      if(absf(yawRate)<0.07f && absf(wheelRPM)<1.0f && pwmApplied==0)yawRate=0;
      yaw+=yawRate*dt;
      yawRateFiltered+=RATE_LPF*(yawRate-yawRateFiltered);
      if(!pointingRateReady){pointingRate=yawRate;pointingRateReady=true;}
      else pointingRate+=(dt/(POINTING_RATE_TAU_SEC+dt))*(yawRate-pointingRate);

    }
  }
  lastGyroUs=us;
}

bool irRecent(uint32_t now){
  if(!ir.active)return false;
  uint32_t started, lastEdge;
  bool low;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    started=lowStartUs;lastEdge=lastLowSeenUs;low=isLow;
  }
  const uint32_t us=micros();
  const uint32_t lowMs=(uint32_t)(us-started)/1000UL;

  const bool validatedTarget=arcValid &&
      (state==AIM||state==FINE||state==FIRE);
  const bool validLevel=low && lowMs>=IR_LOCK_LOW_MIN_MS &&
      (lowMs<=IR_MAX_CONTINUOUS_LOW_MS || validatedTarget);
  
  const bool pulseFresh=!low &&
      (uint32_t)(us-lastEdge)<=IR_RECENT_MS*1000UL &&
      (uint32_t)(now-ir.lastSignal)<=IR_RECENT_MS;
  return validLevel || pulseFresh;
}
void firstIrEncounter(float entry){
  beginAcquisition(millis());
  laserOff();enter(SLOW_SCAN);
  initialiseArc(entry,ir.groupStart?ir.groupStart:millis());
}

void initialiseArc(float entry,uint32_t ms){
  arcEntry=entry;arcStarted=ms;
  arcExit=entry;arcMin=arcMax=entry;arcPath=arcMaxRate=0;
  lastArcPathYaw=entry;
  slowArcStarted=true;
}
// Scan rejects generally mean keep moving, not brake, clear and reverse.
// Three repeated failures prompt a genuine controlled direction recovery.
void rescanOrSeek(){
  completeAcquisition(SCAN_TIMEOUT,millis());
  laserOff();arcValid=false;slowArcStarted=false;noIrAt=0;stableSince=0;
  if(++scanFailCount>=3){scanFailCount=0;controlledSearchRecovery(true);}
  else enter(SEEK);
}
void restartSearch(bool reverseDirection){
  completeAcquisition(TRACK_RECOVERY,millis());
  controlledDeparture=false;
  laserOff();recordEvent(EV_RECOVER);
  arcValid=false;slowArcStarted=false;stableSince=0;noIrAt=0;
  if(reverseDirection)seekDirection=-seekDirection;
  seekLevel=100;seekStallAt=0;clearSince=0;
  enter(UNWIND);
}
void controlledSearchRecovery(bool reverseDirection){
  const float previousCommand=pwmApplied;
  restartSearch(reverseDirection);
  controlledDeparture=true;
  departureApplied=previousCommand;
  departureReference=yawRateFiltered;
  departureStillAt=0;
}
void recoverStall(){
  // A stalled body first retries without reversing; persistent stalls reverse.
  if(seekRecoveryAttempts<2)++seekRecoveryAttempts;
  controlledSearchRecovery(seekRecoveryAttempts>=2);
}
void completedTargetDeparture(){
  seekRecoveryAttempts=0;scanFailCount=0;
  searchDirectionWatchArmed=true;
  const float previousCommand=pwmApplied;
  laserOff();arcValid=false;slowArcStarted=false;
  stableSince=0;noIrAt=0;clearSince=0;
  enter(CLEAR);
  controlledDeparture=true;
  departureApplied=previousCommand;
  departureReference=yawRateFiltered;
  departureStillAt=0;
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
    scanRejectReason=bad;
    cycleScanMs=(uint16_t)min((uint32_t)65535,ms-cycleDetectionAt);
    cycleArcWidthCdeg=(uint16_t)clampf(arcWidth*100.0f,0,65535);
    completeAcquisition(SCAN_REJECT,millis());
    recordEvent(EV_SCAN_REJECT);rescanOrSeek();
  }
  else{
    scanFailCount=0;
    arcCentre=(arcEntry+arcExit)*0.5f;arcValid=true;recordEvent(EV_ARC);
    cycleScanMs=(uint16_t)min((uint32_t)65535,ms-cycleDetectionAt);
    cycleArcWidthCdeg=(uint16_t)clampf(arcWidth*100.0f,0,65535);
    cycleAimAt=millis();targetAimStartedAt=cycleAimAt;shotAttempted=false;
    // No fixed 350 ms idle/brake state: AIM brakes immediately using gyro rate.
    enter(AIM);
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

  // Only measure the path while actual signal evidence is being received.
  // During the 210 ms loss-confirmation gap, leave these statistics frozen.
  if(state==SLOW_SCAN && slowArcStarted && (newPulses || steadyLow)){
    arcMin=fminf(arcMin,yaw);arcMax=fmaxf(arcMax,yaw);
    arcMaxRate=fmaxf(arcMaxRate,absf(yawRate));
    arcPath+=absf(yaw-lastArcPathYaw);lastArcPathYaw=yaw;
  }
  if(stuckLow && ir.active){
    ir.active=false;ir.groupPulses=0;ir.groupStart=0;
    if(state==SLOW_SCAN && slowArcStarted)rescanOrSeek();
  }
  if(!ir.active){
    if(!stuckLow && ((ir.groupPulses>=IR_PULSES_REQUIRED &&
         now-ir.groupStart<=IR_GROUP_MS && ir.groupStart!=0)||steadyLow)){
      ir.active=true;
      if(state==SEEK)firstIrEncounter(ir.firstYaw);
      else if(state==SLOW_SCAN && !slowArcStarted)
        initialiseArc(ir.firstYaw,ir.groupStart);
    }
  }else if(!low && timeElapsed(now,ir.lastSignal,IR_LOSS_MS)){
    ir.active=false;
    ir.groupPulses=0;ir.groupStart=0;
    if(state==SLOW_SCAN && slowArcStarted)finishArc(ir.lastYaw,ir.lastSignal);
  }
  if(!ir.active&&ir.groupStart&&now-ir.groupStart>IR_GROUP_MS){
    ir.groupStart=0;ir.groupPulses=0;
  }
}

void rpmUpdate(uint32_t now){
  if(now-lastRPMMs<RPM_SAMPLE_MS)return;
  int32_t ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ticks=encoderTicks;}
  if(lastRPMMs){
    rpmInstant=(ticks-prevTicks)*60000.0f/(COUNTS_PER_REV*(now-lastRPMMs));
    wheelRPM+=0.35f*(rpmInstant-wheelRPM);
    if(absf(wheelRPM)>WHEEL_HARD_RPM || absf(rpmInstant)>WHEEL_HARD_RPM+8.0f)safeStop(2);
  }
  lastRPMMs=now;prevTicks=ticks;
  if(cycleActive){
    const uint8_t r=(uint8_t)clampf(absf(wheelRPM),0,255);
    if(r>cyclePeakRPM)cyclePeakRPM=r;
  }
}

float wheelFeedforward(){return ENCODER_SIGN*wheelRPM*255.0f/157.0f;}
float ratePWM(float targetDps,float maxCorrection){
  const float correction=clampf(4.0f*(targetDps-yawRateFiltered),-maxCorrection,maxCorrection);
  return clampf(wheelFeedforward()+BODY_SIGN*correction,-MOTOR_PWM_LIMIT,MOTOR_PWM_LIMIT);
}
float cruisePWM(){return seekDirection*seekLevel;}
float scanPWM(){
  // Control actual yaw rate; decelerate progressively as the IR arc develops.
  const float span=absf(yaw-arcEntry);
  const float speed=clampf(SCAN_ENTRY_DPS-SCAN_SLOW_PER_DEG*span,
                            SCAN_MIN_DPS,SCAN_ENTRY_DPS);
  return ratePWM(BODY_SIGN*seekDirection*speed,SCAN_RATE_PWM_LIMIT);
}
float coarsePWM(){
  const float err=arcCentre-yaw,dist=absf(err);
  // Speed permitted by remaining stopping distance, not a fixed 3 deg/s cap.
  const float stopSpeed=sqrtf(2.0f*AIM_BRAKE_DECEL_DPS2*
    fmaxf(0.0f,dist-AIM_BRAKE_MARGIN_DEG));
  const float target=signf(err)*fminf(AIM_MAX_DPS,stopSpeed);
  return ratePWM(target,AIM_RATE_PWM_LIMIT);
}
float fineHoldPWM(){
  const float requested=clampf(0.6f*(arcCentre-yaw),-1.2f,1.2f);
  const float correction=clampf(4.0f*(requested-yawRateFiltered)+holdBias,-65,65);
  return clampf(wheelFeedforward()+BODY_SIGN*correction,-255,255);
}
void updateHoldBias(uint32_t now){
  const float dt=clampf((uint32_t)(now-holdBiasAt)*0.001f,0,0.02f);
  holdBiasAt=now;
  const float err=arcCentre-yaw;
  if(finePulseActive || !arcValid || absf(err)>FINE_REENTER_DEG ||
       absf(yawRateFiltered)>8.0f)return;
  const float candidate=clampf(holdBias+HOLD_KI*err*dt,-HOLD_BIAS_LIMIT,HOLD_BIAS_LIMIT);
  const float delta=candidate-holdBias;
  const float requested=clampf(0.6f*err,-1.2f,1.2f);
  const float inner=4.0f*(requested-yawRateFiltered)+candidate;
  const float raw=wheelFeedforward()+BODY_SIGN*clampf(inner,-65,65);
  if((absf(inner)<=65 || delta*inner<0) &&
     (absf(raw)<=255 || BODY_SIGN*delta*raw<0))holdBias=candidate;
}
float finePWM(uint32_t now){
  updateHoldBias(now);
  const float err=arcCentre-yaw,rate=yawRateFiltered;
  // Project body position forward to avoid correcting after a likely overshoot.
  const float predicted=err-rate*FINE_LOOKAHEAD_SEC;
  if(finePulseActive){
    if(timeElapsed(now,finePulseAt,finePulseDuration) ||
      predicted*finePulseDirection<=0 ||
      (absf(err)<FINE_MIN_ERROR && absf(rate)<1.0f) ||
      absf(rate)>=FINE_RATE_ABORT_DPS){
      finePulseActive=false;fineBrakeAt=now;
      return fineHoldPWM();
    }
    return clampf(wheelFeedforward()+BODY_SIGN*(finePulseDirection*finePulsePwm+holdBias),-255,255);
  }
  if(centred(now) || absf(predicted)<FINE_MIN_ERROR ||
      !timeElapsed(now,fineBrakeAt,FINE_GAP_MS) ||
      absf(rate)>=FINE_RATE_ABORT_DPS)return fineHoldPWM();
  const float e=absf(predicted);
  finePulsePwm=e<1.0f?55:(e<2.5f?80:105);
  const float duty=finePulsePwm/255.0f;
  const float width=0.40f*e/(FINE_GAIN_DEG_PER_DUTY_MS*duty);
  finePulseDuration=(uint16_t)clampf(width,FINE_MIN_PULSE_MS,FINE_MAX_PULSE_MS);
  finePulseDuration=((finePulseDuration+MOTOR_PERIOD_MS-1)/MOTOR_PERIOD_MS)*MOTOR_PERIOD_MS;
  finePulseAt=now;finePulseDirection=signf(predicted);finePulseActive=true;
  recordEvent(EV_PULSE);
  return clampf(wheelFeedforward()+BODY_SIGN*(finePulseDirection*finePulsePwm+holdBias),-255,255);
}

float limitSearchPWM(uint32_t now,float normalCommand,float dt){
  const bool searching=state==CLEAR||state==SEEK||state==SLOW_SCAN;
  if(!searching){
    if(searchSpeedActive)recordEvent(EV_SPEED_OFF);
    searchSpeedActive=searchSpeedRelease=false;searchSpeedSafeAt=0;
    return normalCommand;
  }
  // Raw corrected gyro catches an accelerating flyby before LPF catches up.
  const float observed=fmaxf(absf(yawRate),absf(yawRateFiltered));
  if(!searchSpeedActive && observed>=SEARCH_SPEED_ENTER_DPS){
    searchSpeedActive=true;searchSpeedRelease=false;searchSpeedSafeAt=0;
    recordEvent(EV_SPEED_ON);
  }
  if(searchSpeedActive){
    if(observed<=SEARCH_SPEED_EXIT_DPS){
      if(!searchSpeedSafeAt)searchSpeedSafeAt=now;
      if(timeElapsed(now,searchSpeedSafeAt,100)){
        searchSpeedActive=false;searchSpeedRelease=true;
        recordEvent(EV_SPEED_OFF);
      }
    }else searchSpeedSafeAt=0;
  }
  float wanted=normalCommand;
  if(searchSpeedActive){
    const float target=signf(yawRate)*SEARCH_SPEED_TARGET_DPS;
    const float correction=clampf(4.0f*(target-yawRate),-90,90);
    wanted=clampf(wheelFeedforward()+BODY_SIGN*correction,-255,255);
  }
  if(searchSpeedActive||searchSpeedRelease){
    const float out=pwmApplied+clampf(wanted-pwmApplied,
      -SEARCH_LIMIT_SLEW*dt,SEARCH_LIMIT_SLEW*dt);
    if(searchSpeedRelease && absf(out-normalCommand)<1.0f)searchSpeedRelease=false;
    return out;
  }
  return normalCommand;
}
void motorUpdate(uint32_t now){
  if(now-lastMotorMs<MOTOR_PERIOD_MS)return;
  const float dt=clampf((uint32_t)(now-lastMotorMs)*0.001f,0,0.02f);
  lastMotorMs=now;
  float command=0;
  if(state==CLEAR||state==SEEK)command=cruisePWM();
  else if(state==SLOW_SCAN)command=scanPWM();
  else if(state==AIM)command=coarsePWM();
  else if(state==FINE)command=finePWM(now);
  else if(state==FIRE){
    // Continuous rate/position holding while laser is on: no disruptive pulses.
    updateHoldBias(now);
    const float wanted=fineHoldPWM();
    // Immediately cancel any active correction pulse at laser entry.
    command=(uint32_t)(now-fireStartedAt)<=2*MOTOR_PERIOD_MS ? wanted :
       pwmApplied+clampf(wanted-pwmApplied,
         -FIRE_PWM_SLEW_PER_SEC*dt,FIRE_PWM_SLEW_PER_SEC*dt);
  }
  if(controlledDeparture && state==UNWIND){
    departureReference+=clampf(-departureReference,
      -DEPARTURE_ACCEL_DPS2*dt,DEPARTURE_ACCEL_DPS2*dt);
    command=ratePWM(departureReference,65.0f);
  }
  if(controlledDeparture && (state==UNWIND||state==CLEAR||state==SEEK||state==SLOW_SCAN)){
    departureApplied+=clampf(command-departureApplied,
      -DEPARTURE_PWM_PER_SEC*dt,DEPARTURE_PWM_PER_SEC*dt);
    command=departureApplied;
    if((state==SEEK||state==SLOW_SCAN) &&
        absf(departureApplied-(state==SLOW_SCAN?scanPWM():cruisePWM()))<1.0f)
      controlledDeparture=false;
  }else if(state!=UNWIND && state!=CLEAR && state!=SEEK && state!=SLOW_SCAN)
    controlledDeparture=false;
  command=limitSearchPWM(now,command,dt);
  if(state==STOP||state==BOOT||(state==UNWIND&&!controlledDeparture))command=0;
  pwmApplied=(int16_t)clampf(command,-MOTOR_PWM_LIMIT,MOTOR_PWM_LIMIT);
  motorHardware(pwmApplied);
}

float fireTolerance(){
  
  return clampf(arcWidth*0.42f,FIRE_ANGLE_MIN_TOL_DEG,FIRE_ANGLE_TOL_DEG);
}

bool centred(uint32_t now){
  return calibrated && arcValid && irRecent(now) &&
    absf(arcCentre-yaw)<=fireTolerance() &&
    absf(yawRateFiltered)<=FIRE_RATE_TOL_DPS &&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}

// In-flight tolerance is deliberately distinct from the tighter laser-ON gate.
// The old V38 maintainLock() aborted on a single small angle/rate excursion,
// explaining 18 shortened bursts. Minor deviations are corrected while ON.
// Severe deviations still force laser OFF immediately.
uint8_t fireAbortCause(uint32_t now){
  if(!calibrated || !arcValid || !laserHigh)return 6;
  // A brief body-rate spike is normal for a reaction wheel. V39 aborted all
  // 17 measured short attempts on its 10/12 deg/s limit. Only gross spin is a
  // reason to kill a shot; gyroUpdate() already checks sensor failure.
  if(absf(yawRateFiltered)>FIRE_ABORT_RATE_DPS ||
     absf(yawRate)>FIRE_ABORT_RATE_DPS+20.0f)return 3;
  if(absf(wheelRPM)>FORCED_FIRE_MAX_WHEEL_RPM)return 4;
  // Minor angular excursions should not blink the laser. Reject gross
  // departure that persists for 150 ms, rather than any single noisy sample.
  const float allowed=fmaxf(6.0f,fireTolerance()+FIRE_ABORT_EXTRA_DEG);
  if(absf(arcCentre-yaw)>allowed){
    if(!badFireAngleSince)badFireAngleSince=now;
    if(timeElapsed(now,badFireAngleSince,FIRE_BAD_ANGLE_GRACE_MS))return 2;
  }else badFireAngleSince=0;
  // IR is used to establish a real scanned target. Its temporary disappearance
  // during firing is not a reason to abort a still-centred gyro estimate.
  return 0;
}

// Only use the original V34-quality geometry / IR conditions to count a shot.
bool fireQualityOK(uint32_t now){
  return calibrated && arcValid && irRecent(now) &&
    absf(arcCentre-yaw)<=fireTolerance() &&
    absf(pointingRate)<=FIRE_RATE_TOL_DPS &&
    absf(yawRateFiltered)<=FIRE_QUALITY_RATE_DPS &&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}


bool fireCentred(uint32_t now){
  return calibrated && arcValid && pointingRateReady && irRecent(now) &&
    absf(arcCentre-yaw)<=fireTolerance() &&
    absf(pointingRate)<=FIRE_RATE_TOL_DPS &&
    absf(yawRateFiltered)<HOLD_RATE_HARD_DPS &&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}
bool updateLaserArming(uint32_t now){
  if(shotAttempted || !cycleActive || !calibrated || !arcValid ||
     !targetAimStartedAt || state==STOP){stableSince=0;return false;}
  const uint32_t age=(uint32_t)(now-targetAimStartedAt);
  if(age<EARLY_FIRE_AT_MS)return false;
  const float error=absf(arcCentre-yaw);
  const float rate=absf(yawRateFiltered);
  const float rpm=absf(wheelRPM);
  // Near target: use IR confirmation when available. Preserve some rate
  // margin: pointingRate is deliberately heavily smoothed and V39's
  // narrow rate gates were blocking most real-world opportunities.
  const bool irOK=irRecent(now);
  const bool early=irOK &&
      error<=fireTolerance()+EARLY_FIRE_EXTRA_DEG &&
      rate<=18.0f && rpm<=205.0f;
  const bool broad=age>=EXPECTED_FIRE_AT_MS && irOK &&
      error<=fireTolerance()+1.45f && rate<=27.0f && rpm<=215.0f;
  // At 2.65 s, use the midpoint of the independently validated scanned
  // arc as a last-chance estimate. Do not require a fresh IR edge; the
  // target may be briefly obscured while the reaction wheel corrects.
  // However, never force a shot when the gyro, angle or wheel is grossly unsafe.
  const bool fallback=age>=FORCED_FIRE_AT_MS &&
      error<=FORCED_FIRE_MAX_ERROR_DEG &&
      rate<=FORCED_FIRE_MAX_BODY_RATE &&
      absf(yawRate)<=FORCED_FIRE_MAX_BODY_RATE+15.0f &&
      rpm<=FORCED_FIRE_MAX_WHEEL_RPM &&
      !irStuckReported;
  if(early || broad){
    if(!stableSince)stableSince=now;
  }else stableSince=0;
  if((stableSince && timeElapsed(now,stableSince,EARLY_FIRE_CONFIRM_MS)) ||
     fallback){
    lastLockIrAt=now;
    laserOn();
    if(laserHigh){enter(FIRE);return true;}
  }
  return false;
}

bool trackingNeedsRecovery(uint32_t now){
  const float error=absf(arcCentre-yaw);
  // No IR is normal outside the cone while returning toward its centre.
  // Do not abandon a progressing AIM merely because it has not re-entered.
  if(!trackingProgressAt || error<bestTrackingError-0.5f){
    bestTrackingError=error;trackingProgressAt=now;
  }
  if(error>6.0f && timeElapsed(now,trackingProgressAt,4000))return true;
  if(error>fireTolerance() && abs(pwmApplied)>=245 && absf(wheelRPM)>145){
    if(!headroomAt)headroomAt=now;
    if(timeElapsed(now,headroomAt,700))return true;
  }else headroomAt=0;
  return false;
}
bool searchMovingBackward(uint32_t now){
  // Do not interpret the deliberately ramped handoff as failed search.
  const bool enabled=searchDirectionWatchArmed &&
    (state==CLEAR||state==SEEK||state==SLOW_SCAN) && !controlledDeparture &&
    !searchSpeedActive && !searchSpeedRelease &&
    pwmApplied!=0;
  const float progress=yawRateFiltered*BODY_SIGN*seekDirection;
  if(!enabled || progress>=-WRONG_SEARCH_DPS){wrongSearchAt=0;return false;}
  if(!wrongSearchAt)wrongSearchAt=now;
  return timeElapsed(now,wrongSearchAt,WRONG_SEARCH_CONFIRM_MS);
}
void sequence(uint32_t now){
  // One laser activation per detection. After 2500 ms, depart whether or not
  // the old precision-only quality rule would have counted the shot.
  if(state==FIRE){
    const uint32_t elapsed=(uint32_t)(now-firstFireAt);
    if(laserWatchdogTripped || elapsed>=MAX_FIRE_EPISODE_MS){
      cycleAbortReason=5;laserOff();
      completeAcquisition(FIRE_QUALITY_TIMEOUT,now);
      completedTargetDeparture();return;
    }
    const uint8_t cause=fireAbortCause(now);
    if(cause){
      cycleAbortReason=cause;laserOff();
      completeAcquisition(SHOT_INTERRUPTED,now);
      completedTargetDeparture();return;
    }
    if(elapsed>=FIRE_MIN_MS){
      laserOff();
      // Means the 2500ms laser *command* completed, NOT a verified hit.
      completeAcquisition(HIT_2500MS,now);
      completedTargetDeparture();return;
    }
    return;
  }

  // No five-second chase of a valid scanned arc. At 2.65s a bounded fallback
  // fires if possible; at 3s abandon any attempt that cannot safely fire.
  if((state==AIM || state==FINE) && targetAimStartedAt &&
      timeElapsed(now,targetAimStartedAt,MAX_AIM_FINE_MS)){
    completeAcquisition(AIM_TIMEOUT,now);
    completedTargetDeparture();
    return;
  }

  if(searchMovingBackward(now)){
    recordEvent(EV_SEARCH_WRONG);
    controlledSearchRecovery(true);return;
  }
  if((state==AIM||state==FINE||state==FIRE)&&trackingNeedsRecovery(now)){
    recoverStall();return;
  }
  switch(state){
    case BOOT:
      if(timeElapsed(now,enteredMs,BOOT_WAIT_MS))startWithFixedBias();
      break;
    case UNWIND:
      if(controlledDeparture){
        if(absf(yawRateFiltered)<2.0f){if(!departureStillAt)departureStillAt=now;}
        else departureStillAt=0;
        if((departureStillAt&&timeElapsed(now,departureStillAt,100))||
           timeElapsed(now,enteredMs,1500)){clearSince=0;enter(CLEAR);}
      }else if(timeElapsed(now,enteredMs,650)){clearSince=0;enter(CLEAR);}
      break;
    case CLEAR:
      if(timeElapsed(now,enteredMs,1800)){recoverStall();break;}
      if(!ir.active){
        if(!clearSince)clearSince=now;
        if(timeElapsed(now,clearSince,IR_CLEAR_MS))enter(SEEK);
      }else clearSince=0;
      break;
    case SEEK:{
      if(!searchDirectionWatchArmed && !controlledDeparture && pwmApplied!=0){
        if(timeElapsed(now,startupProgressAt,STARTUP_PROGRESS_WINDOW_MS)){
          const float progress=(yaw-startupProgressYaw)*BODY_SIGN*seekDirection;
          startupProgressAt=now;startupProgressYaw=yaw;
          if(progress<STARTUP_MIN_PROGRESS_DEG){
            recordEvent(EV_SEARCH_WRONG);recoverStall();break;
          }
        }
      }else{startupProgressAt=now;startupProgressYaw=yaw;}
      if(searchSpeedActive||searchSpeedRelease||controlledDeparture){
        seekStallAt=0;seekSlowSince=0;break;
      }
      // Use the direction-corrected yaw rate once the initial acceleration
      // has settled. This detects both wind-up (speed collapsing) and the
      // body starting to reverse despite a fixed motor command.
      const float progress=yawRateFiltered*BODY_SIGN*seekDirection;
      const float speed=fmaxf(0.0f,progress);
      if(speed>seekPeakRate)seekPeakRate=speed;
      const bool armed=timeElapsed(now,enteredMs,SEEK_SLOW_ARM_MS);
      const bool stalled=progress<SEEK_SLOW_ABS_DPS ||
        (seekPeakRate>=SEEK_BASELINE_MIN_DPS &&
         progress<SEEK_SLOW_RELATIVE*seekPeakRate);
      if(armed && stalled){
        if(!seekSlowSince)seekSlowSince=now;
        const uint16_t need=progress < -3.0f ? 220 : SEEK_SLOW_CONFIRM_MS;
        if(timeElapsed(now,seekSlowSince,need)){
          // This is a genuine seek-direction reversal, not V39's two
          // same-direction stall recoveries. Use the existing smooth
          // departure ramp and wheel/gyro speed guards.
          if(seekStallReversals<65535U){
            ++seekStallReversals;
            EEPROM.update(5,(uint8_t)seekStallReversals);
            EEPROM.update(6,(uint8_t)(seekStallReversals>>8));
          }
          controlledSearchRecovery(true);
          break;
        }
      }else seekSlowSince=0;
      // Keep the V39 gradual PWM recovery during initial low-speed motion.
      if(speed<1.5f){
        if(!seekStallAt)seekStallAt=now;
        if(timeElapsed(now,seekStallAt,700))
          seekLevel=100+50*clampf((now-seekStallAt-700UL)/1500.0f,0,1);
      }else{seekStallAt=0;seekLevel=100;}
      break;
    }
    case SLOW_SCAN:
      if(timeElapsed(now,enteredMs,ARC_MAX_MS+250UL))rescanOrSeek();
      break;
    case AIM:
      if(updateLaserArming(now))break;
      if(absf(arcCentre-yaw)<=FINE_ENTER_DEG&&absf(yawRateFiltered)<=3.0f)
        enter(FINE);
      break;
    case FINE:
      if(absf(arcCentre-yaw)>FINE_REENTER_DEG){enter(AIM);break;}
      if(updateLaserArming(now))break;
      if(absf(arcCentre-yaw)<fireTolerance() && !irRecent(now)){
        if(!noIrAt)noIrAt=now;
        if(timeElapsed(now,noIrAt,NO_IR_AT_CENTRE_RESCAN_MS)){
          noIrAt=0;rescanOrSeek();
        }
      }else noIrAt=0;
      break;
    case FIRE: break; // Always processed at the beginning of sequence().
    case STOP:laserOff();break;
    default:safeStop(4);break;
  }
}

#if LIVE_TELEMETRY
void telemetry(uint32_t now){
  if(now-lastPrintMs<50)return;
  lastPrintMs=now;
  static uint16_t sample=0;
  ++sample;
  char line[64];
  const uint8_t flags=(irRecent(now)?1:0)|(arcValid?2:0)|(laserHigh?4:0)|
    (arcValid && absf(arcCentre-yaw)<=fireTolerance()?8:0)|
    (absf(yawRateFiltered)<=FIRE_RATE_TOL_DPS?16:0)|
    (absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM?32:0)|(irStuckReported?64:0)|
    (finePulseActive?128:0);
  const int n=snprintf(line,sizeof(line),"%lu,%u,%u,%d,%d,%d,%d,%d,%u,%u\n",
    (unsigned long)now,sample,(unsigned int)state,
    (int)clampf(yaw*10.0f,-32768,32767),
    (int)clampf((arcValid?arcCentre-yaw:0)*100.0f,-32768,32767),
    (int)clampf(yawRateFiltered*100.0f,-32768,32767),
    (int)clampf(wheelRPM*10.0f,-32768,32767),(int)pwmApplied,
    (unsigned int)flags,(unsigned int)stopCode);
  if(n>0 && n<(int)sizeof(line) && Serial.availableForWrite()>=n)
    Serial.write((const uint8_t*)line,n);
}


#endif

// EEPROM record format (22 bytes):
//  0..3 detection ms from this reset; 4..5 gap to prior detection (10 ms)
//  6..7 scan ms; 8..9 AIM/FINE ms; 10..11 laser-on ms
//  12..13 IR arc width centidegrees; 14..15 signed firing error centidegrees
//  16..17 gap to previous completed shot (10 ms; zero on failure)
//  18 peak reaction-wheel RPM; 19 outcome; 20 session; 21 CRC8.
// Nonblocking byte writes; sequence header committed AFTER record bytes.
uint8_t logCrc(const uint8_t *b){
  uint8_t crc=0xA5;
  for(uint8_t i=0;i<21;++i){
    crc^=b[i];
    for(uint8_t k=0;k<8;++k)
      crc=(crc&0x80)?(uint8_t)((crc<<1)^0x07):(uint8_t)(crc<<1);
  }
  return crc;
}
void put16(uint8_t *b,uint8_t i,uint16_t v){b[i]=v;b[i+1]=v>>8;}
void put32(uint8_t *b,uint8_t i,uint32_t v){
  for(uint8_t j=0;j<4;++j)b[i+j]=(uint8_t)(v>>(j*8));
}
uint16_t get16(const uint8_t *b,uint8_t i){return (uint16_t)b[i]|((uint16_t)b[i+1]<<8);}
uint32_t get32(const uint8_t *b,uint8_t i){
  uint32_t v=0;for(uint8_t j=0;j<4;++j)v|=(uint32_t)b[i+j]<<(8*j);
  return v;
}
void beginAcquisition(uint32_t now){
  if(cycleActive)completeAcquisition(TRACK_RECOVERY,now);
  cycleActive=true;
  cycleDetectionAt=now;
  cycleGap10=(uint16_t)min((uint32_t)65535,
      (uint32_t)(now-(lastDetectionAt?lastDetectionAt:logBootAt))/10U);
  lastDetectionAt=now;
  cycleAimAt=0;cycleFirstFireAt=0;
  cycleScanMs=cycleAimMs=cycleFireMs=cycleArcWidthCdeg=cycleScoreGap10=0;
  cycleFireErrorCdeg=0;cyclePeakRPM=0;
  targetAimStartedAt=0;firstFireAt=0;fireQualitySince=0;
  cycleAbortReason=0;shotAttempted=false;
  laserWatchdogTripped=false;
}
void completeAcquisition(uint8_t outcome,uint32_t now){
  if(!cycleActive)return;
  if(!cycleScanMs)
    cycleScanMs=(uint16_t)min((uint32_t)65535,now-cycleDetectionAt);
  if(cycleAimAt && !cycleAimMs)
    cycleAimMs=(uint16_t)min((uint32_t)65535,
       (cycleFirstFireAt?cycleFirstFireAt:now)-cycleAimAt);
  if(cycleFirstFireAt)
    cycleFireMs=(uint16_t)min((uint32_t)65535,now-cycleFirstFireAt);
  if(outcome==HIT_2500MS){
    cycleScoreGap10=(uint16_t)min((uint32_t)65535,
      ((uint32_t)(now-(lastSuccessfulAt?lastSuccessfulAt:logBootAt)))/10U);
    lastSuccessfulAt=now;
  }
  if(logQueued<4){
    uint8_t *b=logQueue[logTail];
    put32(b,0,cycleDetectionAt);
    put16(b,4,cycleGap10);
    put16(b,6,cycleScanMs);
    put16(b,8,cycleAimMs);
    put16(b,10,cycleFireMs);
    put16(b,12,cycleArcWidthCdeg);
    put16(b,14,(uint16_t)cycleFireErrorCdeg);
    put16(b,16,cycleScoreGap10);
    b[18]=cyclePeakRPM;
    b[19]=(uint8_t)((outcome&0x0F)|((cycleAbortReason&0x0F)<<4));
    b[20]=logSession;
    b[21]=logCrc(b);
    logTail=(logTail+1)%4;++logQueued;
  }else ++logDropped;
  cycleActive=false;
  targetAimStartedAt=0;
  fireQualitySince=0;
}
bool writeEEPROMByte(uint16_t a,uint8_t value){
  if(EECR&_BV(EEPE))return false;
  if(EEPROM.read(a)==value)return true;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    EEAR=a;EEDR=value;EECR|=_BV(EEMPE);EECR|=_BV(EEPE);
  }
  return true;
}
void recorderService(){
  if(!logQueued)return;
  const uint8_t slot=logSequence; // V39: next EEPROM slot 0..44
  if(logByte<22){
    if(writeEEPROMByte(16+(uint16_t)slot*22+logByte,
                       logQueue[logHead][logByte]))++logByte;
  }else if(logByte==22){
    // Advance write head after all 22 record bytes have been written.
    if(writeEEPROMByte(2,(uint8_t)((logSequence+1)%LOG_SLOTS)))++logByte;
  }else{
    const uint8_t nextCount=logCount<LOG_SLOTS?logCount+1:LOG_SLOTS;
    if(!writeEEPROMByte(3,nextCount))return;
    logSequence=(uint8_t)((logSequence+1)%LOG_SLOTS);
    logCount=nextCount;
    logHead=(logHead+1)%4;--logQueued;logByte=0;
  }
}
void recorderInit(){
  // V38 and V39 have the identical 22-byte acquisition record layout and CRC.
  // Preserve those bytes and the 45-slot write head during V40 migration.
  const uint8_t previous=EEPROM.read(1);
  if(EEPROM.read(0)==EEPROM_MAGIC && previous==38){
    const uint8_t oldSeq=EEPROM.read(2);
    const uint8_t oldFull=EEPROM.read(3);
    const uint8_t next=oldSeq%LOG_SLOTS;
    const uint8_t count=oldFull==1?LOG_SLOTS:
       min((uint8_t)LOG_SLOTS,oldSeq);
    EEPROM.update(0,0);
    EEPROM.update(2,next);EEPROM.update(3,count);
    EEPROM.update(5,0);EEPROM.update(6,0);
    EEPROM.update(1,EEPROM_VERSION);EEPROM.update(0,EEPROM_MAGIC);
  }else if(EEPROM.read(0)==EEPROM_MAGIC && previous==39){
    EEPROM.update(0,0);
    EEPROM.update(5,0);EEPROM.update(6,0);
    EEPROM.update(1,EEPROM_VERSION);EEPROM.update(0,EEPROM_MAGIC);
  }else if(EEPROM.read(0)!=EEPROM_MAGIC||previous!=EEPROM_VERSION){
    const uint8_t previousSession=EEPROM.read(4);
    EEPROM.update(0,0);EEPROM.update(1,EEPROM_VERSION);
    EEPROM.update(2,0);EEPROM.update(3,0);
    EEPROM.update(4,previousSession==255?0:previousSession);
    EEPROM.update(5,0);EEPROM.update(6,0);
    EEPROM.update(0,EEPROM_MAGIC);
  }
  logSequence=EEPROM.read(2);
  logCount=EEPROM.read(3);
  if(logSequence>=LOG_SLOTS || logCount>LOG_SLOTS){
    logSequence=logCount=0;
    EEPROM.update(2,0);EEPROM.update(3,0);
  }
  seekStallReversals=(uint16_t)EEPROM.read(5)|
      ((uint16_t)EEPROM.read(6)<<8);
  logSession=(uint8_t)(EEPROM.read(4)+1);
  EEPROM.update(4,logSession);
  logBootAt=millis();lastDetectionAt=0;lastSuccessfulAt=0;
  Serial.print(F("AUTO_RUN,V40,session="));Serial.println(logSession);
}

void printLog(){
  Serial.println(F("ACQUISITION_LOG_BEGIN,V40"));
  Serial.println(F("run,detect_ms,detect_gap_ms,scan_ms,aim_ms,laser_ms,arc_cdeg,fire_error_cdeg,score_gap_ms,peak_wheel_rpm,result,abort_cause"));
  const uint8_t count=logCount;
  const uint8_t start=(uint8_t)((logSequence+LOG_SLOTS-count)%LOG_SLOTS);
  uint16_t good=0,failCrc=0;
  uint32_t fireTotal=0,detectTotal=0,aimTotal=0,scoreTotal=0;
  uint16_t successes=0,aborts=0,afterFirstCount=0;
  uint16_t laserAttempts=0,nearZeroAborts=0;
  uint32_t totalAttemptLaserMs=0;
  uint32_t afterFirstTotal=0;
  uint8_t observedSession=0;
  bool haveSession=false,seenSuccessThisRun=false;
  for(uint8_t i=0;i<count;++i){
    const uint8_t slot=(start+i)%LOG_SLOTS;
    uint8_t b[22];
    for(uint8_t j=0;j<22;++j)b[j]=EEPROM.read(16+(uint16_t)slot*22+j);
    if(logCrc(b)!=b[21]){++failCrc;continue;}
    ++good;
    Serial.print(b[20]);Serial.print(',');
    Serial.print(get32(b,0));Serial.print(',');
    Serial.print((uint32_t)get16(b,4)*10UL);Serial.print(',');
    Serial.print(get16(b,6));Serial.print(',');
    Serial.print(get16(b,8));Serial.print(',');
    Serial.print(get16(b,10));Serial.print(',');
    Serial.print(get16(b,12));Serial.print(',');
    Serial.print((int16_t)get16(b,14));Serial.print(',');
    Serial.print((uint32_t)get16(b,16)*10UL);Serial.print(',');
    Serial.print((unsigned int)b[18]);Serial.print(',');
    Serial.print((unsigned int)(b[19]&0x0F));Serial.print(',');
    Serial.println((unsigned int)(b[19]>>4));
    detectTotal+=(uint32_t)get16(b,4)*10UL;
    aimTotal+=get16(b,8);
    if(!haveSession || observedSession!=b[20]){
      observedSession=b[20];haveSession=true;seenSuccessThisRun=false;
    }
    if((b[19]&0x0F)==HIT_2500MS){
      ++successes;fireTotal+=get16(b,10);scoreTotal+=(uint32_t)get16(b,16)*10UL;
      if(seenSuccessThisRun){++afterFirstCount;afterFirstTotal+=(uint32_t)get16(b,16)*10UL;}
      seenSuccessThisRun=true;
    }
    const uint8_t outcome=b[19]&0x0F;
    if(outcome==SHOT_INTERRUPTED){
      ++aborts;
      if(get16(b,10)==0)++nearZeroAborts;
    }
    if(outcome==HIT_2500MS || outcome==SHOT_INTERRUPTED ||
       outcome==FIRE_QUALITY_TIMEOUT){
      ++laserAttempts;
      totalAttemptLaserMs+=get16(b,10);
    }
  }
  Serial.print(F("ACQUISITION_LOG_END,records="));Serial.print(good);
  Serial.print(F(",bad_crc="));Serial.print(failCrc);
  Serial.print(F(",success="));Serial.print(successes);
  Serial.print(F(",aborts="));Serial.println(aborts);
  Serial.print(F("MEAN_AIM_MS,"));Serial.println(good?aimTotal/good:0);
  Serial.print(F("MEAN_DETECT_GAP_MS,"));Serial.println(good?detectTotal/good:0);
  Serial.print(F("MEAN_SUCCESS_LASER_MS,"));Serial.println(successes?fireTotal/successes:0);
  Serial.print(F("MEAN_SCORE_GAP_INCL_FIRST_MS,"));Serial.println(successes?scoreTotal/successes:0);
  Serial.print(F("AVG_BETWEEN_SCORES_MS,"));Serial.println(afterFirstCount?afterFirstTotal/afterFirstCount:0);
  Serial.print(F("LASER_ATTEMPTS_LOGGED,"));Serial.println(laserAttempts);
  Serial.print(F("MEAN_LASER_ON_MS_PER_ATTEMPT,"));Serial.println(
    laserAttempts?totalAttemptLaserMs/laserAttempts:0);
  Serial.print(F("ZERO_MS_INTERRUPTS,"));Serial.println(nearZeroAborts);
  Serial.println(F("NOTE,completed_shots_are_not_verified_marker_points"));
  Serial.print(F("LOG_QUEUE_DROPPED,"));Serial.println(logDropped);
  Serial.print(F("SEEK_SLOWDOWN_REVERSALS_TOTAL,"));Serial.println(seekStallReversals);
  Serial.println(F("RESULTS,1=completed_laser_command_not_marker_verified,2=severe_abort,3=aim_skipped_by_3s,4=bad_arc,5=recover,6=sensor_stop,7=user_stop,8=scan_timeout,9=max_laser_watchdog"));
  Serial.println(F("ABORT_CAUSE,0=none,1=IR_stale,2=angle,3=body_rate,4=wheel_rpm,5=timeout,6=sensor"));
}
void recorderCommands(){
  if(!Serial.available())return;
  const char c=Serial.read();
  if(c!='d'&&c!='D'&&c!='s'&&c!='S'&&c!='c'&&c!='C')return;
  laserOff();
  completeAcquisition(USER_STOP,millis());
  motorOff();enter(STOP);
  // Explicit serial commands may block, but motor and laser are already OFF.
  while(logQueued || (EECR&_BV(EEPE)))recorderService();
  if(c=='c'||c=='C'){
    EEPROM.update(2,0);EEPROM.update(3,0);
    logSequence=logCount=0;seekStallReversals=0;
    EEPROM.update(5,0);EEPROM.update(6,0);
    Serial.println(F("LOG_CLEARED,press_RESET_to_run"));
  }else if(c=='d'||c=='D')printLog();
  else Serial.println(F("STOPPED,press_RESET_to_run"));
}

void setup(){
  pinMode(PIN_IN1,OUTPUT);pinMode(PIN_IN2,OUTPUT);pinMode(PIN_LASER,OUTPUT);
  pinMode(PIN_LED,OUTPUT);pinMode(PIN_ENC_A,INPUT_PULLUP);
  pinMode(PIN_ENC_B,INPUT_PULLUP);pinMode(PIN_IR,INPUT_PULLUP);
  motorOff();laserOff();initLaserWatchdog();Serial.begin(115200);
  Wire.begin();Wire.setClock(100000UL);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000UL,true);   
#endif
  delay(100);
  const bool mpuOK=mpuWrite(0x6B,0x00)&&mpuWrite(0x1A,0x03)&&
    mpuWrite(0x1B,0x00)&&mpuWrite(0x1C,0x00)&&mpuWrite(0x19,0x00);
  if(!mpuOK)safeStop(1);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),encoderRise,RISING);
  isLow=(PINB&_BV(PB3))==0;
  if(isLow){lowStartUs=micros();lastLowSeenUs=micros();}
  PCMSK0|=_BV(PCINT3);PCICR|=_BV(PCIE0);
  enteredMs=millis();lastGyroUs=micros();
  Serial.println(F("MQ SENTINEL V40 | EARLY FIRE <=3s, 2.5s SHOT, SEEK FLIP, D=DUMP"));
  recorderInit();
  if(!mpuOK){laserOff();motorOff();state=STOP;}

}
void loop(){
  const uint32_t now=millis();

  if(laserWatchdogTripped && state==FIRE){
    laserOff();cycleAbortReason=5;
    completeAcquisition(FIRE_QUALITY_TIMEOUT,now);
    completedTargetDeparture();
    laserWatchdogTripped=false;
  }
  gyroUpdate(now);
  irUpdate(now);
  rpmUpdate(now);
  if(state!=STOP)sequence(now);
  motorUpdate(now);
  digitalWrite(PIN_LED,state==STOP?((now/125)&1):laserHigh);
#if LIVE_TELEMETRY
  telemetry(now);
#endif
  recorderService();recorderCommands();

}
