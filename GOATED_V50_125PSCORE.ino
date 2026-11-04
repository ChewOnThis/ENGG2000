/* MQ SENTINEL V50 - smooth low-jerk aiming + reliable precision firing.
   Based on V47; keeps high-speed SEEK (65 deg/s target), slowed IR scanning,
   quick wind-up reversal, 2.5 s one-shot laser and the V40 EEPROM format.

   Root fix: eliminate alternating 5-20ms fine motor pulses and discontinuous
   AIM/FINE/FIRE commands. Use gyro/encoder feedback with continuously ramped
   wheel drive, predictive velocity braking, and a narrow centred firing gate.
   After a successful IR arc the laser does not require a continuously fresh
   IR pulse, because the body can leave/re-enter that beam while repositioning.
   Ideal laser permission is +/-0.8deg at low yaw rate; after the first 1.35s
   the window gradually expands to +/-1.45deg maximum with controlled rate.
   A 4.5s alignment ceiling remains; no fully unqualified forced shot.

   DRV8874 assumed IN/IN PMODE=HIGH (D5/D6). Quiet hold uses both LOW = coast.
   Test with demonstration laser DISCONNECTED before using it at the target.
*/

struct FireGate; // Forward declaration for Arduino sketch auto-prototypes.

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

const uint8_t PIN_IN1=5, PIN_IN2=6, PIN_LASER=4;
const uint8_t PIN_ENC_A=3, PIN_ENC_B=2, PIN_IR=11, PIN_LED=13, MPU=0x68;
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
// Slow, consistent angular edges reduce midpoint bias from a fast sweep.
const float SCAN_ENTRY_DPS=16.0f, SCAN_MIN_DPS=8.0f;
const float SCAN_SLOW_PER_DEG=0.72f, SCAN_RATE_PWM_LIMIT=105.0f;
// HM 2.5 used a separate long brake state. This is a short active gyro damper.
const uint16_t ARC_DAMP_MIN_MS=65, ARC_DAMP_MAX_MS=300;
const uint16_t STARTUP_DAMP_MIN_MS=150, STARTUP_DAMP_MAX_MS=550;
const float ARC_DAMP_READY_DPS=4.2f;
// Fast return to the midpoint, early braking at the end of approach.
const float AIM_MAX_DPS=13.0f, AIM_BRAKE_DECEL_DPS2=21.0f;
const float STARTUP_AIM_MAX_DPS=10.0f;
const float AIM_BRAKE_MARGIN_DEG=0.55f, AIM_RATE_PWM_LIMIT=125.0f;
// V42: anticipate rotation before the gyro-integrated angle passes centre.
const float AIM_EXTRA_BRAKE_LOOKAHEAD_SEC=0.08f;
const float HOLD_PREDICT_SEC=0.22f;
// Dead-centre coast only after real measured stillness; release promptly.
const float QUIET_ENTER_ERROR_DEG=0.22f, QUIET_EXIT_ERROR_DEG=0.43f;
const float QUIET_ENTER_RATE_DPS=0.38f, QUIET_EXIT_RATE_DPS=1.05f;
const float QUIET_ENTER_RAW_DPS=0.75f, QUIET_EXIT_RAW_DPS=1.65f;
const float QUIET_MAX_DRIFT_DEG=0.085f;
const uint16_t QUIET_CONFIRM_MS=120;
// HM 2.5: aiming target is +/-0.18 deg, not laser permission tolerance.
const float FINE_ENTER_DEG=3.2f, FINE_REENTER_DEG=4.4f, FINE_MIN_ERROR=0.18f;
// Smooth body-rate control; PWM changes are gradually limited per second.
// These are *PWM* change limits, not the motor/gyro angular velocities.
const float SCAN_PWM_SLEW=900.0f;
const float AIM_DAMP_PWM_SLEW=780.0f;
const float AIM_COARSE_PWM_SLEW=560.0f;
const float AIM_FINE_PWM_SLEW=365.0f;
const float FIRE_PWM_SLEW_PER_SEC=320.0f;
const float FINE_RATE_KP=6.5f, FINE_POSITION_KP=1.35f;
const float FINE_MAX_TARGET_DPS=4.2f;
const float HOLD_KI=0.8f, HOLD_BIAS_LIMIT=20.0f;
// HM's +/-0.18deg fine control target is separate from firing permission.
// Two fallback bands recover attempts that V47 rejected, without firing far
// outside the validated midpoint or while rotating rapidly through it.
const float FIRE_ANGLE_TOL_DEG=0.85f, FIRE_ANGLE_MIN_TOL_DEG=0.70f;
const uint16_t MAX_AIM_FINE_MS=4500;
// First two acquisitions only: more settling time, not a forced laser shot.
const uint16_t STARTUP_AIM_EXTRA_MS=1200;
const float STARTUP_EXTENSION_MAX_ERROR_DEG=5.0f;
const float STARTUP_EXTENSION_MAX_RATE_DPS=10.0f;
const uint16_t EARLY_FIRE_AT_MS=450, PRACTICAL_FIRE_AT_MS=1300;
const uint16_t LATE_FIRE_AT_MS=3000, FINAL_FIRE_AT_MS=4000;
const uint16_t FIRE_EARLY_CONFIRM_MS=75, FIRE_LATE_CONFIRM_MS=45;
const float FIRE_ENTRY_RATE_DPS=1.85f, FIRE_ENTRY_RAW_DPS=3.2f;
const float FIRE_LATE_RATE_DPS=3.4f, FIRE_LATE_RAW_DPS=5.6f;
const float FORCED_FIRE_MAX_WHEEL_RPM=185.0f;
const uint16_t FIRE_MIN_MS=2000, MAX_FIRE_EPISODE_MS=2600;
const uint16_t FIRE_BAD_ANGLE_GRACE_MS=450;
const float FIRE_ABORT_EXTRA_DEG=2.0f;
const float FIRE_ABORT_RATE_DPS=35.0f;
const uint16_t LASER_WATCHDOG_TICKS=188;     // Timer2 ~62.5Hz, ~3.008s
// SEEK: detect wind-up as yaw-rate collapse relative to the current sweep.
const uint16_t SEEK_SLOW_ARM_MS=650, SEEK_SLOW_CONFIRM_MS=375;
const float SEEK_SLOW_ABS_DPS=2.8f, SEEK_SLOW_RELATIVE=0.33f;
const float SEEK_BASELINE_MIN_DPS=9.0f;
const float MAX_FIRE_WHEEL_RPM=185.0f;
// Substantially faster than V42 (16 dps) and V45 (34 dps).
// The rate governor does not guarantee a particular physical speed.
const float SEARCH_SPEED_TARGET_DPS=65.0f;
const float SEARCH_SPEED_ENTER_DPS=70.0f, SEARCH_SPEED_EXIT_DPS=57.0f;
const float SEARCH_PWM_BASE=180.0f, SEARCH_PWM_STALL_MAX=205.0f;
const float IR_CAPTURE_TARGET_DPS=17.0f;
const float SEARCH_LIMIT_SLEW=550.0f;
const float DEPARTURE_PWM_PER_SEC=250.0f, DEPARTURE_ACCEL_DPS2=15.0f;
const uint16_t WRONG_SEARCH_CONFIRM_MS=400, STARTUP_PROGRESS_WINDOW_MS=5000;
const float WRONG_SEARCH_DPS=2.0f, STARTUP_MIN_PROGRESS_DEG=4.0f;

// State values kept for compatibility with EEPROM V33 dump state IDs.
enum State:uint8_t {BOOT=0, CLEAR=2, SEEK=3, SLOW_SCAN=7, ARC_DAMP=8,
                    AIM=9, FINE=10, FIRE=11, UNWIND=12, STOP=13};
State state=BOOT;
uint32_t enteredMs=0, clearSince=0, lastGyroMs=0, lastGyroUs=0;
uint32_t lastMotorMs=0, stableSince=0, arcDampStableAt=0;
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

uint8_t stopCode=0, failedI2C=0;

float gyroBias=0,yaw=0,yawRate=0,yawRateFiltered=0;
bool calibrated=false,irStuckReported=false;
volatile bool laserHigh=false,laserWatchdogTripped=false;
volatile uint16_t laserWatchdogRemaining=0;
bool shotAttempted=false;
uint32_t targetAimStartedAt=0,firstFireAt=0;
uint32_t badFireAngleSince=0;
// Quiet hold runs only in FINE/FIRE; the gyro and integrated yaw keep updating.
bool quietHold=false;
uint32_t quietCandidateAt=0;
float quietCandidateYaw=0;
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
// One continuous wheel-drive envelope across DAMP, AIM, FINE and FIRE.
float precisionMotorPWM=0.0f;
bool precisionMotorReady=false;
volatile bool isLow=false;
volatile uint32_t lowStartUs=0,lastLowSeenUs=0;
volatile uint16_t validPulseTotal=0;
ISR(PCINT0_vect) {
  const bool low=(PINB & _BV(PB3))==0;
  if(low==isLow)return;
  const uint32_t t=micros();
  isLow=low;
  if(low){lowStartUs=t;lastLowSeenUs= t;}
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
float allowableFireError();
void initialiseArc(float entry,uint32_t ms);
void controlledSearchRecovery(bool reverseDirection);
void restartSearch(bool reverseDirection);
void recoverStall();


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
uint8_t acquisitionCountSinceBoot=0;
bool startupAcquisition(){return acquisitionCountSinceBoot<=2;}

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
// DRV8874 IN/IN mode: IN1=IN2=0 is Hi-Z/coast, not electrical braking.
// Use only for precision quiet hold; STOP still uses the original active brake.
void motorCoast(){
  analogWrite(PIN_IN1,0);
  analogWrite(PIN_IN2,0);
}
void motorOff(){pwmApplied=0;motorHardware(0);}

void enter(State s){
  state=s;enteredMs=millis();
  if(s!=CLEAR && s!=SEEK && s!=SLOW_SCAN)wrongSearchAt=0;
  if(s==SEEK){seekStallAt=0;seekLevel=SEARCH_PWM_BASE;
    seekSlowSince=0;seekPeakRate=0;
    startupProgressAt=enteredMs;startupProgressYaw=yaw;}
  if(s!=AIM && s!=FINE && s!=FIRE){trackingProgressAt=0;headroomAt=0;holdBias=0;}
  // Preserve an existing quiet hold during FINE -> FIRE, but never across
  // SEEK/CLEAR or a new AIM correction cycle.
  if(s!=FINE && s!=FIRE){quietHold=false;quietCandidateAt=0;}
  holdBiasAt=enteredMs;
  if(s==ARC_DAMP){precisionMotorPWM=(float)pwmApplied;precisionMotorReady=true;}
  if(s!=ARC_DAMP && s!=AIM && s!=FINE && s!=FIRE)
    precisionMotorReady=false;
  if(s!=AIM && s!=FINE)stableSince=0;
}
void safeStop(uint8_t c){
  if(state==STOP)return;
  stopCode=c;laserOff();motorOff();
  completeAcquisition(SENSOR_STOP,millis());
  enter(STOP);
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
  yaw=yawRate=yawRateFiltered=0;
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
  failedI2C=0;
  if(calibrated){
    const float dt=(uint32_t)(us-lastGyroUs)*0.000001f;
    if(dt>0.12f||absf(r)>=235||g>1.75f||g<0.36f){
      if(state!=STOP){
        safeStop(3);
      }
    }
    else {
      yawRate=r-gyroBias;
      if(absf(yawRate)<0.07f && absf(wheelRPM)<1.0f && pwmApplied==0)yawRate=0;
      yaw+=yawRate*dt;
      yawRateFiltered+=RATE_LPF*(yawRate-yawRateFiltered);


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
  laserOff();arcValid=false;slowArcStarted=false;stableSince=0;
  if(++scanFailCount>=3){scanFailCount=0;controlledSearchRecovery(true);}
  else enter(SEEK);
}
void restartSearch(bool reverseDirection){
  completeAcquisition(TRACK_RECOVERY,millis());
  controlledDeparture=false;
  laserOff();
  arcValid=false;slowArcStarted=false;stableSince=0;
  if(reverseDirection)seekDirection=-seekDirection;
  seekLevel=SEARCH_PWM_BASE;seekStallAt=0;clearSince=0;
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
  stableSince=0;clearSince=0;
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

    cycleScanMs=(uint16_t)min((uint32_t)65535,ms-cycleDetectionAt);
    cycleArcWidthCdeg=(uint16_t)clampf(arcWidth*100.0f,0,65535);
    completeAcquisition(SCAN_REJECT,millis());
    rescanOrSeek();
  }
  else{
    scanFailCount=0;
    arcCentre=(arcEntry+arcExit)*0.5f;arcValid=true;
    cycleScanMs=(uint16_t)min((uint32_t)65535,ms-cycleDetectionAt);
    cycleArcWidthCdeg=(uint16_t)clampf(arcWidth*100.0f,0,65535);
    cycleAimAt=millis();targetAimStartedAt=cycleAimAt;shotAttempted=false;
    arcDampStableAt=0;
    // Brake body rotation with gyro feedback rather than an extended
    // motor short. Starts the 4.5s AIM budget now; max damping ~440ms.
    enter(ARC_DAMP);
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
float cruisePWM(){
  return seekDirection*clampf(seekLevel,SEARCH_PWM_BASE,SEARCH_PWM_STALL_MAX);
}
float scanPWM(){
  // Control actual yaw rate; decelerate progressively as the IR arc develops.
  const float span=absf(yaw-arcEntry);
  const float speed=clampf(SCAN_ENTRY_DPS-SCAN_SLOW_PER_DEG*span,
                            SCAN_MIN_DPS,SCAN_ENTRY_DPS);
  return ratePWM(BODY_SIGN*seekDirection*speed,SCAN_RATE_PWM_LIMIT);
}
// Rate feedback acts on the body, using wheel RPM feed-forward to avoid
// abruptly dragging an already-spinning reaction wheel to a halt.
float precisionRatePWM(float targetDps,float rateKp,float maxCorrection){
  const float correction=clampf(rateKp*(targetDps-yawRateFiltered),
                              -maxCorrection,maxCorrection);
  return clampf(wheelFeedforward()+BODY_SIGN*correction,-255.0f,255.0f);
}
float predictedCentreError(){
  return (arcCentre-yaw)-yawRateFiltered*HOLD_PREDICT_SEC;
}
float coarsePWM(){
  const float err=arcCentre-yaw,dist=absf(err);
  const float rate=yawRateFiltered;
  const float approaching=(err*rate>0.0f)?absf(rate):0.0f;
  // Reserve physical stopping distance rather than abruptly reversing torque
  // once the midpoint has already been crossed.
  const float stopDistance=approaching*approaching/(2.0f*AIM_BRAKE_DECEL_DPS2)
      +approaching*AIM_EXTRA_BRAKE_LOOKAHEAD_SEC;
  const float available=fmaxf(0.0f,dist-AIM_BRAKE_MARGIN_DEG-stopDistance);
  const float speedCap=fminf(startupAcquisition()?STARTUP_AIM_MAX_DPS:AIM_MAX_DPS,
       sqrtf(2.0f*AIM_BRAKE_DECEL_DPS2*available));
  const float desired=clampf(1.9f*(err-rate*0.17f),-speedCap,speedCap);
  return precisionRatePWM(desired,6.5f,AIM_RATE_PWM_LIMIT);
}
float fineHoldPWM(){
  // Near centre, reduce desired angular speed continuously toward ZERO.
  // The predictive term avoids powering through the midpoint while drifting.
  const float err=arcCentre-yaw,rate=yawRateFiltered;
  const float predicted=err-rate*HOLD_PREDICT_SEC;
  float desired=clampf(FINE_POSITION_KP*predicted,
                       -FINE_MAX_TARGET_DPS,FINE_MAX_TARGET_DPS);
  if(absf(err)<=FINE_MIN_ERROR && absf(rate)<0.55f)desired=0;
  const float effort=clampf(FINE_RATE_KP*(desired-rate)+holdBias,-85.0f,85.0f);
  return clampf(wheelFeedforward()+BODY_SIGN*effort,-255.0f,255.0f);
}
// Coast only when well-centred and nearly motionless. Keep HM 2.5's tighter
// centring target, but never impose abrupt electrical braking on the wheel.
bool updateQuietHold(uint32_t now){
  if((state!=FINE && state!=FIRE)||!calibrated||!arcValid){
    quietHold=false;quietCandidateAt=0;return false;
  }
  const float err=absf(arcCentre-yaw);
  const float predicted=absf(predictedCentreError());
  const float rate=absf(yawRateFiltered);
  const float raw=absf(yawRate);
  if(quietHold){
    if(err>QUIET_EXIT_ERROR_DEG || predicted>QUIET_EXIT_ERROR_DEG ||
       rate>QUIET_EXIT_RATE_DPS || raw>QUIET_EXIT_RAW_DPS){
      quietHold=false;quietCandidateAt=0;
    }else return true;
  }
  const bool candidate=err<=QUIET_ENTER_ERROR_DEG &&
      predicted<=QUIET_ENTER_ERROR_DEG &&
      rate<=QUIET_ENTER_RATE_DPS && raw<=QUIET_ENTER_RAW_DPS;
  if(!candidate){quietCandidateAt=0;return false;}
  if(!quietCandidateAt){quietCandidateAt=now;quietCandidateYaw=yaw;return false;}
  if(absf(yaw-quietCandidateYaw)>QUIET_MAX_DRIFT_DEG){
    quietCandidateAt=now;quietCandidateYaw=yaw;return false;
  }
  if(timeElapsed(now,quietCandidateAt,QUIET_CONFIRM_MS)){
    quietHold=true;quietCandidateAt=0;
    holdBias=0;holdBiasAt=now;
    return true;
  }
  return false;
}
void updateHoldBias(uint32_t now){
  const float dt=clampf((uint32_t)(now-holdBiasAt)*0.001f,0,0.02f);
  holdBiasAt=now;
  const float err=arcCentre-yaw;
  if(!arcValid || absf(err)>FINE_REENTER_DEG ||
       absf(yawRateFiltered)>5.5f)return;
  const float candidate=clampf(holdBias+HOLD_KI*err*dt,
                                -HOLD_BIAS_LIMIT,HOLD_BIAS_LIMIT);
  const float requested=clampf(FINE_POSITION_KP*predictedCentreError(),-4.2f,4.2f);
  const float inner=FINE_RATE_KP*(requested-yawRateFiltered)+candidate;
  const float raw=wheelFeedforward()+BODY_SIGN*clampf(inner,-85,85);
  if((absf(inner)<=85 || (candidate-holdBias)*inner<0) &&
     (absf(raw)<=255 || BODY_SIGN*(candidate-holdBias)*raw<0))
    holdBias=candidate;
}
float finePWM(uint32_t now){
  updateHoldBias(now);
  return fineHoldPWM(); // Continuous rate/position correction; NO pulses.
}
// Preserve the physical motor drive across state boundaries; limit how fast
// its effective PWM can change, including reversal. Crucially use float state
// (not repeatedly rounded integer PWM) for reliable small 5-ms increments.
float smoothPrecisionPWM(float wanted,float dt,float slew){
  if(!precisionMotorReady){
    precisionMotorPWM=(float)pwmApplied;
    precisionMotorReady=true;
  }
  precisionMotorPWM+=clampf(wanted-precisionMotorPWM,-slew*dt,slew*dt);
  return precisionMotorPWM;
}

// Rate-limit SEEK and decelerate as soon as IR is suspected.
// The 2-pulse acquisition filter is unchanged; a single pulse only slows yaw.
float limitSearchPWM(uint32_t now,float normalCommand,float dt){
  if(state==SLOW_SCAN){
    // Once the target is detected, scanPWM owns gyro speed and slow edges.
    // Otherwise the IR-capture governor pins SCAN near 17 deg/s.
    searchSpeedActive=searchSpeedRelease=false;
    searchSpeedSafeAt=0;
    return normalCommand;
  }
  if(state!=SEEK && state!=CLEAR){
    searchSpeedActive=searchSpeedRelease=false;
    searchSpeedSafeAt=0;
    return normalCommand;
  }
  const bool irHint=state==SEEK && !ir.active &&
    ((ir.groupPulses && ir.groupStart &&
      (uint32_t)(now-ir.groupStart)<IR_GROUP_MS) || ir.low);
  const bool capture=irHint;
  const float requestedSpeed=capture?IR_CAPTURE_TARGET_DPS:SEARCH_SPEED_TARGET_DPS;
  const float actual=fmaxf(absf(yawRate),absf(yawRateFiltered));
  const float enterSpeed=capture?IR_CAPTURE_TARGET_DPS+4.0f:SEARCH_SPEED_ENTER_DPS;
  const float exitSpeed=capture?IR_CAPTURE_TARGET_DPS-2.0f:SEARCH_SPEED_EXIT_DPS;
  if(capture || (!searchSpeedActive && actual>=enterSpeed)){
    searchSpeedActive=true;
    searchSpeedRelease=false;
    searchSpeedSafeAt=0;
  }
  if(searchSpeedActive && !capture){
    if(actual<=exitSpeed){
      if(!searchSpeedSafeAt)searchSpeedSafeAt=now;
      if(timeElapsed(now,searchSpeedSafeAt,100)){
        searchSpeedActive=false;searchSpeedRelease=true;
      }
    }else searchSpeedSafeAt=0;
  }
  float wanted=normalCommand;
  if(searchSpeedActive){
    // A fast gyro-based rate controller, always pointing in intended direction.
    const float requested=BODY_SIGN*seekDirection*requestedSpeed;
    const float correction=clampf(4.0f*(requested-yawRate),-100.0f,100.0f);
    wanted=clampf(wheelFeedforward()+BODY_SIGN*correction,
                  -MOTOR_PWM_LIMIT,MOTOR_PWM_LIMIT);
  }
  if(searchSpeedActive||searchSpeedRelease){
    const float result=pwmApplied+clampf(wanted-pwmApplied,
         -SEARCH_LIMIT_SLEW*dt,SEARCH_LIMIT_SLEW*dt);
    if(searchSpeedRelease && absf(result-normalCommand)<1.0f)
      searchSpeedRelease=false;
    return result;
  }
  return normalCommand;
}

void motorUpdate(uint32_t now){
  if(now-lastMotorMs<MOTOR_PERIOD_MS)return;
  const float dt=clampf((uint32_t)(now-lastMotorMs)*0.001f,0,0.02f);
  lastMotorMs=now;
  float command=0;
  if(state==CLEAR||state==SEEK)command=cruisePWM();
  else if(state==SLOW_SCAN){
    // A sudden 65dps SEEK -> slow SCAN command is another source of wobble.
    // Rate-limit the motor handoff while still responding to measured gyro rate.
    const float wanted=scanPWM();
    command=pwmApplied+clampf(wanted-pwmApplied,
         -SCAN_PWM_SLEW*dt,SCAN_PWM_SLEW*dt);
  }
  else if(state==ARC_DAMP){
    command=smoothPrecisionPWM(precisionRatePWM(0.0f,7.0f,115.0f),
                               dt,AIM_DAMP_PWM_SLEW);
  }else if(state==AIM){
    command=smoothPrecisionPWM(coarsePWM(),dt,AIM_COARSE_PWM_SLEW);
  }else if(state==FINE || state==FIRE){
    if(updateQuietHold(now)){
      // Driver physically COASTS. Track spinning-wheel back-EMF so the first
      // command after a drift is near neutral, not a large braking impulse.
      precisionMotorPWM=wheelFeedforward();
      precisionMotorReady=true;
      command=precisionMotorPWM;
    }else{
      const float wanted=finePWM(now);
      const float slew=state==FIRE?FIRE_PWM_SLEW_PER_SEC:AIM_FINE_PWM_SLEW;
      command=smoothPrecisionPWM(wanted,dt,slew);
    }
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
  if(quietHold && (state==FINE || state==FIRE))motorCoast();
  else motorHardware(pwmApplied);
}

float fireTolerance(){
  return clampf(arcWidth*0.42f,FIRE_ANGLE_MIN_TOL_DEG,FIRE_ANGLE_TOL_DEG);
}



// In-flight tolerance is deliberately distinct from the tighter laser-ON gate.
// The old V38 maintainLock() aborted on a single small angle/rate excursion,
// explaining 18 shortened bursts. Minor deviations are corrected while ON.
// Severe deviations still force laser OFF immediately.
uint8_t fireAbortCause(uint32_t now){
  if(!calibrated || !arcValid || !laserHigh)return 6;
  // Allow the HM-style correction loop a moment to repair a small drift.
  if(absf(yawRateFiltered)>FIRE_ABORT_RATE_DPS ||
     absf(yawRate)>FIRE_ABORT_RATE_DPS+8.0f)return 3;
  if(absf(wheelRPM)>FORCED_FIRE_MAX_WHEEL_RPM)return 4;
  const float allowed=fmaxf(3.0f,allowableFireError()+FIRE_ABORT_EXTRA_DEG);
  if(absf(arcCentre-yaw)>allowed){
    if(!badFireAngleSince)badFireAngleSince=now;
    if(timeElapsed(now,badFireAngleSince,FIRE_BAD_ANGLE_GRACE_MS))return 2;
  }else badFireAngleSince=0;
  return 0;
}

// Precise entry and ongoing FINE feedback are deliberately distinct.


// Validated IR scan already supplies the midpoint. A fresh pulse may be
// absent as the chassis rotates BACK into the measured target. Require the
// current and predicted gyro angle to be within a narrow arc-derived window.
float allowableFireError(){
  // HM 2.5 laser permission; fine controller aims considerably tighter.
  // Very narrow detected arcs cannot authorise a wider off-arc shot.
  const float arcAllowance=0.5f*arcWidth+0.12f;
  return fminf(FIRE_ANGLE_TOL_DEG,fmaxf(0.18f,arcAllowance));
}

// Fire closer to the HM2.5 centre, but give up less often than the overly
// strict V47 gate. No fresh IR required AFTER a successfully validated scan.
// Time-dependent gate: centre-first, then a bounded opportunity shot.
// The fallback is NOT an unconditional timeout fire.
struct FireGate {float angle,projected,filteredRate,rawRate;uint16_t confirm;};
FireGate fireGateForAge(uint32_t age);
FireGate fireGateForAge(uint32_t age){
  if(age>=3350) return {2.15f,2.80f,6.0f,9.0f,25};
  if(age>=2300) return {1.65f,2.10f,4.5f,7.0f,35};
  if(age>=1100) return {1.20f,1.55f,3.0f,5.5f,45};
  return {0.90f,1.15f,2.2f,4.0f,65};
}
uint8_t aimSkippedReason(){
  if(!calibrated || !arcValid || irStuckReported)return 6;
  if(absf(wheelRPM)>FORCED_FIRE_MAX_WHEEL_RPM)return 4;
  const FireGate g=fireGateForAge(MAX_AIM_FINE_MS);
  if(absf(yawRateFiltered)>g.filteredRate ||
     absf(yawRate)>g.rawRate)return 3;
  if(absf(arcCentre-yaw)>g.angle ||
     absf(predictedCentreError())>g.projected)return 2;
  return 0;
}
bool updateLaserArming(uint32_t now){
  if(shotAttempted || !cycleActive || !calibrated || !arcValid ||
     !targetAimStartedAt || state==STOP){stableSince=0;return false;}
  const uint32_t age=(uint32_t)(now-targetAimStartedAt);
  if(age<400){stableSince=0;return false;}
  const FireGate g=fireGateForAge(age);
  const float e=arcCentre-yaw, rate=yawRateFiltered;
  // The arc midpoint is a geometric estimate; never fire outside the measured
  // central region, even if the time-based gate has widened.
  const float edge=fmaxf(0.30f,0.5f*arcWidth-0.25f);
  const float maxAngle=fminf(g.angle,edge);
  const float predicted=predictedCentreError();
  const bool movingToward=e*rate>=0.0f;
  const bool lowRate=absf(rate)<=2.0f;
  const bool good=absf(e)<=maxAngle &&
      absf(predicted)<=fminf(g.projected,edge+0.55f) &&
      absf(rate)<=g.filteredRate && absf(yawRate)<=g.rawRate &&
      absf(wheelRPM)<=FORCED_FIRE_MAX_WHEEL_RPM && !irStuckReported &&
      (lowRate || movingToward);
  if(!good){stableSince=0;return false;}
  if(!stableSince)stableSince=now;
  if(!timeElapsed(now,stableSince,g.confirm))return false;
  laserOn();
  if(laserHigh){enter(FIRE);return true;}
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
      // Means the 2000ms laser *command* completed, NOT a verified hit.
      completeAcquisition(HIT_2500MS,now);
      completedTargetDeparture();return;
    }
    return;
  }

  // The late HM-style gate is broader but still demands a near-centre,
  // slow body. At 4.5s skip if the actual pointing is not acceptable.
  if((state==ARC_DAMP || state==AIM || state==FINE) && targetAimStartedAt &&
      timeElapsed(now,targetAimStartedAt,MAX_AIM_FINE_MS)){
    // Only the first two acquisitions can earn an extra 1.2s, and only
    // while they remain close enough and slow enough to correct safely.
    const bool startupExtension=startupAcquisition() &&
        !timeElapsed(now,targetAimStartedAt,
                     MAX_AIM_FINE_MS+STARTUP_AIM_EXTRA_MS) &&
        absf(arcCentre-yaw)<=STARTUP_EXTENSION_MAX_ERROR_DEG &&
        absf(yawRateFiltered)<=STARTUP_EXTENSION_MAX_RATE_DPS &&
        absf(wheelRPM)<=FORCED_FIRE_MAX_WHEEL_RPM && !irStuckReported;
    if(startupExtension){} else {
    cycleAbortReason=aimSkippedReason();
    completeAcquisition(AIM_TIMEOUT,now);
    completedTargetDeparture();
    return;
    }
  }

  if(searchMovingBackward(now)){
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
            recoverStall();break;
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
          seekLevel=SEARCH_PWM_BASE+(SEARCH_PWM_STALL_MAX-SEARCH_PWM_BASE)*
              clampf((now-seekStallAt-700UL)/1500.0f,0,1);
      }else{seekStallAt=0;seekLevel=SEARCH_PWM_BASE;}
      break;
    }
    case SLOW_SCAN:
      if(timeElapsed(now,enteredMs,ARC_MAX_MS+250UL))rescanOrSeek();
      break;
    case ARC_DAMP:
      if(absf(yawRateFiltered)<=ARC_DAMP_READY_DPS){
        if(!arcDampStableAt)arcDampStableAt=now;
      }else arcDampStableAt=0;
      if((timeElapsed(now,enteredMs,startupAcquisition()?STARTUP_DAMP_MIN_MS:ARC_DAMP_MIN_MS) &&
          arcDampStableAt && timeElapsed(now,arcDampStableAt,
              startupAcquisition()?110:65)) ||
          timeElapsed(now,enteredMs,
              startupAcquisition()?STARTUP_DAMP_MAX_MS:ARC_DAMP_MAX_MS))enter(AIM);
      break;
    case AIM:
      if(updateLaserArming(now))break;
      if(absf(arcCentre-yaw)<=FINE_ENTER_DEG&&absf(yawRateFiltered)<=3.0f)
        enter(FINE);
      break;
    case FINE:
      if(absf(arcCentre-yaw)>FINE_REENTER_DEG){enter(AIM);break;}
      if(updateLaserArming(now))break;
      break;
    case FIRE: break; // Always processed at the beginning of sequence().
    case STOP:laserOff();break;
    default:safeStop(4);break;
  }
}


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
  if(acquisitionCountSinceBoot<3)++acquisitionCountSinceBoot;
  cycleDetectionAt=now;
  cycleGap10=(uint16_t)min((uint32_t)65535,
      (uint32_t)(now-(lastDetectionAt?lastDetectionAt:logBootAt))/10U);
  lastDetectionAt=now;
  cycleAimAt=0;cycleFirstFireAt=0;
  cycleScanMs=cycleAimMs=cycleFireMs=cycleArcWidthCdeg=cycleScoreGap10=0;
  cycleFireErrorCdeg=0;cyclePeakRPM=0;
  targetAimStartedAt=0;firstFireAt=0;
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
  // Preserve compatible 22-byte EEPROM records and the 45-slot write head.
  // Format version deliberately remains 40 across V41/V42 firmware updates.
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
  Serial.print(F("AUTO_RUN,V49,session="));Serial.println(logSession);
}

void printLog(){
  Serial.println(F("ACQUISITION_LOG_BEGIN,V49"));
  Serial.println(F("run,detect_ms,detect_gap_ms,scan_ms,aim_ms,laser_ms,arc_cdeg,fire_error_cdeg,score_gap_ms,peak_wheel_rpm,result,abort_cause"));
  const uint8_t count=logCount;
  const uint8_t start=(uint8_t)((logSequence+LOG_SLOTS-count)%LOG_SLOTS);
  uint16_t good=0,failCrc=0;
  uint32_t fireTotal=0,detectTotal=0,aimTotal=0,scoreTotal=0;
  uint16_t successes=0,aborts=0,afterFirstCount=0;
  uint16_t laserAttempts=0,nearZeroAborts=0;
  uint16_t within05=0,within09=0;
  uint32_t totalFireErrorCdeg=0;
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
      const int16_t signedError=(int16_t)get16(b,14);
      const uint16_t absError=(uint16_t)(signedError<0?-(int32_t)signedError:signedError);
      totalFireErrorCdeg+=absError;
      if(absError<=50)++within05;
      if(absError<=90)++within09;
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
  Serial.print(F("MEAN_ABS_FIRE_START_ERROR_CDEG,"));Serial.println(
      laserAttempts?totalFireErrorCdeg/laserAttempts:0);
  Serial.print(F("FIRE_START_WITHIN_0P5_DEG,"));Serial.println(within05);
  Serial.print(F("FIRE_START_WITHIN_0P9_DEG,"));Serial.println(within09);
  Serial.println(F("NOTE,completed_shots_are_not_verified_marker_points"));
  Serial.print(F("LOG_QUEUE_DROPPED,"));Serial.println(logDropped);
  Serial.print(F("SEEK_SLOWDOWN_REVERSALS_TOTAL,"));Serial.println(seekStallReversals);
  Serial.println(F("RESULTS,1=completed_laser_command_not_marker_verified,2=severe_abort,3=aim_skipped_by_4p5s,4=bad_arc,5=recover,6=sensor_stop,7=user_stop,8=scan_timeout,9=max_laser_watchdog"));
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
  Serial.println(F("MQ SENTINEL V49 | SEEK 65DPS, HM CENTRE +/-0.9, D=DUMP"));
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
  recorderService();recorderCommands();

}
