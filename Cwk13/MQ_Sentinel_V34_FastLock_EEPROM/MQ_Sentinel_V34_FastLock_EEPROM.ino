/* MQ SENTINEL V34 - faster acquisition / reduced legacy code
   Hardware: Nano ATmega328P, MPU6050 X yaw, D11 IR, D3/D2 encoder,
   D5/D6 DRV8874 IN-IN, D4 eye-safe demonstration laser.
   A=arm EEPROM logger for next boot, D=dump. Default USB boot is IDLE.
   EEPROM layout remains V33-compatible: 60 x 5-second windows and 24 events.
   V34 tuning is provisional: verify direction, current, rate, and aiming on rig.
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
const uint16_t FIRE_DWELL_MS=2000, NO_IR_AT_CENTRE_RESCAN_MS=3500;
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
bool calibrated=false,laserHigh=false,irStuckReported=false;
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

// Recorder queue snapshots control events; EEPROM writes are nonblocking.
const uint8_t EV_STATE=1,EV_ARC=2,EV_PULSE=3,EV_DAMP=4,EV_LASER_ON=5,
 EV_LASER_OFF=6,EV_FAULT=9,EV_END=10,EV_LOCK_LOST=12,EV_COMPLETE=14,
 EV_RECOVER=15,EV_SEARCH_WRONG=16,EV_GYRO_DETAIL=17,EV_SCAN_REJECT=18,
 EV_SPEED_ON=19,EV_SPEED_OFF=20,EV_WINDOW=21;
bool recording=false;
uint8_t saved=0,qHead=0,qTail=0,qSize=0,writeByte=0;
uint8_t stateEvents=0;
uint8_t eventQueue[8][12];
uint16_t dropped=0;
uint32_t recordStarted=0;
uint8_t summarySaved=0,arcEvents=0,laserEvents=0;
uint32_t windowAt=0,windowConfirmAt=0;
uint16_t windowBestMs=0,blockCount[5]={0,0,0,0,0};
float windowPeakRate=0,windowPeakPointingRate=0;
uint8_t windowFlags=0;
void resetWindow(uint32_t now);
void recordEvent(uint8_t event);

void laserOff(){
  const bool wasOn=laserHigh;digitalWrite(PIN_LASER,LOW);laserHigh=false;
  if(wasOn)recordEvent(EV_LASER_OFF);
}
void laserOn(){
  // Hardware command; never log/flush synchronously before enabling laser.
  digitalWrite(PIN_LASER,HIGH);
  laserHigh=true;
  recordEvent(EV_LASER_ON);
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
    startupProgressAt=enteredMs;startupProgressYaw=yaw;}
  if(s!=AIM && s!=FINE && s!=FIRE){trackingProgressAt=0;headroomAt=0;holdBias=0;}
  holdBiasAt=enteredMs;
  finePulseActive=false;fineBrakeAt=enteredMs;
  if(s!=AIM && s!=FINE)stableSince=0;
  recordEvent(EV_STATE);
}
void safeStop(uint8_t c){
  if(state==STOP)return;
  stopCode=c;laserOff();motorOff();enter(STOP);recordEvent(EV_FAULT);
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
  laserOff();arcValid=false;slowArcStarted=false;noIrAt=0;stableSince=0;
  if(++scanFailCount>=3){scanFailCount=0;controlledSearchRecovery(true);}
  else enter(SEEK);
}
void restartSearch(bool reverseDirection){
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
  controlledSearchRecovery(false);
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
  if(bad){scanRejectReason=bad;recordEvent(EV_SCAN_REJECT);rescanOrSeek();}
  else{
    scanFailCount=0;
    arcCentre=(arcEntry+arcExit)*0.5f;arcValid=true;recordEvent(EV_ARC);
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

bool maintainLock(uint32_t now){
  const bool fresh=irRecent(now);
  if(fresh)lastLockIrAt=now;
  const bool shortGap=!irStuckReported && ir.active && lastLockIrAt &&
      timeElapsed(now,lastLockIrAt,0) && now-lastLockIrAt<=FIRE_IR_HOLD_MS;
  if(!calibrated || !arcValid || !(fresh||shortGap) ||
      absf(arcCentre-yaw)>fireTolerance()+FIRE_HOLD_EXTRA_DEG ||
      absf(wheelRPM)>MAX_FIRE_WHEEL_RPM){rateViolationAt=0;return false;}
  const float speed=absf(yawRateFiltered);
  if(speed>=HOLD_RATE_HARD_DPS){rateViolationAt=0;return false;}
  if(speed<=FIRE_HOLD_RATE_DPS){rateViolationAt=0;return true;}
  if(!rateViolationAt)rateViolationAt=now;
  return !timeElapsed(now,rateViolationAt,HOLD_RATE_CONFIRM_MS);
}


bool fireCentred(uint32_t now){
  return calibrated && arcValid && pointingRateReady && irRecent(now) &&
    absf(arcCentre-yaw)<=fireTolerance() &&
    absf(pointingRate)<=FIRE_RATE_TOL_DPS &&
    absf(yawRateFiltered)<HOLD_RATE_HARD_DPS &&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}
bool updateLaserArming(uint32_t now){
  if(!fireCentred(now)){
    if(stableSince)recordEvent(EV_LOCK_LOST);
    stableSince=0;return false;
  }
  if(!stableSince)stableSince=now;
  if(timeElapsed(now,stableSince,FIRE_CONFIRM_MS)){
    lastLockIrAt=now;
    laserOn();
    enter(FIRE);
    return true;
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
          if(progress<STARTUP_MIN_PROGRESS_DEG){recordEvent(EV_SEARCH_WRONG);recoverStall();break;}
        }
      }else{startupProgressAt=now;startupProgressYaw=yaw;}
      if(searchSpeedActive||searchSpeedRelease){seekStallAt=0;break;}
      const float progress=searchDirectionWatchArmed?
        yawRateFiltered*BODY_SIGN*seekDirection:absf(yawRateFiltered);
      if(progress<1.5f){
        if(!seekStallAt)seekStallAt=now;
        if(timeElapsed(now,seekStallAt,700))
          seekLevel=100+50*clampf((now-seekStallAt-700UL)/1500.0f,0,1);
        if(timeElapsed(now,seekStallAt,2200))recoverStall();
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
    case FIRE:
      if(!maintainLock(now)){laserOff();enter(FINE);}
      else if(timeElapsed(now,fireStartedAt,FIRE_DWELL_MS)){
        recordEvent(EV_COMPLETE);laserOff();completedTargetDeparture();
      }
      break;
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

// 16-byte header + 84 records x 12 bytes = 1024 bytes.
// Last 8 slots reserved for laser/fault/end. Only first 8 pulse pairs saved.

void resetWindow(uint32_t now){
  windowAt=now;windowBestMs=0;windowPeakRate=0;windowPeakPointingRate=0;windowFlags=0;
  for(uint8_t i=0;i<5;++i)blockCount[i]=0;
  // Preserve a still-running confirmation interval across window boundaries.
}
void recordEvent(uint8_t event){
  if(!recording)return;
  const bool summary=event==EV_WINDOW;
  const bool laser=event==EV_LASER_ON||event==EV_LASER_OFF;
  const bool critical=event==EV_COMPLETE||event==EV_FAULT||event==EV_GYRO_DETAIL||event==EV_END;
  const bool phase=event==EV_STATE;
  const bool recovery=event==EV_SEARCH_WRONG||event==EV_SCAN_REJECT||event==EV_RECOVER;
  if(!summary && !laser && !phase && !recovery && event!=EV_ARC && !critical)return;
  if(phase && stateEvents>=12)return;
  if(laser && laserEvents>=12)return;
  if(event==EV_ARC && arcEvents>=4)return;
  uint8_t queued=0;
  for(uint8_t n=0;n<qSize;++n)
    if((eventQueue[(qHead+n)%8][10]==EV_WINDOW)==summary)++queued;
  if(qSize==8 || (summary?summarySaved:saved)+queued >=
      (summary?60:(critical?24:16))){++dropped;return;}
  if(laser)++laserEvents;
  if(event==EV_ARC)++arcEvents;
  if(phase)++stateEvents;
  uint8_t *b=eventQueue[qTail];
  const uint16_t t=(uint16_t)min((uint32_t)65535,(uint32_t)((millis()-recordStarted)/10UL));
  const int16_t error=(int16_t)clampf(event==EV_GYRO_DETAIL?gyroFaultDtMs:
      (event==EV_ARC?arcCentre*10:(arcValid?arcCentre-yaw:0)*100),-32768,32767);
  const uint16_t packedRates=(uint8_t)clampf(windowPeakRate*2,0,255)|
      ((uint16_t)(uint8_t)clampf(windowPeakPointingRate*10,0,255)<<8);
  const int16_t rate=summary?(int16_t)packedRates:
      (int16_t)clampf(event==EV_GYRO_DETAIL?gyroFaultRawDps*100:
      (event==EV_ARC?arcWidth*100:yawRateFiltered*100),-32768,32767);
  const int16_t wheel=(int16_t)clampf(event==EV_GYRO_DETAIL?gyroFaultAccelG*1000:
      (summary?(arcValid?arcCentre*10:0):wheelRPM*10),-32768,32767);
  b[0]=t;b[1]=t>>8;b[2]=error;b[3]=error>>8;b[4]=rate;b[5]=rate>>8;
  b[6]=wheel;b[7]=wheel>>8;
  b[8]=summary?(uint8_t)min((uint16_t)127,(uint16_t)(windowBestMs/2)):
      (uint8_t)(int8_t)(pwmApplied/2);
  b[9]=state;b[10]=event;
  b[11]=(irRecent(millis())?1:0)|(arcValid?2:0)|(laserHigh?4:0)|
      ((event==EV_FAULT?stopCode:0)<<4);
  if(summary){
    uint8_t worst=0;uint16_t count=0;
    for(uint8_t i=0;i<5;++i)if(blockCount[i]>count){count=blockCount[i];worst=i+1;}
    b[9]=(state&15)|(worst<<4);b[11]=windowFlags;
  }
  if(event==EV_GYRO_DETAIL)b[11]=gyroFaultMask;
  qTail=(qTail+1)%8;++qSize;
}
bool writeEEPROMByte(uint16_t address,uint8_t value){
  if(EECR&_BV(EEPE))return false;
  if(EEPROM.read(address)==value)return true;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){EEAR=address;EEDR=value;EECR|=_BV(EEMPE);EECR|=_BV(EEPE);}
  return true;
}
void recorderService(){
  if(!qSize || finePulseActive)return;
  const bool summary=eventQueue[qHead][10]==EV_WINDOW;
  const uint8_t count=summary?summarySaved:saved;
  const uint16_t slot=summary?count:60+count;
  if(writeByte<12){
    if(writeEEPROMByte(16+slot*12+writeByte,eventQueue[qHead][writeByte]))++writeByte;
  }else if(writeEEPROMByte(summary?4:2,count+1)){
    if(summary)++summarySaved;else ++saved;
    writeByte=0;qHead=(qHead+1)%8;--qSize;
  }
}
void recorderInit(){
  if(EEPROM.read(0)!=0xC7 || EEPROM.read(1)!=32){
    EEPROM.update(0,0);EEPROM.update(1,32);EEPROM.update(2,0);
    EEPROM.update(4,0);EEPROM.update(3,0);EEPROM.update(0,0xC7);
  }
  saved=EEPROM.read(2);summarySaved=EEPROM.read(4);
  if(saved>24)saved=0;
  if(summarySaved>60)summarySaved=0;
  if(EEPROM.read(3)==0xA1){
    EEPROM.update(3,0);EEPROM.update(2,0);EEPROM.update(4,0);
    saved=summarySaved=0;recording=true;recordStarted=millis();resetWindow(recordStarted);
    Serial.println(F("RECORDING,60_windows_plus_24_events"));
  }else{
    state=STOP;motorOff();laserOff();Serial.println(F("IDLE,A=arm_next_boot,D=dump"));
  }
}
int16_t readSigned(uint16_t a){return (int16_t)(EEPROM.read(a)|((uint16_t)EEPROM.read(a+1)<<8));}
void recorderCommands(){
  if(!Serial.available())return;
  const char c=Serial.read();if(c!='a'&&c!='A'&&c!='d'&&c!='D')return;
  if(recording && (c=='d'||c=='D'))recordEvent(EV_WINDOW);
  recording=false;laserOff();motorOff();state=STOP;finePulseActive=false;
  while(qSize || (EECR&_BV(EEPE)))recorderService();
  if(c=='a'||c=='A'){
    EEPROM.update(3,0xA1);Serial.println(F("ARMED_NEXT_BOOT,power_off_then_battery_on"));return;
  }
  EEPROM.update(3,0);
  Serial.println(F("EVENT_LOG_BEGIN,V34"));
  Serial.println(F("time_ms,event,state_or_block,error_cdeg,rate_cdps,rpm_or_centre,pwm_or_confirm,flags"));
  uint8_t si=0,ei=0;
  while(si<summarySaved || ei<saved){
    const uint16_t sa=16+(uint16_t)si*12,ea=16+(uint16_t)(60+ei)*12;
    const bool takeSummary=si<summarySaved && (ei>=saved ||
      (uint16_t)readSigned(sa)<=(uint16_t)readSigned(ea));
    const uint16_t a=takeSummary?sa:ea;
    if(takeSummary)++si;else ++ei;
    Serial.print((uint32_t)(uint16_t)readSigned(a)*10UL);
    Serial.print(',');Serial.print(EEPROM.read(a+10));
    Serial.print(',');Serial.print(EEPROM.read(a+9));
    for(uint8_t j=2;j<=6;j+=2){Serial.print(',');Serial.print(readSigned(a+j));}
    Serial.print(',');Serial.print((int16_t)(int8_t)EEPROM.read(a+8)*2);
    Serial.print(',');Serial.println(EEPROM.read(a+11));
  }
  Serial.print(F("EVENT_LOG_END,windows="));Serial.print(summarySaved);
  Serial.print(F(",events="));Serial.println(saved);
}
void recorderObserve(){
  if(!recording)return;
  const uint32_t now=millis();
  const bool tracking=state==AIM||state==FINE||state==FIRE;
  if(tracking){
    const bool ok[5]={irRecent(now),arcValid,absf(arcCentre-yaw)<=fireTolerance(),
      pointingRateReady && absf(pointingRate)<=FIRE_RATE_TOL_DPS &&
        absf(yawRateFiltered)<HOLD_RATE_HARD_DPS,absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM};
    const uint8_t bits[5]={1,2,8,16,32};bool all=true;
    for(uint8_t i=0;i<5;++i)if(!ok[i]){
      all=false;if(blockCount[i]!=65535)++blockCount[i];windowFlags|=bits[i];
    }
    if(all){
      windowFlags|=64;if(!windowConfirmAt)windowConfirmAt=now;
      windowBestMs=max(windowBestMs,(uint16_t)min((uint32_t)254,now-windowConfirmAt));
    }else windowConfirmAt=0;
    windowPeakRate=fmaxf(windowPeakRate,absf(yawRateFiltered));
    windowPeakPointingRate=fmaxf(windowPeakPointingRate,absf(pointingRate));
  }else windowConfirmAt=0;
  if(laserHigh)windowFlags|=4;
  if(timeElapsed(now,windowAt,5000)){
    const uint8_t before=qSize;recordEvent(EV_WINDOW);
    if(qSize!=before)resetWindow(now);
  }
  if(now-recordStarted>=300000UL){
    if(now>windowAt)recordEvent(EV_WINDOW);
    recordEvent(EV_END);recording=false;
  }
}

void setup(){
  pinMode(PIN_IN1,OUTPUT);pinMode(PIN_IN2,OUTPUT);pinMode(PIN_LASER,OUTPUT);
  pinMode(PIN_LED,OUTPUT);pinMode(PIN_ENC_A,INPUT_PULLUP);
  pinMode(PIN_ENC_B,INPUT_PULLUP);pinMode(PIN_IR,INPUT_PULLUP);
  motorOff();laserOff();Serial.begin(115200);
  Wire.begin();Wire.setClock(100000UL);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000UL,true);   
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
  Serial.println(F("MQ SENTINEL V34 | A=ARM_NEXT_BOOT, D=DUMP"));
  recorderInit();

}
void loop(){
  const uint32_t now=millis();

  gyroUpdate(now);
  irUpdate(now);
  rpmUpdate(now);
  if(state!=STOP)sequence(now);
  motorUpdate(now);
  digitalWrite(PIN_LED,state==STOP?((now/125)&1):laserHigh);
#if LIVE_TELEMETRY
  telemetry(now);
#endif
  recorderObserve();recorderService();recorderCommands();

}
