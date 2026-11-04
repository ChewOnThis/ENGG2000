/* HM 3.2: body-rate scan braking and wheel-headroom recovery.
   Preserve HM 3.1 IR entry and HM 2.5 fine alignment / firing control.
   EEPROM format 23 retained. Continuous USB CSV disabled to fit Nano.
*/
#define LIVE_CSV 0
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

const uint8_t PIN_IN1=5, PIN_IN2=6, PIN_LASER=4;
const uint8_t PIN_ENC_A=3, PIN_ENC_B=2, PIN_IR=11, PIN_LED=13, MPU=0x68;
#define LASER_ENABLED 1         
const int8_t BODY_SIGN=-1;     
const int8_t ENCODER_SIGN=-1;  
const uint8_t YAW_AXIS=0;      

const float GYRO_X_BIAS=24.3267f;                    

#define LASER_CAL_SELFTEST_ENABLED 0
const uint16_t LASER_CAL_SELFTEST_MS=350;

#define AXIS_CHECK_ENABLED 1

const uint16_t CAL_SAMPLES=1000, GYRO_INTERVAL_MS=5;
const float GYRO_COUNTS_PER_DPS=131.0f, ACC_COUNTS_PER_G=16384.0f;
const float CAL_MAX_SD_DPS=0.30f, CAL_ACC_TOL=0.15f;
const float RATE_LPF=0.16f;
const uint32_t BOOT_WAIT_MS=2000UL;
const uint32_t IR_MIN_PULSE_US=100UL, IR_MAX_PULSE_US=35000UL;
const uint8_t IR_PULSES_REQUIRED=2;

const uint16_t IR_GROUP_MS=180, IR_LOSS_MS=210, IR_RECENT_MS=260;
const uint16_t IR_MAX_CONTINUOUS_LOW_MS=9000;
const uint16_t IR_LOCK_LOW_MIN_MS=7;
const uint16_t IR_LOW_STEADY_MS=9, IR_CLEAR_MS=220;
const uint16_t IR_POLL_MS=5, IR_PATH_MS=30;
const uint16_t ARC_MIN_MS=65, ARC_MAX_MS=8500;
const float ARC_MIN_DEG=0.5f, ARC_MAX_DEG=85.0f;
const float ARC_MIN_EFFICIENCY=0.48f, ARC_MAX_BACKTRACK_DEG=2.5f;
const float ARC_MAX_SWEEP_RATE=85.0f;

const int8_t SEEK_MOTOR_SIGN=-1;              
const float MOTOR_PWM_LIMIT=255.0f;           
const float WHEEL_HARD_RPM=250.0f;            

const float FAST_BRAKE_KP=14.0f, FAST_BRAKE_MIN_PWM=30.0f;
const uint16_t FAST_BRAKE_STILL_MS=200, FAST_BRAKE_TIMEOUT_MS=6500;
const float BACKOUT_DPS=3.0f, BACKOUT_PWM_MAX=125.0f;
const float SCAN_BODY_DPS=2.8f, SCAN_PWM_MAX=115.0f;
const float RATE_KP=15.0f, RATE_FRICTION_PWM=30.0f;
const uint16_t PRE_SCAN_STILL_MS=220, PRE_SCAN_TIMEOUT_MS=5500;
const uint16_t ARC_BRAKE_STILL_MS=220, ARC_BRAKE_TIMEOUT_MS=6000;

const uint16_t MOTOR_PERIOD_MS=5, REVERSE_DEADTIME_MS=24;
const uint16_t RPM_SAMPLE_MS=50;

const float UNWIND_PWM_LIMIT=160.0f, UNWIND_RPM_FINISH=18.0f;
const float COUNTS_PER_REV=700.0f;

const uint16_t FAST_BRAKE_PULSE_PERIOD_MS=115, FAST_BRAKE_PULSE_ON_MS=28;
const uint16_t ARC_BRAKE_PULSE_PERIOD_MS=105, ARC_BRAKE_PULSE_ON_MS=24;
const uint16_t AIM_BRAKE_PULSE_PERIOD_MS=115, AIM_BRAKE_PULSE_ON_MS=18;
const uint16_t FINE_BRAKE_PULSE_START_MS=56, FINE_BRAKE_PULSE_MS=22;

const float COARSE_KP_POS=1.3f, COARSE_KP_RATE=6.0f;
const float COARSE_FRICTION_PWM=38.0f, COARSE_MAX_PWM=115.0f;
const float FINE_ENTER_DEG=3.2f, FINE_REENTER_DEG=4.4f;
const uint16_t FINE_PERIOD_MS=145, FINE_PULSE_MS=45;
const float FINE_MAX_PWM=74.0f, FINE_KP_POS=22.0f, FINE_KD_RATE=10.0f;
const float FINE_MIN_ERROR=0.18f;

const int16_t FINE_DRIVE_PWM=105;       
const uint16_t FINE_DRIVE_MIN_MS=5, FINE_DRIVE_MAX_MS=40;
const uint16_t FINE_SETTLE_MIN_MS=180, FINE_STILL_CONFIRM_MS=80;
const uint16_t FINE_SETTLE_RECOVERY_MS=900;
const float FINE_GAIN_DEG_PER_DUTY_MS=0.122f; 
const float FINE_MOVE_FRACTION=0.30f;   
const float FINE_SETTLE_BODY_DPS=1.5f, FINE_SETTLE_WHEEL_RPM=12.0f;
const float FINE_PULSE_RATE_LIMIT_DPS=3.5f;
int16_t finePulsePwm=0;
bool finePulseActive=false;
uint32_t finePulseAt=0, fineBrakeAt=0, fineStillAt=0;
uint16_t finePulseDuration=0;
float finePulseDirection=0;


const float FIRE_ANGLE_TOL_DEG=0.90f, FIRE_RATE_TOL_DPS=3.20f;
const float FIRE_ANGLE_MIN_TOL_DEG=0.90f;
const float FIRE_HOLD_EXTRA_DEG=0.40f, FIRE_HOLD_RATE_DPS=4.3f;
const uint16_t FIRE_CONFIRM_MS=85, FIRE_IR_HOLD_MS=90;
const uint16_t NO_IR_AT_CENTRE_RESCAN_MS=3500;
const float MAX_FIRE_WHEEL_RPM=185.0f;
const uint16_t PRINT_INTERVAL_MS=10;

enum State:uint8_t {
  BOOT, CAL, CLEAR, SEEK, FAST_BRAKE, BACKOUT, SETTLE,
  SLOW_SCAN, ARC_BRAKE, AIM, FINE, FIRE, UNWIND, STOP
};
// V23: angle-error integral, in body-torque PWM units.
const float HOLD_KI=0.8f, HOLD_BIAS_LIMIT=20.0f;
float holdBias=0;
uint32_t holdBiasAt=0,lastBiasEvent=0;
uint8_t biasEvents=0;
// V24 recovery/firing timers are not safety-stop conditions.
int8_t seekDirection=SEEK_MOTOR_SIGN;
float seekLevel=100.0f;
const float SEEK_CRUISE_DPS=30.0f, TRAVEL_RATE_KI=3.0f;
float travelTrim=0;
uint32_t travelTrimAt=0;
uint8_t travelMode=0;
uint32_t plateauAt=0;
float plateauRPM=0;
uint32_t seekStallAt=0,missingTargetAt=0,rateViolationAt=0;
uint32_t trackingProgressAt=0,headroomAt=0,fireStartedAt=0;
float bestTrackingError=0;
const uint16_t HOLD_RATE_CONFIRM_MS=40,FIRE_DWELL_MS=2000;
const float HOLD_RATE_HARD_DPS=8.0f;
void restartSearch(bool reverseDirection);
// V26: isolated controlled departure after successful dwell only.
bool controlledDeparture=false;
float departureApplied=0,departureReference=0;
uint32_t departureAt=0,departureStillAt=0;
const float DEPARTURE_PWM_PER_SEC=250.0f,DEPARTURE_ACCEL_DPS2=15.0f;
bool searchDirectionWatchArmed=false;
uint32_t wrongSearchAt=0;
uint32_t startupProgressAt=0;
float startupProgressYaw=0;
const uint16_t STARTUP_PROGRESS_WINDOW_MS=5000;
const float STARTUP_MIN_PROGRESS_DEG=4.0f;
uint8_t gyroFaultMask=0,scanRejectReason=0;
uint16_t gyroFaultDtMs=0;
float gyroFaultRawDps=0,gyroFaultAccelG=0;
uint8_t scanRejectEvents=0;

const uint16_t WRONG_SEARCH_CONFIRM_MS=400;
const float WRONG_SEARCH_DPS=2.0f;
uint8_t wrongSearchEvents=0;
void controlledSearchRecovery(bool reverseDirection);
// V30 speed guard affects travel/search states only.
const float SEARCH_SPEED_ENTER_DPS=60.0f,SEARCH_SPEED_EXIT_DPS=45.0f;
const float SEARCH_SPEED_TARGET_DPS=40.0f,SEARCH_LIMIT_SLEW=500.0f;
bool searchSpeedActive=false,searchSpeedRelease=false;
uint32_t searchSpeedSafeAt=0;
uint8_t speedEvents=0;
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
// Used only for laser ENTRY qualification, never motor feedback.
const float POINTING_RATE_TAU_SEC=0.150f;
float pointingRate=0;
bool pointingRateReady=false;
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
// HM 2.3: 40 rolling diagnostic samples + 19 retained events, 17 bytes each.
const uint8_t LOG_FORMAT=23, LOG_BYTES=17, LOG_SAMPLES=40, LOG_EVENTS=19;
const uint16_t LOG_BASE=16;
static_assert(LOG_BASE+(LOG_SAMPLES+LOG_EVENTS)*LOG_BYTES<=E2END+1,
              "EEPROM layout exceeds Nano storage");
const uint8_t EV_STATE=1,EV_ARC=2,EV_PULSE=3,EV_DAMP=4,EV_LASER_ON=5,
EV_LASER_OFF=6,EV_SAT_ON=7,EV_SAT_OFF=8,EV_FAULT=9,EV_END=10,EV_GATE=11,
EV_LOCK_LOST=12,EV_BIAS=13,EV_COMPLETE=14,EV_RECOVER=15,EV_SEARCH_WRONG=16,
EV_GYRO_DETAIL=17,EV_SCAN_REJECT=18,EV_SPEED_ON=19,EV_SPEED_OFF=20,
EV_WINDOW=21,EV_AIM_WINDOW=22;
bool recording=false;
uint8_t saved=0,summarySaved=0,sampleNext=0,qHead=0,qTail=0,qSize=0,writeByte=0;
uint8_t eventQueue[4][LOG_BYTES];
uint16_t dropped=0;
uint32_t recordStarted=0,lastSampleAt=0,lastPulseLogAt=0;
bool logPulsePair=false;
uint8_t speedLogEvents=0;
uint16_t longestGateMs=0;
void recordEvent(uint8_t event);
void laserOff(){
  const bool wasOn=laserHigh;digitalWrite(PIN_LASER,LOW);laserHigh=false;
  if(wasOn)recordEvent(EV_LASER_OFF);
}
void laserOn(){
#if LASER_ENABLED

  digitalWrite(PIN_LASER,HIGH);
  laserHigh=true;recordEvent(EV_LASER_ON);
  Serial.println(F("LASER_D4_HIGH,OUTPUT_COMMAND_CONFIRMED"));
#else
  Serial.println(F("LASER_DISABLED_IN_BUILD"));
#endif
}
bool timeElapsed(uint32_t now,uint32_t since,uint32_t waitMs); 

// DRV8874 PWM mode (PMODE HIGH): both inputs HIGH electrically brake the
// WHEEL. That does not imply zero BODY rate. LOW/LOW would coast instead.
// Travel/body-rate control uses wheel feedforward plus signed torque feedback.
void motorHardware(int16_t p){
  if(p>255)p=255; else if(p<-255)p=-255;
  if(p>0){analogWrite(PIN_IN1,255);analogWrite(PIN_IN2,255-p);motorMode=MOTOR_DRIVE;}
  else if(p<0){analogWrite(PIN_IN2,255);analogWrite(PIN_IN1,255+p);motorMode=MOTOR_DRIVE;}
  else {analogWrite(PIN_IN1,255);analogWrite(PIN_IN2,255);motorMode=MOTOR_IDLE;}
}

// Legacy name: motorOff selects the electrical wheel-brake state, not coast.
void motorOff(){pwmGoal=pwmActual=0;pwmApplied=0;motorHardware(0);}

void serviceLaserSelfTest(uint32_t now){
  if(laserSelfTestActive &&
     (state!=CAL || timeElapsed(now,laserSelfTestStarted,LASER_CAL_SELFTEST_MS))){
    laserOff();laserSelfTestActive=false;
    Serial.println(F("LASER_CAL_TEST_OFF,D4_LOW"));
  }
}
void enter(State s){
  plateauAt=0;
  if(s!=CLEAR && s!=SEEK && s!=SLOW_SCAN && s!=AIM)resetTravelSpeed();
  state=s;enteredMs=millis();
  rateViolationAt=0;
  if(s!=CLEAR && s!=SEEK && s!=SLOW_SCAN)wrongSearchAt=0;
  if(s==FIRE)fireStartedAt=enteredMs;
  if(s==SEEK){
    seekStallAt=0;seekLevel=100.0f;
    startupProgressAt=enteredMs;startupProgressYaw=yaw;
  }
  // Preserve progress tracking across AIM/FINE/FIRE instead of resetting
  // it on every oscillation across the state boundary.
  if(s!=AIM && s!=FINE && s!=FIRE){
    missingTargetAt=0;trackingProgressAt=0;headroomAt=0;
  }
  if(s!=AIM && s!=FINE && s!=FIRE)holdBias=0;
  holdBiasAt=enteredMs;
  recordEvent(EV_STATE);

  finePulseActive=false;fineBrakeAt=enteredMs;fineStillAt=0;

  if(s!=AIM && s!=FINE)stableSince=0;
  if(s==SEEK)lastEncoderEdgeAt=enteredMs;
}
void safeStop(uint8_t c){
  if(state==STOP)return;
  stopCode=c;laserSelfTestActive=false;laserOff();motorOff();enter(STOP);
  recordEvent(EV_FAULT);
  Serial.print(F("SAFETY_STOP,code="));Serial.println(c);
}
void beginClear(){
  laserOff();arcValid=false;slowArcStarted=false;
  clearSince=0;slowYawSince=0;phaseStillAt=0;
  enter(CLEAR);
}
void encoderRise(){encoderTicks+=digitalRead(PIN_ENC_B)?-1:+1; ++encoderRisingEdges;}

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
  dps=v[4+YAW_AXIS]/GYRO_COUNTS_PER_DPS;   
  return true;
}
void startWithFixedBias(){
  motorOff();laserOff();laserSelfTestActive=false;
  gyroBias=GYRO_X_BIAS/GYRO_COUNTS_PER_DPS;
  calibrated=true;fallbackBias=false;
  yaw=yawRate=yawRateFiltered=0;pointingRate=0;pointingRateReady=false;
  lastGyroUs=micros();lastGyroMs=millis();
  Serial.println(F("FIXED_X_BIAS"));
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
  laserOff();
  enter(SLOW_SCAN);
  initialiseArc(entry,ir.groupStart?ir.groupStart:millis());
}

void initialiseArc(float entry,uint32_t ms){
  arcEntry=entry;arcStarted=ms;
  arcExit=entry;arcMin=arcMax=entry;arcPath=arcMaxRate=0;
  lastArcPathYaw=entry;lastArcPathMs=millis();
  slowArcStarted=true;
  Serial.println(F("SLOW_ARC_ENTRY"));
}
void rescanOrSeek(){controlledSearchRecovery(false);}
void restartSearch(bool reverseDirection){
  controlledDeparture=false;
  laserOff();recordEvent(EV_RECOVER);
  arcValid=false;slowArcStarted=false;stableSince=0;noIrAt=0;
  if(reverseDirection)seekDirection=-seekDirection;
  seekLevel=100.0f;seekStallAt=0;clearSince=0;
  enter(UNWIND); // brake for 650 ms, then actively leave any old IR window
  Serial.println(F("RECOVER,brake_then_continuous_seek"));
}


void controlledSearchRecovery(bool reverseDirection){
  const float previousCommand=pwmApplied;
  restartSearch(reverseDirection);
  controlledDeparture=true;
  departureApplied=previousCommand;
  departureReference=yawRateFiltered;
  departureAt=millis();departureStillAt=0;
  Serial.println(F("CONTROLLED_SEARCH_TRANSITION"));
}
void completedTargetDeparture(){
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
  if(bad){
    ++rejectedArcs;
    scanRejectReason=bad;recordEvent(EV_SCAN_REJECT);
    Serial.print(F("ARC_BAD,reason="));Serial.print(bad);
    Serial.println();
    rescanOrSeek();
  }else{
    ++validArcs;arcCentre=(arcEntry+arcExit)*0.5f;arcValid=true;recordEvent(EV_ARC);
    phaseStillAt=0;
    Serial.println(F("ARC_OK"));
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

  // Only measure the path while actual signal evidence is being received.
  // During the 210 ms loss-confirmation gap, leave these statistics frozen.
  if(state==SLOW_SCAN && slowArcStarted && (newPulses || steadyLow)){
    arcMin=fminf(arcMin,yaw);arcMax=fmaxf(arcMax,yaw);
    arcMaxRate=fmaxf(arcMaxRate,absf(yawRate));
    arcPath+=absf(yaw-lastArcPathYaw);lastArcPathYaw=yaw;
  }
  if(stuckLow && ir.active){
    ir.active=false;ir.groupPulses=0;ir.groupStart=0;
    Serial.println(F("IR_INVALID,continuous_LOW_timeout"));
    if(state==SLOW_SCAN && slowArcStarted)rescanOrSeek();
  }
  if(!ir.active){
    if(!stuckLow && ((ir.groupPulses>=IR_PULSES_REQUIRED &&
         now-ir.groupStart<=IR_GROUP_MS && ir.groupStart!=0)||steadyLow)){
      ir.active=true;++irOnCount;
      Serial.print(F("IR_ON,pulses="));Serial.println(ir.groupPulses);
      if(state==SEEK)firstIrEncounter(ir.firstYaw);
      else if(state==SLOW_SCAN && !slowArcStarted)
        initialiseArc(ir.firstYaw,ir.groupStart);
    }
  }else if(!low && timeElapsed(now,ir.lastSignal,IR_LOSS_MS)){
    ir.active=false;++irOffCount;
    Serial.println(F("IR_OFF"));
    ir.groupPulses=0;ir.groupStart=0;
    if(state==SLOW_SCAN && slowArcStarted)finishArc(ir.lastYaw,ir.lastSignal);
  }
  if(!ir.active&&ir.groupStart&&now-ir.groupStart>IR_GROUP_MS){
    ir.groupStart=0;ir.groupPulses=0;
  }
}

void rpmUpdate(uint32_t now){
  if(now-lastRPMMs<RPM_SAMPLE_MS)return;
  int32_t ticks;uint32_t edges;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ticks=encoderTicks;edges=encoderRisingEdges;}
  if(edges!=previousEncoderEdges){lastEncoderEdgeAt=now;previousEncoderEdges=edges;}
  if(lastRPMMs){
    rpmInstant=(ticks-prevTicks)*60000.0f/(COUNTS_PER_REV*(now-lastRPMMs));
    wheelRPM+=0.35f*(rpmInstant-wheelRPM);
    if(absf(wheelRPM)>WHEEL_HARD_RPM || absf(rpmInstant)>WHEEL_HARD_RPM+8.0f)safeStop(2);
  }
  lastRPMMs=now;prevTicks=ticks;
}

float ratePWM(float targetDps,float maxCorrection){
  const float ff=ENCODER_SIGN*wheelRPM*255.0f/157.0f;
  const float correction=clampf(4.0f*(targetDps-yawRateFiltered),
      -maxCorrection,maxCorrection);
  return clampf(ff+BODY_SIGN*correction,-MOTOR_PWM_LIMIT,MOTOR_PWM_LIMIT);
}

void resetTravelSpeed(){travelTrim=0;travelMode=0;travelTrimAt=millis();}
float travelRatePWM(float target,float gain,float limit,float trimLimit,uint8_t mode){
  const uint32_t now=millis();
  const uint8_t key=mode+(target<0?4:0);
  float dt=clampf((uint32_t)(now-travelTrimAt)*0.001f,0,0.02f);
  if(travelMode!=key){travelTrim=0;dt=0;travelMode=key;}
  travelTrimAt=now;
  // Fade stored load compensation before the fine-control handoff.
  travelTrim=clampf(travelTrim,-trimLimit,trimLimit);
  const float error=target-yawRateFiltered;
  const float base=gain*error;
  const float candidate=clampf(travelTrim+TRAVEL_RATE_KI*error*dt,-trimLimit,trimLimit);
  const float delta=candidate-travelTrim, inner=base+candidate;
  const float ff=ENCODER_SIGN*wheelRPM*255.0f/157.0f;
  const float voltage=ff+BODY_SIGN*clampf(inner,-limit,limit);
  // Do not wind up against output limits or a deliberately ramped/guarded
  // search transition. Compensation may still unwind out of saturation.
  if(!controlledDeparture && !searchSpeedActive && !searchSpeedRelease &&
     (absf(inner)<=limit || delta*inner<0) &&
     (absf(voltage)<=255 || BODY_SIGN*delta*voltage<0))travelTrim=candidate;
  return clampf(ff+BODY_SIGN*clampf(base+travelTrim,-limit,limit),-255,255);
}
float cruisePWM(){
  if((state==CLEAR || state==SEEK || state==SLOW_SCAN) && !controlledDeparture)
    return travelRatePWM(BODY_SIGN*seekDirection*SEEK_CRUISE_DPS,4.0f,150.0f,60.0f,1);
  return seekDirection*seekLevel; // Preserve the controlled-departure ramp handoff.
}

float bodyBrakePWM(float maxPwm){return ratePWM(0,maxPwm);}
float aimSpeed(float distance){
  return distance>20.0f?6.0f:
    (distance>8.0f?0.25f*distance+1.0f:fminf(3.0f,0.8f*distance));
}
float coarsePWM(){
  const float err=arcCentre-yaw, distance=absf(err);
  const float speed=aimSpeed(distance);
  const float limit=distance>20.0f?125.0f:(distance>8.0f?100.0f:65.0f);
  const float gain=fmaxf(4.0f,speed+1.0f);
  if(distance>FINE_ENTER_DEG){
    const float trimLimit=60.0f*clampf((distance-FINE_ENTER_DEG)/4.8f,0,1);
    return travelRatePWM(signf(err)*speed,gain,limit,trimLimit,2);
  }
  resetTravelSpeed(); // Original coarse response at/inside 3.2 degrees.
  const float correction=clampf(gain*(signf(err)*speed-yawRateFiltered),-limit,limit);
  const float ff=ENCODER_SIGN*wheelRPM*255.0f/157.0f;
  return clampf(ff+BODY_SIGN*correction,-255,255);
}

float fineHoldPWM(){
  const float requested=clampf(0.6f*(arcCentre-yaw),-1.2f,1.2f);
  const float correction=clampf(4.0f*(requested-yawRateFiltered)+holdBias,-65,65);
  const float ff=ENCODER_SIGN*wheelRPM*255.0f/157.0f;
  return clampf(ff+BODY_SIGN*correction,-255,255);
}
void updateHoldBias(uint32_t now){
  const float dt=clampf((uint32_t)(now-holdBiasAt)*0.001f,0,0.02f);
  holdBiasAt=now;
  const float err=arcCentre-yaw;
  // Learn in near-target hold only, not during a pulse or a rapid pass.
  if(finePulseActive || !arcValid || absf(err)>FINE_REENTER_DEG ||
      absf(yawRateFiltered)>8.0f)return;
  const float candidate=clampf(holdBias+HOLD_KI*err*dt,-HOLD_BIAS_LIMIT,HOLD_BIAS_LIMIT);
  const float delta=candidate-holdBias;
  const float requested=clampf(0.6f*err,-1.2f,1.2f);
  const float base=4.0f*(requested-yawRateFiltered);
  const float inner=base+candidate;
  const float ff=ENCODER_SIGN*wheelRPM*255.0f/157.0f;
  const float raw=ff+BODY_SIGN*clampf(inner,-65,65);
  const bool innerOK=absf(inner)<=65 || delta*inner<0;
  const bool voltageOK=absf(raw)<=255 || BODY_SIGN*delta*raw<0;
  if(innerOK && voltageOK)holdBias=candidate;
}
float finePWM(uint32_t now){
  updateHoldBias(now);
  const float err=arcCentre-yaw;
  const float ff=ENCODER_SIGN*wheelRPM*255.0f/157.0f;
  if(finePulseActive){
    if(timeElapsed(now,finePulseAt,finePulseDuration) ||
       err*finePulseDirection<=0 || absf(err)<=FINE_MIN_ERROR ||
       absf(yawRateFiltered)>=FINE_PULSE_RATE_LIMIT_DPS){
      finePulseActive=false;fineBrakeAt=now;
      recordEvent(EV_DAMP);
      return fineHoldPWM();
    }
    return clampf(ff+BODY_SIGN*(finePulseDirection*finePulsePwm+holdBias),-255,255);
  }
  // Maintain wheel momentum while damping body rate; zero-voltage wheel
  // braking between every correction was repeatedly disturbing the body.
  // Laser permission is not the centering target. Keep fine corrections
  // active outside +/-0.18 degree even if the laser is already allowed on.
  // Within that band, fineHoldPWM still restores position and damps rate.
  if(absf(err)<=FINE_MIN_ERROR)
    return fineHoldPWM();
  if(!timeElapsed(now,fineBrakeAt,180) ||
      absf(yawRateFiltered)>=FINE_PULSE_RATE_LIMIT_DPS)
    return fineHoldPWM();
  finePulsePwm=absf(err)<1.0f?45:65;
  const float duty=finePulsePwm/255.0f;
  const float width=0.20f*absf(err)/(FINE_GAIN_DEG_PER_DUTY_MS*duty);
  finePulseDuration=(uint16_t)clampf(width,5,20);
  finePulseDuration=((finePulseDuration+MOTOR_PERIOD_MS-1)/MOTOR_PERIOD_MS)*MOTOR_PERIOD_MS;
  finePulseAt=now;finePulseDirection=signf(err);finePulseActive=true;
  recordEvent(EV_PULSE);
  return clampf(ff+BODY_SIGN*(finePulseDirection*finePulsePwm+holdBias),-255,255);
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
    const float ff=ENCODER_SIGN*wheelRPM*255.0f/157.0f;
    wanted=clampf(ff+BODY_SIGN*correction,-255,255);
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
  const float motorDt=clampf((uint32_t)(now-lastMotorMs)*0.001f,0,0.02f);
  lastMotorMs=now;
  float command=0;
  if(state==CLEAR || state==SEEK || state==SLOW_SCAN)command=cruisePWM();
  else if(state==AIM)command=coarsePWM();
  else if(state==ARC_BRAKE)
    command=pwmApplied+clampf(ratePWM(0,90.0f)-pwmApplied,-2.0f,2.0f);
  else if(state==FINE || state==FIRE)command=finePWM(now);
  if(controlledDeparture && state==UNWIND){
    departureReference+=clampf(-departureReference,
      -DEPARTURE_ACCEL_DPS2*motorDt,DEPARTURE_ACCEL_DPS2*motorDt);
    command=ratePWM(departureReference,65.0f);
  }
  if(controlledDeparture && (state==UNWIND||state==CLEAR||state==SEEK||state==SLOW_SCAN)){
    departureApplied+=clampf(command-departureApplied,
      -DEPARTURE_PWM_PER_SEC*motorDt,DEPARTURE_PWM_PER_SEC*motorDt);
    command=departureApplied;
    // Complete the handoff only once the ordinary V24 seek command matches.
    if((state==SEEK||state==SLOW_SCAN) && absf(departureApplied-cruisePWM())<1.0f)
      controlledDeparture=false;
  }else if(state!=UNWIND && state!=CLEAR && state!=SEEK && state!=SLOW_SCAN)controlledDeparture=false;
  command=limitSearchPWM(now,command,motorDt);
  if(state==STOP || state==BOOT ||
      (state==UNWIND && !controlledDeparture))command=0;
  pwmApplied=(int16_t)clampf(command,-MOTOR_PWM_LIMIT,MOTOR_PWM_LIMIT);
  motorHardware(pwmApplied);
}

float fireTolerance(){
  
  return clampf(arcWidth*0.42f,FIRE_ANGLE_MIN_TOL_DEG,FIRE_ANGLE_TOL_DEG);
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
  // Accept bounded fast ripple only while tightly centred and the slower
  // rate estimate is low. Angle/IR/wheel checks and >=8 deg/s cutoff remain.
  const bool centredRipple=pointingRateReady &&
    absf(arcCentre-yaw)<=fireTolerance() && absf(pointingRate)<=1.5f;
  if(speed<=FIRE_HOLD_RATE_DPS || centredRipple){rateViolationAt=0;return true;}
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
    if(stableSince){
      longestGateMs=max(longestGateMs,(uint16_t)min((uint32_t)254,(uint32_t)(now-stableSince)));
      recordEvent(EV_LOCK_LOST);
    }
    stableSince=0;return false;
  }
  if(!stableSince)stableSince=now;
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
bool trackingNeedsRecovery(uint32_t now){
  const float error=absf(arcCentre-yaw);
  // No IR is normal outside the cone while returning toward its centre.
  // Do not abandon a progressing AIM merely because it has not re-entered.
  missingTargetAt=0;
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
    pwmApplied*seekDirection>=80;
  const float progress=yawRateFiltered*BODY_SIGN*seekDirection;
  if(!enabled || progress>=-WRONG_SEARCH_DPS){wrongSearchAt=0;return false;}
  if(!wrongSearchAt)wrongSearchAt=now;
  return timeElapsed(now,wrongSearchAt,WRONG_SEARCH_CONFIRM_MS);
}
bool travelSaturated(uint32_t now){
  const float distance=absf(arcCentre-yaw);
  const bool active=state==CLEAR || state==SEEK || state==SLOW_SCAN ||
    state==ARC_BRAKE || (state==AIM && distance>FINE_ENTER_DEG);
  const float target=state==ARC_BRAKE?0:
    (state==AIM?signf(arcCentre-yaw)*aimSpeed(distance):BODY_SIGN*seekDirection*SEEK_CRUISE_DPS);
  const bool rateMiss=state==ARC_BRAKE?absf(pointingRate)>3.0f:
    absf(target-yawRateFiltered)>fmaxf(1.5f,0.25f*absf(target));
  // Look for lost acceleration at high drive, not a fixed maximum RPM.
  if(!active || controlledDeparture || searchSpeedActive || searchSpeedRelease ||
     abs(pwmApplied)<245 || absf(wheelRPM)<40 ||
     pwmApplied*ENCODER_SIGN*wheelRPM<=0 || !rateMiss){plateauAt=0;return false;}
  const float rpm=absf(wheelRPM);
  if(!plateauAt || rpm>plateauRPM+2.0f){plateauAt=now;plateauRPM=rpm;return false;}
  return timeElapsed(now,plateauAt,500);
}
void recoverWheelMomentum(){
  // Choose search drive opposite the wheel's maintaining-voltage polarity.
  // The existing recovery ramps body rate and applied PWM; it does not snap.
  const int8_t unloadDirection=absf(wheelRPM)>=40?
    (wheelRPM>0?-ENCODER_SIGN:ENCODER_SIGN):-seekDirection;
  controlledSearchRecovery(seekDirection!=unloadDirection);
}
void sequence(uint32_t now){
  if(travelSaturated(now)){
    recordEvent(EV_SAT_ON);recoverWheelMomentum();return;
  }
  if(searchMovingBackward(now)){
    recordEvent(EV_SEARCH_WRONG);
    Serial.println(F("SEARCH_BACKWARD,reverse_via_controlled_transition"));
    controlledSearchRecovery(true);return;
  }
  if((state==AIM||state==FINE||state==FIRE) && trackingNeedsRecovery(now)){
    restartSearch(true);return;
  }
  switch(state){
    case BOOT:
      if(timeElapsed(now,enteredMs,BOOT_WAIT_MS))startWithFixedBias();
      break;
    case UNWIND:
      if(controlledDeparture){
        if(absf(yawRateFiltered)<2.0f){
          if(!departureStillAt)departureStillAt=now;
        }else departureStillAt=0;
        if((departureStillAt && timeElapsed(now,departureStillAt,100)) ||
            timeElapsed(now,enteredMs,1500)){clearSince=0;enter(CLEAR);}
      }else if(timeElapsed(now,enteredMs,650)){clearSince=0;enter(CLEAR);}
      break;
    case CLEAR:
      // Drive continuously while clearing the old cone; do not sit at rest
      // waiting for a stationary receiver to stop seeing an active beacon.
      if(timeElapsed(now,enteredMs,3000)){controlledSearchRecovery(true);break;}
      if(!ir.active){
        if(!clearSince)clearSince=now;
        if(timeElapsed(now,clearSince,IR_CLEAR_MS))enter(SEEK);
      }else clearSince=0;
      break;
    case SEEK:{
      if(!searchDirectionWatchArmed && !controlledDeparture &&
         pwmApplied!=0){
        if(timeElapsed(now,startupProgressAt,STARTUP_PROGRESS_WINDOW_MS)){
          const float progress=(yaw-startupProgressYaw)*BODY_SIGN*seekDirection;
          startupProgressAt=now;startupProgressYaw=yaw;
          if(progress<STARTUP_MIN_PROGRESS_DEG){
            recordEvent(EV_SEARCH_WRONG);
            controlledSearchRecovery(true);break;
          }
        }
      }else {startupProgressAt=now;startupProgressYaw=yaw;}
      if(searchSpeedActive||searchSpeedRelease){seekStallAt=0;break;}
      // Preserve V26's initial search; signed-progress recovery starts only
      // after a completed two-second hold, where the regression was observed.
      const float progress=searchDirectionWatchArmed?
        yawRateFiltered*BODY_SIGN*seekDirection:absf(yawRateFiltered);
      if(progress<1.5f){
        if(!seekStallAt)seekStallAt=now;
        if(timeElapsed(now,seekStallAt,700))
          seekLevel=100.0f+50.0f*clampf((now-seekStallAt-700UL)/1500.0f,0,1);
        if(timeElapsed(now,seekStallAt,2200))controlledSearchRecovery(true);
      }else {seekStallAt=0;seekLevel=100;}
      break;
    }
    case SLOW_SCAN:
      if(timeElapsed(now,enteredMs,ARC_MAX_MS+250UL))rescanOrSeek();
      break;
    case ARC_BRAKE:
      if(pointingRateReady && absf(pointingRate)<=3.0f && absf(yawRateFiltered)<8.0f){
        if(!phaseStillAt)phaseStillAt=now;
      }else phaseStillAt=0;
      if(phaseStillAt && timeElapsed(now,phaseStillAt,100))enter(AIM);
      else if(timeElapsed(now,enteredMs,1500))recoverWheelMomentum();
      break;
    case AIM:
      if(updateLaserArming(now))break;
      if(absf(arcCentre-yaw)<=FINE_ENTER_DEG && absf(yawRateFiltered)<=3.0f)
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
      if(!maintainLock(now)){
        laserOff();enter(FINE);Serial.println(F("LASER_OFF,lock_lost"));
      }else if(timeElapsed(now,fireStartedAt,FIRE_DWELL_MS)){
        recordEvent(EV_COMPLETE);laserOff();
        Serial.println(F("TARGET_HOLD_COMPLETE,2000ms_laser_command"));
        completedTargetDeparture();
      }
      break;
    case STOP:laserOff();break;
    default:rescanOrSeek();break;
  }
}

void telemetry(uint32_t now){
  if(now-lastPrintMs<PRINT_INTERVAL_MS)return;
  lastPrintMs=now;
  static uint16_t sample=0;
  ++sample;
  if(state==STOP || state==CAL)return; 
  char line[64];
  const uint8_t flags=entryFlags(now);
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


uint8_t recordCRC(const uint8_t *p){
  uint8_t crc=0;
  for(uint8_t i=0;i<LOG_BYTES-1;++i){
    crc^=p[i];
    for(uint8_t j=0;j<8;++j)crc=(crc&0x80)?(uint8_t)((crc<<1)^0x07):(uint8_t)(crc<<1);
  }
  return crc;
}
bool ringEvent(uint8_t event){
  return event==EV_AIM_WINDOW || event==EV_PULSE || event==EV_DAMP;
}
void packSigned(uint8_t *p,float value){
  const int16_t v=(int16_t)clampf(value,-32768,32767);
  p[0]=(uint8_t)v;p[1]=(uint16_t)v>>8;
}
uint8_t entryFlags(uint32_t now){
  return (irRecent(now)?1:0)|(arcValid?2:0)|(laserHigh?4:0)|
    (arcValid && absf(arcCentre-yaw)<=fireTolerance()?8:0)|
    (pointingRateReady && absf(pointingRate)<=FIRE_RATE_TOL_DPS &&
       absf(yawRateFiltered)<HOLD_RATE_HARD_DPS?16:0)|
    (absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM?32:0)|(finePulseActive?64:0)|
    (calibrated?128:0);
}
void recordEvent(uint8_t event){
  if(!recording)return;
  const uint32_t now=millis();
  // The prior run spent 12 of 14 ordinary event slots on this repeating pair.
  if((event==EV_SPEED_ON || event==EV_SPEED_OFF) && speedLogEvents>=2)return;
  const bool sample=ringEvent(event);
  const bool critical=event==EV_FAULT || event==EV_COMPLETE || event==EV_END || event==EV_SAT_ON;
  if(event==EV_PULSE){
    logPulsePair=now-lastPulseLogAt>=1000;
    if(!logPulsePair)return;
    lastPulseLogAt=now;
  }else if(event==EV_DAMP){
    if(!logPulsePair)return;
    logPulsePair=false;
  }else if(!sample && !critical && event!=EV_ARC && event!=EV_LASER_ON &&
           event!=EV_LASER_OFF && event!=EV_RECOVER && event!=EV_SEARCH_WRONG &&
           event!=EV_SCAN_REJECT && event!=EV_SPEED_ON && event!=EV_SPEED_OFF)return;
  uint8_t queued=0;
  for(uint8_t i=0;i<qSize;++i)
    if(!ringEvent(eventQueue[(qHead+i)%4][14]))++queued;
  // Reserve five event slots for completion/fault/end information.
  if(qSize==4 || (!sample && saved+queued>=(critical?LOG_EVENTS:LOG_EVENTS-5))){
    if(dropped<65535)++dropped;
    return;
  }
  if(event==EV_SPEED_ON || event==EV_SPEED_OFF)++speedLogEvents;
  uint8_t *b=eventQueue[qTail];
  // Unsigned timestamp; all other 16-bit measurements are signed.
  const uint16_t t=(uint16_t)min((uint32_t)65535,(uint32_t)((now-recordStarted)/10UL));
  b[0]=t;b[1]=t>>8;
  packSigned(b+2,(arcValid?arcCentre-yaw:0)*100.0f);
  packSigned(b+4,yawRateFiltered*100.0f);
  packSigned(b+6,pointingRate*100.0f);
  packSigned(b+8,wheelRPM*10.0f);
  const bool travel=state==CLEAR || state==SEEK ||
      (state==AIM && absf(arcCentre-yaw)>FINE_ENTER_DEG);
  packSigned(b+10,(travel?travelTrim:holdBias)*100.0f);
  const uint16_t motor=(uint16_t)(pwmApplied+255); // exact -255..+255
  b[12]=(uint8_t)motor;
  b[13]=(state&15)|((stopCode&7)<<4)|((motor&256)?128:0);
  b[14]=event;b[15]=entryFlags(now);b[16]=recordCRC(b);
  qTail=(qTail+1)%4;++qSize;
}
bool writeEEPROMByte(uint16_t address,uint8_t value){
  if(EECR&_BV(EEPE))return false;
  if(EEPROM.read(address)==value)return true;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){EEAR=address;EEDR=value;EECR|=_BV(EEMPE);EECR|=_BV(EEPE);}
  return true;
}
void recorderService(){
  if(!qSize || finePulseActive)return;
  const bool sample=ringEvent(eventQueue[qHead][14]);
  const uint8_t slot=sample?sampleNext:LOG_SAMPLES+saved;
  const uint16_t address=LOG_BASE+(uint16_t)slot*LOG_BYTES;
  // Invalidate first, then write payload/CRC, then commit the event byte.
  // A power cut during overwrite cannot expose a partially updated record.
  if(writeByte==0){
    if(writeEEPROMByte(address+14,0xFF))++writeByte;
  }else if(writeByte<=LOG_BYTES){
    const uint8_t i=writeByte-1;
    if(i==14 || writeEEPROMByte(address+i,eventQueue[qHead][i]))++writeByte;
  }else if(writeByte==LOG_BYTES+1){
    if(writeEEPROMByte(address+14,eventQueue[qHead][14]))++writeByte;
  }else if(writeByte==LOG_BYTES+2){
    const uint8_t count=sample?min((uint8_t)LOG_SAMPLES,(uint8_t)(summarySaved+1)):saved+1;
    if(writeEEPROMByte(sample?4:2,count))++writeByte;
  }else if(writeByte==LOG_BYTES+3){
    if(writeEEPROMByte(5,sample?(sampleNext+1)%LOG_SAMPLES:sampleNext))++writeByte;
  }else if(writeByte==LOG_BYTES+4){
    if(writeEEPROMByte(6,(uint8_t)dropped))++writeByte;
  }else if(writeEEPROMByte(7,dropped>>8)){
    if(sample){summarySaved=min((uint8_t)LOG_SAMPLES,(uint8_t)(summarySaved+1));sampleNext=(sampleNext+1)%LOG_SAMPLES;}
    else ++saved;
    writeByte=0;qHead=(qHead+1)%4;--qSize;
  }
}

bool logFormatOK(){return EEPROM.read(0)==0xC7 && EEPROM.read(1)==LOG_FORMAT;}
void recorderInit(){
  if(!logFormatOK()){
    state=STOP;motorOff();laserOff();
    Serial.println(F("NO_HM23_LOG,A=arm_new_format,D=check"));return;
  }
  saved=EEPROM.read(2);summarySaved=EEPROM.read(4);sampleNext=EEPROM.read(5);
  dropped=(uint16_t)EEPROM.read(6)|((uint16_t)EEPROM.read(7)<<8);
  if(saved>LOG_EVENTS || summarySaved>LOG_SAMPLES || sampleNext>=LOG_SAMPLES){
    saved=summarySaved=sampleNext=0;
    state=STOP;motorOff();laserOff();Serial.println(F("LOG_HEADER_BAD,A=reset"));return;
  }
  // A power cut after a full-ring payload commit but before its pointer
  // update must not put the newest sample first. Rebuild its order by time.
  if(summarySaved==LOG_SAMPLES){
    uint8_t b[LOG_BYTES];uint16_t latest=0;
    for(uint8_t i=0;i<LOG_SAMPLES;++i)if(readRecord(i,b)){
      const uint16_t t=b[0]|((uint16_t)b[1]<<8);
      if(t>latest || (t==latest && i==EEPROM.read(5))){latest=t;sampleNext=(i+1)%LOG_SAMPLES;}
    }
  }
  if(EEPROM.read(3)==0xA1){
    EEPROM.update(3,0);EEPROM.update(2,0);EEPROM.update(4,0);EEPROM.update(5,0);
    EEPROM.update(6,0);EEPROM.update(7,0);
    saved=summarySaved=sampleNext=0;dropped=0;speedLogEvents=0;
    recording=true;recordStarted=millis();lastSampleAt=recordStarted;
    Serial.println(F("RECORDING,HM3.2,200ms,40ring,19events"));
  }else{
    state=STOP;motorOff();laserOff();Serial.println(F("IDLE,A=arm_next_boot,D=dump"));
  }
}
int16_t readSigned(uint16_t a){return (int16_t)(EEPROM.read(a)|((uint16_t)EEPROM.read(a+1)<<8));}
bool readRecord(uint8_t slot,uint8_t *b){
  for(uint8_t i=0;i<LOG_BYTES;++i)b[i]=EEPROM.read(LOG_BASE+(uint16_t)slot*LOG_BYTES+i);
  return b[14]!=0xFF && recordCRC(b)==b[16];
}
void recorderCommands(){
  if(!Serial.available())return;
  const char c=Serial.read();if(c!='a'&&c!='A'&&c!='d'&&c!='D')return;
  if(recording && (c=='d'||c=='D'))recordEvent(EV_AIM_WINDOW);
  recording=false;laserOff();motorOff();state=STOP;finePulseActive=false;
  while(qSize || (EECR&_BV(EEPE)))recorderService();
  if(c=='a'||c=='A'){
    if(!logFormatOK() || EEPROM.read(2)>LOG_EVENTS || EEPROM.read(4)>LOG_SAMPLES || EEPROM.read(5)>=LOG_SAMPLES){
      EEPROM.update(0,0);EEPROM.update(1,LOG_FORMAT);EEPROM.update(2,0);
      EEPROM.update(4,0);EEPROM.update(5,0);EEPROM.update(6,0);EEPROM.update(7,0);
      EEPROM.update(0,0xC7);
    }
    EEPROM.update(3,0xA1);Serial.println(F("ARMED_NEXT_BOOT,power_off_then_battery_on"));return;
  }
  EEPROM.update(3,0);
  if(!logFormatOK()){Serial.println(F("OLD_LOG_PRESERVED,use_previous_sketch_to_dump"));return;}
  Serial.println(F("EVENT_LOG_BEGIN,HM3.2,format=23"));
  Serial.println(F("time_ms,event,state,error_cdeg,fast_cdps,pointing_cdps,wheel_rpm_d10,pwm,bias_cPWM,flags,stop"));
  uint8_t si=0,ei=0,bad=0,b[LOG_BYTES];
  const uint8_t first=summarySaved==LOG_SAMPLES?sampleNext:0;
  while(si<summarySaved || ei<saved){
    const uint8_t ss=(first+si)%LOG_SAMPLES,es=LOG_SAMPLES+ei;
    const uint16_t sa=LOG_BASE+(uint16_t)ss*LOG_BYTES,ea=LOG_BASE+(uint16_t)es*LOG_BYTES;
    const bool sample=si<summarySaved && (ei>=saved ||
      (uint16_t)readSigned(sa)<=(uint16_t)readSigned(ea));
    if(!readRecord(sample?ss:es,b)){++bad;if(sample)++si;else ++ei;continue;}
    if(sample)++si;else ++ei;
    Serial.print((uint32_t)(b[0]|((uint16_t)b[1]<<8))*10UL);
    Serial.print(',');Serial.print(b[14]);
    Serial.print(',');Serial.print(b[13]&15);
    for(uint8_t i=2;i<=8;i+=2){Serial.print(',');Serial.print((int16_t)(b[i]|((uint16_t)b[i+1]<<8)));}
    Serial.print(',');Serial.print((int16_t)(b[12]|((uint16_t)(b[13]&128)<<1))-255);
    Serial.print(',');Serial.print((int16_t)(b[10]|((uint16_t)b[11]<<8)));
    Serial.print(',');Serial.print(b[15]);
    Serial.print(',');Serial.println((b[13]>>4)&7);
  }
  Serial.print(F("EVENT_LOG_END,samples="));Serial.print(summarySaved);
  Serial.print(F(",events="));Serial.print(saved);
  Serial.print(F(",bad_crc="));Serial.print(bad);
  Serial.print(F(",dropped="));Serial.println(dropped);
}
void recorderObserve(){
  if(!recording)return;
  const uint32_t now=millis();
  if(state==CLEAR || state==SEEK || state==SLOW_SCAN || state==ARC_BRAKE || state==UNWIND || state==AIM || state==FINE || state==FIRE){
    if(now-lastSampleAt>=200){recordEvent(EV_AIM_WINDOW);lastSampleAt=now;}
  }
  if(now-recordStarted>=300000UL){recordEvent(EV_END);recording=false;}
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
  Serial.println(F("MQ HM 3.2 | BODY BRAKE / HEADROOM"));
  Serial.println(F("LIVE_CSV_OFF,EEPROM_ON"));
  recorderInit();

}
void loop(){
  const uint32_t now=millis();

  gyroUpdate(now);
  serviceLaserSelfTest(millis());
  irUpdate(now);
  rpmUpdate(now);
  if(state!=STOP)sequence(now);
  motorUpdate(now);
  digitalWrite(PIN_LED,state==STOP?((now/125)&1):laserHigh);
#if LIVE_CSV
  telemetry(now);
#endif
  recorderObserve();recorderService();recorderCommands();
}
