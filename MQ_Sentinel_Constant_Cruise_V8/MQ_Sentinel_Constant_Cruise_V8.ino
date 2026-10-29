/*
  MQ SENTINEL V8 — continuous-speed sweep and micro-pulse target lock
  Classic Arduino Nano (ATmega328P), DRV8874 IN/IN PMODE=HIGH, nSLEEP=HIGH.
  D5 IN1, D6 IN2, D4 laser module, D3 encoder A, D2 encoder B,
  D11 active-LOW 38kHz demodulating IR, A4/A5 MPU6050, D13 indicator.

  2 sec still -> 1000-sample gyro bias check -> SEEK constant wheel RPM
  -> capture complete IR ON/OFF arc -> AIM with braking curve -> FINE with
  low-amplitude, raised-cosine motor pulses -> FIRE when recent IR LOW pulses
  and angle/rate agree. Keep same target after fire. No serial commands.

  CARE: D4 only to correctly driven eye-safe laser module with beam contained.
  A demodulating IR receiver outputs short LOW bursts, not a steady LOW.
  SEEK/SCAN wheel RPM is regulated; constant wheel RPM cannot keep a free body
  rotating indefinitely. Automatic UNWIND restarts available wheel momentum.
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
const uint8_t YAW_AXIS=2;      // 1=MPU Y; 2=MPU Z (used for recorded yaw logs)
const float PRIOR_Y_BIAS_DPS=84.4403f/131.0f;
const float PRIOR_Z_BIAS_DPS=-0.12317f;

// -------------------------- SENSORS ---------------------------------------
const uint16_t CAL_SAMPLES=1000, GYRO_INTERVAL_MS=5;
const float GYRO_COUNTS_PER_DPS=131.0f, ACC_COUNTS_PER_G=16384.0f;
const float CAL_MAX_SD_DPS=0.65f, CAL_ACC_TOL=0.15f;
const float RATE_LPF=0.16f;
const uint32_t BOOT_WAIT_MS=2000UL;
const uint32_t IR_MIN_PULSE_US=110UL, IR_MAX_PULSE_US=30000UL;
const uint8_t IR_PULSES_REQUIRED=2;
const uint16_t IR_GROUP_MS=140, IR_LOSS_MS=170, IR_RECENT_MS=190;
const uint16_t IR_LOW_STEADY_MS=9, IR_CLEAR_MS=180;
const uint16_t IR_POLL_MS=5, IR_PATH_MS=30;
const uint16_t ARC_MIN_MS=65, ARC_MAX_MS=8500;
const float ARC_MIN_DEG=0.5f, ARC_MAX_DEG=85.0f;
const float ARC_MIN_EFFICIENCY=0.48f, ARC_MAX_BACKTRACK_DEG=2.5f;
const float ARC_MAX_SWEEP_RATE=85.0f;

// ---------------------- CRUISE / FLYWHEEL ---------------------------------
// Higher sustained wheel RPM target; PWM capped below V7 peak for margin.
const float SEEK_WHEEL_RPM=165.0f;           // signed using SEEK_MOTOR_SIGN
const int8_t SEEK_MOTOR_SIGN=-1;            // negative IN2 drive (V7 motor direction)
const float CRUISE_BASE_PWM=175.0f, CRUISE_RPM_KP=0.75f;
const float MOTOR_PWM_LIMIT=235.0f;
const float WHEEL_TAPER_RPM=205.0f, WHEEL_HARD_RPM=250.0f;
const float MOTOR_SLEW_PER_SEC=330.0f;
const uint16_t MOTOR_PERIOD_MS=5, REVERSE_DEADTIME_MS=24;
const float WHEEL_UNWIND_TRIGGER_RPM=140.0f;
const float UNWIND_PWM_LIMIT=155.0f, UNWIND_RPM_FINISH=18.0f;
const uint16_t NO_YAW_UNWIND_MS=1500;
const float NO_YAW_THRESHOLD_DPS=1.1f;
const float COUNTS_PER_REV=700.0f;

// ------------------------- AIMING / LOCK -----------------------------------
const float COARSE_RATE_LIMIT_DPS=17.0f;
const float COARSE_KP_POS=1.7f, COARSE_KP_RATE=8.0f;
const float COARSE_FRICTION_PWM=60.0f, COARSE_MAX_PWM=175.0f;
const float BRAKE_ACCEL_DPS2=20.2f;  // provisional physical estimate; tune later
const float AIM_CURVE_WIDTH_DEG=11.0f;
const float FINE_ENTER_DEG=3.2f, FINE_REENTER_DEG=4.4f;
const float FINE_ENTER_DPS=8.0f;
const uint16_t FINE_PERIOD_MS=135, FINE_PULSE_MS=55;
const float FINE_MAX_PWM=86.0f, FINE_KP_POS=28.0f, FINE_KD_RATE=11.0f;
const float FINE_MIN_ERROR=0.18f;
const float FIRE_ANGLE_TOL_DEG=2.3f, FIRE_RATE_TOL_DPS=2.4f;
const uint16_t FIRE_CONFIRM_MS=120;
const uint16_t NO_IR_AT_CENTRE_RESCAN_MS=3500;
const float MAX_FIRE_WHEEL_RPM=185.0f;
const uint16_t PRINT_INTERVAL_MS=250;

// -------------------------- STATE ------------------------------------------
enum State:uint8_t { BOOT, CAL, CLEAR, SEEK, SCAN, AIM, FINE, FIRE, UNWIND, STOP };
State state=BOOT;
uint32_t enteredMs=0, clearSince=0, lastPrintMs=0, lastGyroMs=0, lastGyroUs=0;
uint32_t lastMotorMs=0, reverseUntil=0, slowYawSince=0, unwindSince=0;
uint32_t fineStartedMs=0, stableSince=0, noIrAt=0;
uint8_t stopCode=0, failedI2C=0;
uint16_t calN=0, irOnCount=0, irOffCount=0, validArcs=0, rejectedArcs=0, fires=0;
float calMean=0, calM2=0, calSD=0, calMinG=10, calMaxG=0;
float gyroBias=0, yaw=0, yawRate=0, yawRateFiltered=0, rawRate=0, accelG=1, tempC=0;
bool calibrated=false, fallbackBias=false, laserHigh=false;
float wheelRPM=0, rpmInstant=0;
int32_t prevTicks=0;
uint32_t lastRPMMs=0;
volatile int32_t encoderTicks=0;
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
const char* stateName(){
  switch(state){
    case BOOT:return "BOOT";case CAL:return "CAL";case CLEAR:return "CLEAR";
    case SEEK:return "SEEK";case SCAN:return "SCAN";case AIM:return "AIM";
    case FINE:return "FINE";case FIRE:return "FIRE";
    case UNWIND:return "UNWIND";default:return "STOP";
  }
}
void laserOff(){digitalWrite(PIN_LASER,LOW);laserHigh=false;}
void laserOn(){
#if LASER_ENABLED
  digitalWrite(PIN_LASER,HIGH);laserHigh=true;
#endif
}
void motorHardware(int16_t p){
  if(p>0){analogWrite(PIN_IN2,0);analogWrite(PIN_IN1,p);}
  else if(p<0){analogWrite(PIN_IN1,0);analogWrite(PIN_IN2,-p);}
  else {analogWrite(PIN_IN1,0);analogWrite(PIN_IN2,0);}
}
void motorOff(){pwmGoal=pwmActual=0;pwmApplied=0;motorHardware(0);}
void enter(State s){
  state=s;enteredMs=millis();stableSince=0;
  Serial.print(F("STATE,"));Serial.print(stateName());
  Serial.print(F(",yaw="));Serial.print(yaw,2);
  Serial.print(F(",ir="));Serial.println(ir.active?1:0);
}
void safeStop(uint8_t c){
  if(state==STOP)return;
  stopCode=c;laserOff();motorOff();enter(STOP);
  Serial.print(F("SAFETY_STOP,code="));Serial.println(c);
}
void beginClear(){
  laserOff();arcValid=false;clearSince=0;slowYawSince=0;
  enter(CLEAR);
}
void encoderRise(){encoderTicks+=digitalRead(PIN_ENC_B)?-1:+1;}
int32_t countNow(){int32_t c;ATOMIC_BLOCK(ATOMIC_RESTORESTATE){c=encoderTicks;}return c;}

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
  dps=v[YAW_AXIS==1?5:6]/GYRO_COUNTS_PER_DPS;
  return true;
}
void startCalibration(){
  motorOff();laserOff();calibrated=false;
  calN=0;calMean=calM2=calSD=0;calMinG=10;calMaxG=0;
  enter(CAL);
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
    if(g<calMinG)calMinG=g;
    if(g>calMaxG)calMaxG=g;
    if(calN>=CAL_SAMPLES){
      calSD=sqrtf(calM2/(calN-1));
      const float prior=YAW_AXIS==1?PRIOR_Y_BIAS_DPS:PRIOR_Z_BIAS_DPS;
      const bool still=calSD<CAL_MAX_SD_DPS&&absf(calMean-prior)<0.20f&&
          calMinG>1.0f-CAL_ACC_TOL&&calMaxG<1.0f+CAL_ACC_TOL;
      fallbackBias=!still;
      gyroBias=still?(0.80f*calMean+0.20f*prior):prior;
      calibrated=true;yaw=yawRate=yawRateFiltered=0;lastGyroUs=us;
      Serial.print(F("CAL_DONE,bias_dps="));Serial.print(gyroBias,5);
      Serial.print(F(",sd="));Serial.print(calSD,4);
      Serial.print(F(",temp_C="));Serial.print(tempC,2);
      Serial.print(F(",source="));Serial.println(still?F("MEASURED+KNOWN"):F("KNOWN_FALLBACK"));
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
      if(state==SCAN){
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
bool irRecent(uint32_t now){
  // A demodulated 38kHz signal consists of short LOW pulses; constant LOW
  // cannot be required at one arbitrary instant for a stable laser lock.
  uint32_t lastLow;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){lastLow=lastLowSeenUs;}
  return ir.active && (uint32_t)(micros()-lastLow)<IR_RECENT_MS*1000UL &&
         (uint32_t)(now-ir.lastSignal)<IR_RECENT_MS;
}
void initialiseArc(float entry,uint32_t ms){
  arcEntry=entry;arcStarted=ms;
  arcExit=entry;arcMin=arcMax=entry;arcPath=arcMaxRate=0;
  lastArcPathYaw=entry;lastArcPathMs=millis();
  enter(SCAN);
}
void finishArc(float exit,uint32_t ms){
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
    beginClear();
  }else{
    ++validArcs;arcCentre=(arcEntry+arcExit)*0.5f;arcValid=true;
    Serial.print(F("ARC_OK,entry="));Serial.print(arcEntry,2);
    Serial.print(F(",exit="));Serial.print(arcExit,2);
    Serial.print(F(",centre="));Serial.print(arcCentre,2);
    Serial.print(F(",width="));Serial.println(arcWidth,2);
    motorOff();             // immediately stop the high-RPM cruise command
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
    ir.pulsesDisplayed+=newPulses;
    if(!ir.groupStart||now-ir.groupStart>IR_GROUP_MS){
      ir.groupStart=now;ir.groupPulses=0;ir.firstYaw=yaw;
    }
    const uint32_t sum=(uint32_t)ir.groupPulses+newPulses;
    ir.groupPulses=(sum>65535?65535:(uint16_t)sum);
    ir.lastSignal=now;ir.lastYaw=yaw;
  }
  const bool steadyLow=low&&((uint32_t)(micros()-lowAt)>=IR_LOW_STEADY_MS*1000UL);
  if(steadyLow){
    if(!ir.groupStart||now-ir.groupStart>IR_GROUP_MS){
      ir.groupStart=now;ir.groupPulses=0;ir.firstYaw=yaw;
    }
    ir.lastSignal=now;ir.lastYaw=yaw;
  }
  if(!ir.active){
    if((ir.groupPulses>=IR_PULSES_REQUIRED&&now-ir.groupStart<=IR_GROUP_MS)||steadyLow){
      ir.active=true;++irOnCount;
      Serial.print(F("IR_ON,edge="));Serial.print(ir.firstYaw,2);
      Serial.print(F(",pulses="));Serial.println(ir.groupPulses);
      if(state==SEEK)initialiseArc(ir.firstYaw,ir.groupStart);
    }
  }else if(!low && now-ir.lastSignal>=IR_LOSS_MS){
    ir.active=false;++irOffCount;
    Serial.print(F("IR_OFF,edge="));Serial.println(ir.lastYaw,2);
    ir.groupPulses=0;ir.groupStart=0;
    if(state==SCAN)finishArc(ir.lastYaw,ir.lastSignal);
  }
  if(!ir.active&&ir.groupStart&&now-ir.groupStart>IR_GROUP_MS){
    ir.groupStart=0;ir.groupPulses=0;
  }
}

// --------------------------- MOTOR CONTROL --------------------------------
void rpmUpdate(uint32_t now){
  if(now-lastRPMMs<100)return;
  const int32_t ticks=countNow();
  if(lastRPMMs){
    rpmInstant=(ticks-prevTicks)*60000.0f/(COUNTS_PER_REV*(now-lastRPMMs));
    wheelRPM+=0.35f*(rpmInstant-wheelRPM);
    if(absf(wheelRPM)>WHEEL_HARD_RPM||absf(rpmInstant)>WHEEL_HARD_RPM+25.0f)safeStop(2);
  }
  lastRPMMs=now;prevTicks=ticks;
}
float cruisePWM(){
  // Wheel RPM closed loop, instead of gyro-rate driven jerky seek PWM.
  const float target=SEEK_WHEEL_RPM;
  const float current=wheelRPM*SEEK_MOTOR_SIGN*ENCODER_SIGN;
  float out=CRUISE_BASE_PWM+CRUISE_RPM_KP*(target-current);
  if(current>target+8.0f)out=0;  // coast if above target, no instant reversal
  if(absf(wheelRPM)>WHEEL_TAPER_RPM){
    const float k=clampf((WHEEL_HARD_RPM-absf(wheelRPM))/(WHEEL_HARD_RPM-WHEEL_TAPER_RPM),0,1);
    out*=k;
  }
  return SEEK_MOTOR_SIGN*clampf(out,0,MOTOR_PWM_LIMIT);
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
  // Raised-cosine pulse envelope: gradually ramps 0 -> peak -> 0.
  // Micro pulses repeat at 135ms intervals, with zero power between pulses.
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
    if(pwmApplied)motorOff();
    return;
  }
  float goal=0;
  if(state==CLEAR||state==SEEK||state==SCAN)goal=cruisePWM();
  else if(state==UNWIND){
    goal=clampf(-wheelRPM*ENCODER_SIGN*1.30f,-UNWIND_PWM_LIMIT,UNWIND_PWM_LIMIT);
  }
  else if(state==AIM)goal=coarsePWM();
  else if(state==FINE||state==FIRE)goal=finePWM(now);
  // No sudden discontinuities outside fine mode. Fine pulse envelope supplies
  // its own rise/fall ramp and must not be stretched by large slew memory.
  const bool micro=(state==FINE||state==FIRE);
  const float step=(micro?2500.0f:MOTOR_SLEW_PER_SEC)*dt;
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
  // Do not call a narrow IR cone centred when merely near its edge.
  return clampf(arcWidth*0.40f,0.65f,FIRE_ANGLE_TOL_DEG);
}
bool centred(uint32_t now){
  return calibrated&&arcValid&&irRecent(now)&&
    absf(arcCentre-yaw)<=fireTolerance()&&
    absf(yawRateFiltered)<=FIRE_RATE_TOL_DPS&&
    absf(wheelRPM)<=MAX_FIRE_WHEEL_RPM;
}
const char* gate(uint32_t now){
  if(!arcValid)return "NOARC";
  if(!irRecent(now))return "NOIR";
  if(absf(arcCentre-yaw)>fireTolerance())return "ANGLE";
  if(absf(yawRateFiltered)>FIRE_RATE_TOL_DPS)return "RATE";
  if(absf(wheelRPM)>MAX_FIRE_WHEEL_RPM)return "WHEEL";
  return "READY";
}
void sequence(uint32_t now){
  switch(state){
    case BOOT:
      if(now-enteredMs>=BOOT_WAIT_MS)startCalibration();
      break;
    case CAL:break;
    case CLEAR:
      if(!ir.active){
        if(!clearSince)clearSince=now;
        if(now-clearSince>=IR_CLEAR_MS){clearSince=0;enter(SEEK);}
      }else clearSince=0;
      break;
    case SEEK:
      // Constant cruise RPM does not create indefinite reaction torque.
      // Rewind if wheel is spinning but the body is no longer rotating.
      if(absf(wheelRPM)>WHEEL_UNWIND_TRIGGER_RPM &&
         absf(yawRateFiltered)<NO_YAW_THRESHOLD_DPS){
        if(!slowYawSince)slowYawSince=now;
        if(now-slowYawSince>NO_YAW_UNWIND_MS){
          slowYawSince=0;unwindSince=0;
          enter(UNWIND);
          Serial.println(F("MOMENTUM_RESET,body_rate_low_at_cruise_RPM"));
        }
      }else slowYawSince=0;
      break;
    case SCAN:
      if(now-arcStarted>ARC_MAX_MS+IR_LOSS_MS){
        ++rejectedArcs;Serial.println(F("ARC_BAD,reason=timeout"));beginClear();
      }
      break;
    case UNWIND:
      if(absf(wheelRPM)<UNWIND_RPM_FINISH){
        if(!unwindSince)unwindSince=now;
        if(now-unwindSince>=300)beginClear();
      }else unwindSince=0;
      break;
    case AIM:
      if(absf(arcCentre-yaw)<=FINE_ENTER_DEG&&absf(yawRateFiltered)<=FINE_ENTER_DPS){
        fineStartedMs=now;enter(FINE);
      }
      break;
    case FINE:
      if(absf(arcCentre-yaw)>FINE_REENTER_DEG){enter(AIM);break;}
      if(centred(now)){
        if(!stableSince)stableSince=now;
        if(now-stableSince>=FIRE_CONFIRM_MS){
          laserOn();++fires;enter(FIRE);
          Serial.println(F("LASER_ON,D4_HIGH,arc_centre_and_recent_IR_LOW"));
        }
      }else stableSince=0;
      if(arcValid&&absf(arcCentre-yaw)<fireTolerance()&&!irRecent(now)){
        if(!noIrAt)noIrAt=now;
        if(now-noIrAt>NO_IR_AT_CENTRE_RESCAN_MS){
          Serial.println(F("REACQUIRE,midpoint_without_IR"));beginClear();
        }
      }else noIrAt=0;
      break;
    case FIRE:
      // No fixed 2-second firing timer; protect lock continuously.
      if(!centred(now)){
        laserOff();fineStartedMs=now;enter(FINE);
        Serial.println(F("LASER_OFF,centre_or_IR_lost"));
      }
      break;
    case STOP:laserOff();break;
  }
}

// ----------------------------- TELEMETRY ----------------------------------
void header(){
  Serial.println(F(" Time | State  | Yaw(deg) | Rate(dps) | IR | Raw | Puls | PWM  | RPM   | Centre | Error | Gate  | LZR"));
  Serial.println(F("------+--------+----------+-----------+----+-----+------+-------+-------+--------+-------+-------+----"));
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
  Serial.print(F(" | "));printFloat(wheelRPM,5,1);
  Serial.print(F(" | "));
  if(arcValid)printFloat(arcCentre,6,2);else Serial.print(F("    --"));
  Serial.print(F(" | "));
  if(arcValid)printFloat(arcCentre-yaw,5,2);else Serial.print(F("   --"));
  Serial.print(F(" | "));Serial.print(gate(now));
  Serial.print(F(" |  "));Serial.println(laserHigh?1:0);
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
  Serial.println(F("MQ SENTINEL V8 | CONSTANT RPM SEEK | FINE MICRO-PULSES | FRESH IR ARC"));
  Serial.println(F("D5/D6 MOTOR | D4 LASER | D3/D2 ENCODER | D11 IR | A4/A5 GYRO"));
  Serial.print(F("CRUISE_RPM="));Serial.print(SEEK_WHEEL_RPM,0);
  Serial.print(F(",MAX_PWM="));Serial.print(MOTOR_PWM_LIMIT,0);
  Serial.print(F(",YAW_AXIS="));Serial.print(YAW_AXIS==1?'Y':'Z');
  Serial.print(F(",LASER_ENABLED="));Serial.println(LASER_ENABLED);
}
void loop(){
  const uint32_t now=millis();
  if(state!=STOP){
    gyroUpdate(now);
    if(state!=STOP)irUpdate(now);
    if(state!=STOP)rpmUpdate(now);
    if(state!=STOP)sequence(now);
  }
  motorUpdate(now);
  digitalWrite(PIN_LED,state==STOP?((now/125)&1):laserHigh);
  telemetry(now);
}
