/*
 MQ SENTINEL V9.1 - NO EEPROM, DIRECTIONAL PERSISTENCE
 Arduino Nano ATmega328P, DRV8874 IN/IN
 D2 left IR, D3 centre IR, D4 right IR (active LOW, 38 kHz receivers)
 D6 IN1 PWM, D7 IN2, D8 laser, D9 encoder A, D10 encoder B, D13 LED.

 Power ON -> safe 5-second boot pause -> WAIT_FLASH -> track upon valid IR.
 Every power cycle starts a new run. No EEPROM read/write and no stored arm flag.
 Serial 115200: T toggle telemetry, D one status report, X emergency stop,
 R restart WAIT_FLASH, C clear RAM counters + restart WAIT_FLASH.
 USB disconnection does not preserve RAM metrics. Logs may be captured live on PC.
 Laser accepts stable 010 or 111 for 180 ms; 101 maintains previous correction
 at half coarse PWM; 111 stops. Laser max 2000 ms then SEEK; aborts on
 confirmed centre loss for safety.
  Keep the laser eye-safe and ensure its switching hardware is compatible with D8.
*/
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

const uint8_t PIN_IR[3] = {2,3,4};
const uint8_t PIN_IN1=6, PIN_IN2=7, PIN_LASER=8;
const uint8_t PIN_ENC_A=9, PIN_ENC_B=10, PIN_LED=13;
const int8_t BODY_CCW=-1, BODY_CW=1;
const int8_t LEFT_TORQUE=BODY_CCW, RIGHT_TORQUE=BODY_CW;
const uint16_t ENCODER_CPR=700;
const uint8_t IR_MASK=_BV(PD2)|_BV(PD3)|_BV(PD4);

const float SEEK_PWM=174.0f, SEEK_RPM_SOFT=145.0f;
const float SEEK_RPM_HARD=195.0f, SEEK_MIN_PWM=108.0f;
const float COARSE_PWM=148.0f, FINE_PWM=100.0f;
const float AMBIGUOUS_101_PWM=COARSE_PWM*0.5f; // 74 PWM, last correction direction
const float COUNTER_PWM=70.0f;
const uint16_t COUNTER_MS=145;
const uint16_t IR_FRESH_MS=65, IR_MIN_PULSE_US=500, IR_MAX_PULSE_US=20000;
const uint16_t IR_CONFIRM_WINDOW_MS=160;
const uint8_t IR_REQUIRED_PULSES=2;
const uint16_t MICRO_SIDE_STABLE_MS=70, MICRO_PULSE_MS=145;
const uint16_t MICRO_COOLDOWN_MS=240, MIN_TARGET_LOST_MS=230;
const uint16_t SETTLE_MS=180, QUIET_MOTOR_MS=0; // 180ms confirmed centre lock; no motor-speed veto
const uint16_t FIRE_MS=2000, CENTRE_DROPOUT_GRACE_MS=80;
const uint16_t POST_SHOT_CLEAR_MS=180, TRACE_MS=250;
const float MOTOR_OMEGA=0.0125f, MOTOR_MAX_RATE=0.76f;
const uint16_t MOTOR_TICK_MS=5, ZERO_CROSS_GUARD_MS=18;
const uint16_t BOOT_DELAY_MS=5000;
const bool ACCEPT_ALL_THREE_AS_CENTRED=true;

volatile uint8_t irPreviousPort=0;
volatile uint32_t irFallMicros[3]={0,0,0};
volatile uint8_t irCompleted[3]={0,0,0};
volatile uint16_t irRawFalls[3]={0,0,0};
volatile uint16_t irAccepted[3]={0,0,0};
volatile uint16_t irRejected[3]={0,0,0};
volatile uint8_t irLowValid[3]={0,0,0};
volatile long encoderTicks=0;
volatile unsigned long encoderEdges=0;
volatile uint8_t previousEncoderA=0;

uint32_t lastIR[3]={0,0,0}, firstEvidenceAt[3]={0,0,0};
uint32_t lastEvidenceAt[3]={0,0,0};
uint8_t evidenceCount[3]={0,0,0};
uint8_t rawIRMask=0, sensorMask=0, lastSensorMask=0;
unsigned long zoneChangedAt=0, centreCandidateSince=0, centreLastSeenAt=0;
float wheelRPM=0;
long previousEncoder=0;
unsigned long speedSampleAt=0;

ISR(PCINT0_vect) {
  uint8_t a=(PINB&_BV(PB1))?1:0;
  if(a && !previousEncoderA) {
    encoderTicks+=(PINB&_BV(PB2))?-1:1;
    encoderEdges++;
  }
  previousEncoderA=a;
}
ISR(PCINT2_vect) {
  uint8_t port=PIND&IR_MASK;
  uint8_t changed=(port^irPreviousPort)&IR_MASK;
  irPreviousPort=port;
  if(!changed) return;
  uint32_t nowUs=micros();
  for(uint8_t i=0;i<3;i++) {
    uint8_t bit=(uint8_t)(1U<<(i+2));
    if(!(changed&bit)) continue;
    if(!(port&bit)) {
      irFallMicros[i]=nowUs;
      irLowValid[i]=1;
      irRawFalls[i]++;
    } else if(irLowValid[i]) {
      uint32_t width=nowUs-irFallMicros[i];
      irLowValid[i]=0;
      if(width>=IR_MIN_PULSE_US && width<=IR_MAX_PULSE_US) {
        if(irCompleted[i]!=255) irCompleted[i]++;
        irAccepted[i]++;
      } else irRejected[i]++;
    }
  }
}

static bool hasLeft(uint8_t m){return m&1;}
static bool hasCentre(uint8_t m){return m&2;}
static bool hasRight(uint8_t m){return m&4;}
static bool isCentred(uint8_t m){return m==2||(ACCEPT_ALL_THREE_AS_CENTRED&&m==7);}

void readSensors(unsigned long now) {
  uint8_t pulses[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(uint8_t i=0;i<3;i++){pulses[i]=irCompleted[i];irCompleted[i]=0;}
  }
  uint8_t next=0;
  rawIRMask=0;
  for(uint8_t i=0;i<3;i++) {
    uint8_t bit=(uint8_t)(1U<<i);
    if(digitalRead(PIN_IR[i])==LOW) rawIRMask|=bit;
    if(pulses[i]) {
      if(!evidenceCount[i]||now-firstEvidenceAt[i]>IR_CONFIRM_WINDOW_MS) {
        firstEvidenceAt[i]=now;
        evidenceCount[i]=0;
      }
      evidenceCount[i]=(uint8_t)min((uint16_t)IR_REQUIRED_PULSES,
                        (uint16_t)(evidenceCount[i]+pulses[i]));
      lastEvidenceAt[i]=now;
      if(evidenceCount[i]>=IR_REQUIRED_PULSES){lastIR[i]=now;firstEvidenceAt[i]=now;}
    } else if(evidenceCount[i]&&now-lastEvidenceAt[i]>IR_CONFIRM_WINDOW_MS) evidenceCount[i]=0;
    if(lastIR[i]&&now-lastIR[i]<=IR_FRESH_MS) next|=bit;
  }
  sensorMask=next;
  if(isCentred(next)) {
    if(!centreCandidateSince) centreCandidateSince=now;
    centreLastSeenAt=now;
  } else if(!centreLastSeenAt||now-centreLastSeenAt>CENTRE_DROPOUT_GRACE_MS) centreCandidateSince=0;
  if(next!=lastSensorMask){zoneChangedAt=now;lastSensorMask=next;}
}

void readWheel(unsigned long now) {
  if(now-speedSampleAt<75UL) return;
  long count;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){count=encoderTicks;}
  unsigned long dt=now-speedSampleAt;
  wheelRPM=(count-previousEncoder)*60000.0f/(ENCODER_CPR*(float)dt);
  previousEncoder=count;
  speedSampleAt=now;
}

float requestedPWM=0, motorPWM=0, motorVelocity=0;
unsigned long lastMotorAt=0, zeroCrossSince=0;
int8_t lastPhysicalPolarity=0;
int16_t lastAppliedPWM=0;
void releaseMotor(){
  analogWrite(PIN_IN1,0);
  digitalWrite(PIN_IN1,LOW);
  digitalWrite(PIN_IN2,LOW);
  lastAppliedPWM=0;
}
void setMotorRaw(int16_t signedPower){
  int power=constrain(abs(signedPower),0,255);
  if(power<8){releaseMotor();return;}
  if(signedPower<0){digitalWrite(PIN_IN2,HIGH);analogWrite(PIN_IN1,255-power);}
  else {digitalWrite(PIN_IN2,LOW);analogWrite(PIN_IN1,power);}
  lastAppliedPWM=signedPower;
}
void requestMotor(float signedTarget){requestedPWM=constrain(signedTarget,-205.0f,205.0f);}
void emergencyMotorOff(){
  requestedPWM=motorPWM=motorVelocity=0;
  lastPhysicalPolarity=0;
  zeroCrossSince=0;
  releaseMotor();
}
void serviceMotor(unsigned long now){
  if(now-lastMotorAt<MOTOR_TICK_MS) return;
  float dt=(float)min(now-lastMotorAt,(unsigned long)25);
  lastMotorAt=now;
  float error=requestedPWM-motorPWM;
  float accel=MOTOR_OMEGA*MOTOR_OMEGA*error-2.0f*MOTOR_OMEGA*motorVelocity;
  motorVelocity=constrain(motorVelocity+accel*dt,-MOTOR_MAX_RATE,MOTOR_MAX_RATE);
  float next=motorPWM+motorVelocity*dt;
  if((requestedPWM-motorPWM)*(requestedPWM-next)<=0){next=requestedPWM;motorVelocity=0;}
  motorPWM=next;
  int16_t output=(int16_t)motorPWM;
  int8_t sign=output>7?1:(output< -7?-1:0);
  if(!sign){
    if(lastPhysicalPolarity) zeroCrossSince=now;
    lastPhysicalPolarity=0;
    releaseMotor();
    return;
  }
  if(lastPhysicalPolarity&&sign!=lastPhysicalPolarity){
    lastPhysicalPolarity=0;
    zeroCrossSince=now;
    releaseMotor();
    return;
  }
  if(!lastPhysicalPolarity){
    if(zeroCrossSince==0)zeroCrossSince=now;
    if(now-zeroCrossSince<ZERO_CROSS_GUARD_MS){releaseMotor();return;}
    lastPhysicalPolarity=sign;
  }
  setMotorRaw(output);
}

enum Mode:uint8_t {WAIT_FLASH,SEEK,TRACK,SETTLE,FIRE,HALT};
Mode mode=WAIT_FLASH;
unsigned long modeSince=0,laserStarted=0;
int8_t seekDirection=BODY_CW,lastCorrectionDirection=0;
unsigned long counterStarted=0, fineZoneSince=0, microPulseAt=0, microCooldownUntil=0;
unsigned long clearStarted=0,missingStarted=0;
int8_t fineZoneDirection=0,counterDirection=0;
bool microPulseActive=false,requireClearAfterShot=false,telemetryEnabled=false;
unsigned long telemetryLastAt=0,runStart=0;

// These are RAM-only statistics, reset on power loss or C command.
uint32_t targetSeenCount=0,targetLostCount=0,zoneTransitions=0;
uint32_t confirmedAlignments=0,shotAttempts=0,shotCompletions=0,shotAborts=0;
uint32_t totalLaserOnMs=0,firstDetectionMs=0,firstAlignmentMs=0;
uint16_t maxLaserOnMs=0;
uint8_t previousLoggedZone=0;

void updateCounters(unsigned long now){
  if(sensorMask==previousLoggedZone)return;
  uint8_t old=previousLoggedZone;
  previousLoggedZone=sensorMask;
  zoneTransitions++;
  if(!old&&sensorMask){
    targetSeenCount++;
    if(!firstDetectionMs)firstDetectionMs=now-runStart;
  } else if(old&&!sensorMask) targetLostCount++;
}

void modeTo(uint8_t next,unsigned long now){mode=static_cast<Mode>(next);modeSince=now;}
void enterStandby(unsigned long now){
  digitalWrite(PIN_LASER,LOW);
  emergencyMotorOff();
  lastCorrectionDirection=0;
  clearStarted=missingStarted=counterStarted=0;
  fineZoneSince=microPulseAt=microCooldownUntil=0;
  fineZoneDirection=0;
  microPulseActive=false;
  requireClearAfterShot=false;
  modeTo(WAIT_FLASH,now);
}
void enterSeek(unsigned long now){
  seekDirection=-seekDirection;
  counterStarted=0;
  lastCorrectionDirection=0;
  missingStarted=0;
  microPulseActive=false;
  fineZoneDirection=0;
  requestMotor(0);
  modeTo(SEEK,now);
}
void enterTrack(unsigned long now){
  Mode previous=mode;
  modeTo(TRACK,now);
  missingStarted=counterStarted=0;
  microPulseActive=false;
  fineZoneDirection=0;
  lastCorrectionDirection=previous==SEEK?seekDirection:0;
}
float seekPower(){
  float rpm=abs(wheelRPM);
  if(rpm<=SEEK_RPM_SOFT)return SEEK_PWM;
  if(rpm>=SEEK_RPM_HARD)return SEEK_MIN_PWM;
  return SEEK_PWM-(SEEK_PWM-SEEK_MIN_PWM)*
           ((rpm-SEEK_RPM_SOFT)/(SEEK_RPM_HARD-SEEK_RPM_SOFT));
}
int8_t zoneCorrection(uint8_t mask){
  if(mask==1||mask==3)return LEFT_TORQUE;
  if(mask==4||mask==6)return RIGHT_TORQUE;
  return 0;
}
void driveTracking(unsigned long now){
  int8_t correction=zoneCorrection(sensorMask);
  bool fineOverlap=sensorMask==3||sensorMask==6;
  if(fineOverlap){
    if(fineZoneDirection!=correction){
      fineZoneDirection=correction;
      fineZoneSince=now;
      microPulseActive=false;
      microCooldownUntil=now+MICRO_SIDE_STABLE_MS;
      requestMotor(0);
      return;
    }
    if(now-fineZoneSince<MICRO_SIDE_STABLE_MS){requestMotor(0);return;}
    if(!microPulseActive&&(long)(now-microCooldownUntil)>=0){
      microPulseActive=true;
      microPulseAt=now;
      lastCorrectionDirection=correction;
    }
    if(microPulseActive&&now-microPulseAt<MICRO_PULSE_MS){
      counterStarted=0;
      requestMotor(correction*FINE_PWM);
    } else {
      if(microPulseActive){microPulseActive=false;microCooldownUntil=now+MICRO_COOLDOWN_MS;}
      requestMotor(0);
    }
    return;
  }
  microPulseActive=false;
  fineZoneDirection=0;
  // Both outer sensors see the target, but centre does not. Keep moving
  // towards the last known correction at half coarse speed. Do not infer
  // a direction if no previous correction was established.
  if(sensorMask==5){
    counterStarted=0;
    requestMotor(lastCorrectionDirection * AMBIGUOUS_101_PWM);
    return;
  }
  // 111 (all three) and 010 both require zero motor torque before firing.
  // Do not apply counter-torque to 111; it is a valid lock candidate.
  if(sensorMask==7){
    counterStarted=0;
    requestMotor(0);
    return;
  }
  if(correction){
    lastCorrectionDirection=correction;
    counterStarted=0;
    requestMotor(correction*COARSE_PWM);
    return;
  }
  if(isCentred(sensorMask)){
    if(lastCorrectionDirection&&!counterStarted){
      counterStarted=now?now:1;
      counterDirection=-lastCorrectionDirection;
      lastCorrectionDirection=0;
    }
    if(counterStarted&&now-counterStarted<COUNTER_MS)requestMotor(counterDirection*COUNTER_PWM);
    else requestMotor(0);
    return;
  }
  counterStarted=0;
  requestMotor(0);
}

// File-local (not function-local) timer is reset explicitly for each new settle.
unsigned long motorQuietSince=0;
void serviceController(unsigned long now){
  if(mode==HALT){digitalWrite(PIN_LASER,LOW);emergencyMotorOff();return;}
  if(mode==WAIT_FLASH){
    digitalWrite(PIN_LASER,LOW);
    requestMotor(0);
    if(sensorMask)enterTrack(now);
    return;
  }
  if(mode==SEEK){
    digitalWrite(PIN_LASER,LOW);
    if(requireClearAfterShot){
      if(sensorMask==0){
        if(!clearStarted)clearStarted=now;
        if(now-clearStarted>=POST_SHOT_CLEAR_MS){requireClearAfterShot=false;clearStarted=0;}
      } else clearStarted=0;
    }
    if(!requireClearAfterShot&&sensorMask){enterTrack(now);return;}
    requestMotor(seekDirection*seekPower());
    return;
  }
  if(mode==TRACK){
    digitalWrite(PIN_LASER,LOW);
    if(!sensorMask){
      requestMotor(0);
      if(!missingStarted)missingStarted=now;
      if(now-missingStarted>=MIN_TARGET_LOST_MS)enterSeek(now);
      return;
    }
    missingStarted=0;
    driveTracking(now);
    if(isCentred(sensorMask)&&centreCandidateSince&&now-centreCandidateSince>=40UL){
      motorQuietSince=0;
      modeTo(SETTLE,now);
    }
    return;
  }
  if(mode==SETTLE){
    digitalWrite(PIN_LASER,LOW);
    requestMotor(0); // stop applying corrections once centre has been acquired
    // A stable confirmed centre-only detection is sufficient to fire.
    // Do not depend on flywheel RPM or the motor PWM ramp reaching precisely zero:
    // the encoder measures the wheel, not the satellite chassis.
    if(!isCentred(sensorMask)){
      if(centreLastSeenAt && now-centreLastSeenAt<=CENTRE_DROPOUT_GRACE_MS)
        return; // tolerate short AGC gaps
      motorQuietSince=0;
      counterStarted=0;
      modeTo(TRACK,now);
      return;
    }
    if(centreCandidateSince && now-centreCandidateSince>=SETTLE_MS){
      confirmedAlignments++;
      if(!firstAlignmentMs)firstAlignmentMs=now-runStart;
      emergencyMotorOff(); // ensure the motor driver is off before laser ON
      modeTo(FIRE,now);
      laserStarted=now;
      shotAttempts++;
      digitalWrite(PIN_LASER,HIGH);
      Serial.println(F("LASER ON: 010/111 confirmed; max 2000ms pulse"));
    }
    return;
  }
  if(mode==FIRE){
    // Disable laser if centre alignment has been lost beyond the normal
    // receiver dropout grace. A lost beacon must not illuminate blindly.
    requestMotor(0);
    if(!isCentred(sensorMask) &&
       (!centreLastSeenAt || now-centreLastSeenAt>CENTRE_DROPOUT_GRACE_MS)){
      digitalWrite(PIN_LASER,LOW);
      uint16_t duration=(uint16_t)min(now-laserStarted,65535UL);
      shotAborts++;
      totalLaserOnMs+=duration;
      maxLaserOnMs=max(maxLaserOnMs,duration);
      requireClearAfterShot=true;
      clearStarted=0;
      enterSeek(now);
      Serial.println(F("LASER ABORT: centre lost; SEEKING"));
      return;
    }
    if(now-laserStarted>=FIRE_MS){
      digitalWrite(PIN_LASER,LOW);
      uint16_t duration=(uint16_t)min(now-laserStarted,65535UL);
      shotCompletions++;
      totalLaserOnMs+=duration;
      maxLaserOnMs=max(maxLaserOnMs,duration);
      requireClearAfterShot=true; // avoid immediately firing the same beacon again
      clearStarted=0;
      enterSeek(now); // begin turning on the very next loop, without a delay
      Serial.println(F("LASER OFF: 2000ms complete; SEEKING next target"));
    }
    return;
  }
}

void printStatus(unsigned long now){
  unsigned long edges;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){edges=encoderEdges;}
  Serial.println(F("BEGIN SENTINEL V9.1 RAM STATUS"));
  Serial.print(F("uptime_ms,"));Serial.println(now-runStart);
  Serial.print(F("mode,"));Serial.println((int)mode);
  Serial.print(F("zone,"));Serial.println(sensorMask);
  Serial.print(F("wheel_rpm,"));Serial.println(wheelRPM,1);
  Serial.print(F("motor_pwm,"));Serial.println(lastAppliedPWM);
  Serial.print(F("encoder_edges,"));Serial.println(edges);
  Serial.print(F("target_seen,"));Serial.println(targetSeenCount);
  Serial.print(F("target_lost,"));Serial.println(targetLostCount);
  Serial.print(F("zone_transitions,"));Serial.println(zoneTransitions);
  Serial.print(F("confirmed_alignments,"));Serial.println(confirmedAlignments);
  Serial.print(F("shot_attempts,"));Serial.println(shotAttempts);
  Serial.print(F("shot_completions,"));Serial.println(shotCompletions);
  Serial.print(F("shot_aborts,"));Serial.println(shotAborts);
  Serial.print(F("max_laser_on_ms,"));Serial.println(maxLaserOnMs);
  Serial.print(F("total_laser_on_ms,"));Serial.println(totalLaserOnMs);
  Serial.print(F("first_detection_ms,"));Serial.println(firstDetectionMs);
  Serial.print(F("first_alignment_ms,"));Serial.println(firstAlignmentMs);
  Serial.println(F("END SENTINEL V8.9"));
}

void showTelemetry(unsigned long now){
  if(!telemetryEnabled||now-telemetryLastAt<TRACE_MS)return;
  telemetryLastAt=now;
  static uint16_t prevRaw[3]={0,0,0},prevValid[3]={0,0,0},prevReject[3]={0,0,0};
  uint16_t raw[3],valid[3],rejected[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(uint8_t i=0;i<3;i++){
      raw[i]=irRawFalls[i];valid[i]=irAccepted[i];rejected[i]=irRejected[i];
    }
  }
  Serial.print(F("t="));Serial.print(now-runStart);
  Serial.print(F(" mode="));Serial.print((int)mode);
  Serial.print(F(" filtered(LCR)="));
  Serial.print(hasLeft(sensorMask)?'1':'0');
  Serial.print(hasCentre(sensorMask)?'1':'0');
  Serial.print(hasRight(sensorMask)?'1':'0');
  Serial.print(F(" rawLOW(LCR)="));
  for(uint8_t i=0;i<3;i++)Serial.print((rawIRMask&(1<<i))?'1':'0');
  Serial.print(F(" falls(L/C/R)="));
  for(uint8_t i=0;i<3;i++){
    if(i)Serial.print('/');
    Serial.print((uint16_t)(raw[i]-prevRaw[i]));prevRaw[i]=raw[i];
  }
  Serial.print(F(" valid="));
  for(uint8_t i=0;i<3;i++){
    if(i)Serial.print('/');
    Serial.print((uint16_t)(valid[i]-prevValid[i]));prevValid[i]=valid[i];
  }
  Serial.print(F(" reject="));
  for(uint8_t i=0;i<3;i++){
    if(i)Serial.print('/');
    Serial.print((uint16_t)(rejected[i]-prevReject[i]));prevReject[i]=rejected[i];
  }
  Serial.print(F(" rpm="));Serial.print(wheelRPM,1);
  Serial.print(F(" pwm="));Serial.println(motorPWM,0);
}

void resetRamCounters(unsigned long now){
  targetSeenCount=targetLostCount=zoneTransitions=0;
  confirmedAlignments=shotAttempts=shotCompletions=shotAborts=0;
  totalLaserOnMs=firstDetectionMs=firstAlignmentMs=0;
  maxLaserOnMs=0;
  previousLoggedZone=sensorMask;
  runStart=now;
}
void serialCommands(unsigned long now){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='T'||c=='t'){
      telemetryEnabled=!telemetryEnabled;
      Serial.println(telemetryEnabled?F("TELEMETRY ON"):F("TELEMETRY OFF"));
    } else if(c=='D'||c=='d')printStatus(now);
    else if(c=='X'||c=='x'){
      digitalWrite(PIN_LASER,LOW);
      emergencyMotorOff();
      modeTo(HALT,now);
      Serial.println(F("HALTED. R=restart standby, C=reset counters and restart."));
    } else if(c=='R'||c=='r'){
      enterStandby(now);
      Serial.println(F("READY: waiting for a validated IR flash."));
    } else if(c=='C'||c=='c'){
      enterStandby(now);
      resetRamCounters(now);
      Serial.println(F("RAM COUNTERS CLEARED: ready for IR flash."));
    }
  }
}

void setup(){
  // Initialise driver OFF before enabling interrupts.
  pinMode(PIN_IN1,OUTPUT);pinMode(PIN_IN2,OUTPUT);
  pinMode(PIN_LASER,OUTPUT);pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LASER,LOW);
  digitalWrite(PIN_LED,LOW);
  emergencyMotorOff();
  for(uint8_t i=0;i<3;i++)pinMode(PIN_IR[i],INPUT);
  pinMode(PIN_ENC_A,INPUT);pinMode(PIN_ENC_B,INPUT);
  previousEncoderA=digitalRead(PIN_ENC_A);
  irPreviousPort=PIND&IR_MASK;
  PCMSK0|=_BV(PCINT1);PCICR|=_BV(PCIE0);
  PCMSK2|=_BV(PCINT18)|_BV(PCINT19)|_BV(PCINT20);
  PCICR|=_BV(PCIE2);

  Serial.begin(115200);
  unsigned long now=millis();
  speedSampleAt=lastMotorAt=now;
  resetRamCounters(now);
  enterStandby(now);
  Serial.println(F("MQ SENTINEL V9.1 NO EEPROM / 010+111 LASER / 101 HALF PWM"));
  Serial.println(F("T=telemetry D=status X=stop R=restart C=clear RAM+restart"));
  Serial.println(F("Boot pause 5s: motor and laser OFF. X to halt."));
  unsigned long bootAt=millis();
  while(millis()-bootAt<BOOT_DELAY_MS){
    now=millis();
    serialCommands(now);
    // Always keep motion disabled during initial pause.
    digitalWrite(PIN_LASER,LOW);
    emergencyMotorOff();
    if(mode==HALT)break;
  }
  if(mode!=HALT){
    runStart=millis();
    Serial.println(F("AUTO-START READY: waiting for first confirmed IR flash."));
  }
}

void loop(){
  unsigned long now=millis();
  serialCommands(now);
  readSensors(now);
  readWheel(now);
  if(mode==HALT){
    digitalWrite(PIN_LASER,LOW);
    emergencyMotorOff();
  } else {
    updateCounters(now);
    serviceController(now);
    serviceMotor(now);
  }
  digitalWrite(PIN_LED,(mode==FIRE||mode==SETTLE)?HIGH:LOW);
  showTelemetry(now);
}
