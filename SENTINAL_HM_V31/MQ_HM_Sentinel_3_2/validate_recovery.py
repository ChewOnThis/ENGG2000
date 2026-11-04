"""Production body-brake and wheel-headroom recovery checks.
Usage: python3 validate_recovery.py PATH_TO_HM_3_1_INO
Hardware/plant response is not simulated; pin-level driver behavior is retained.
"""
from pathlib import Path
import re,subprocess,sys,tempfile
s=Path(__file__).with_name('MQ_HM_Sentinel_3_2.ino').read_text();old=Path(sys.argv[1]).read_text()
def fn(text,name):
    m=re.search(r'^(?:void|bool|float) '+name+r'\([^;\n]*\)\{',text,re.M);assert m,name
    i=text.index('{',m.start())+1;depth=1
    while depth:
        depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[m.start():i]
for n in ['finePWM','fineHoldPWM','updateHoldBias','fireTolerance','fireCentred','maintainLock','updateLaserArming','controlledSearchRecovery','completedTargetDeparture','restartSearch','travelRatePWM','cruisePWM','firstIrEncounter','motorHardware','trackingNeedsRecovery']:
    assert fn(s,n)==fn(old,n),n+' changed'
for line in old.splitlines():
    if line.startswith('const '):assert line in s,line
seq=fn(s,'sequence');prev=fn(old,'sequence')
assert seq.split('    case FIRE:',1)[1]==prev.split('    case FIRE:',1)[1]
assert fn(s,'enter').replace('  plateauAt=0;\n','')==fn(old,'enter')
assert fn(s,'motorUpdate').split('  if(controlledDeparture && state==UNWIND)',1)[0].replace('  else if(state==ARC_BRAKE)\n    command=pwmApplied+clampf(ratePWM(0,90.0f)-pwmApplied,-2.0f,2.0f);\n','')==fn(old,'motorUpdate').split('  if(controlledDeparture && state==UNWIND)',1)[0]
pre=r'''
#include <cstdint>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
float absf(float x){return std::fabs(x);}
float signf(float x){return x>0?1:x<0?-1:0;}
float clampf(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
bool timeElapsed(uint32_t now,uint32_t since,uint32_t wait){return now-since>=wait;}
const int BOOT=0,CLEAR=2,SEEK=3,SLOW_SCAN=7,ARC_BRAKE=8,AIM=9,FINE=10,FIRE=11,UNWIND=12,STOP=13;
const int EV_SAT_ON=7;
const float BODY_SIGN=-1,ENCODER_SIGN=-1,MOTOR_PWM_LIMIT=255,FINE_ENTER_DEG=3.2f,SEEK_CRUISE_DPS=30;
const int MOTOR_PERIOD_MS=5;
const float DEPARTURE_ACCEL_DPS2=15,DEPARTURE_PWM_PER_SEC=250;
int state=SEEK,seekDirection=-1;int16_t pwmApplied=0,hardwarePWM=0;
float yawRateFiltered=0,pointingRate=0,wheelRPM=0,arcCentre=0,yaw=0,plateauRPM=0;
float departureReference=0,departureApplied=0;
uint32_t plateauAt=0,lastMotorMs=0,phaseStillAt=0,enteredMs=0,clockMs=0;
bool controlledDeparture=false,searchSpeedActive=false,searchSpeedRelease=false,pointingRateReady=true;
bool recovered=false,reverseCalled=false;int saturationEvents=0;
void controlledSearchRecovery(bool reverse){recovered=true;reverseCalled=reverse;if(reverse)seekDirection=-seekDirection;controlledDeparture=true;state=UNWIND;}
void recordEvent(int event){if(event==EV_SAT_ON)++saturationEvents;}
void enter(int value){state=value;enteredMs=clockMs;plateauAt=0;}
float cruisePWM(){return 100;}
float coarsePWM(){return 20;}
float finePWM(uint32_t){return 17;}
float limitSearchPWM(uint32_t,float p,float){return p;}
void motorHardware(int16_t p){hardwarePWM=p;}
'''
functions='\n'.join(fn(s,n) for n in ['ratePWM','aimSpeed','travelSaturated','recoverWheelMomentum','motorUpdate'])
head=seq.split('  if(searchMovingBackward',1)[0].replace('void sequence(uint32_t now){','void checkRecovery(uint32_t now){')
arc=seq.split('    case ARC_BRAKE:',1)[1].split('    case AIM:',1)[0]
functions+='\n'+head+'switch(state){case ARC_BRAKE:'+arc+'default:break;}}'
test=r'''
void reset(){
 state=SEEK;seekDirection=-1;pwmApplied=hardwarePWM=0;
 yawRateFiltered=pointingRate=wheelRPM=arcCentre=yaw=plateauRPM=0;
 plateauAt=lastMotorMs=phaseStillAt=enteredMs=clockMs=0;
 controlledDeparture=searchSpeedActive=searchSpeedRelease=recovered=reverseCalled=false;
 pointingRateReady=true;saturationEvents=0;
}
int main(){
 // Flat wheel RPM alone is not enough; require high drive and missed rate.
 reset();wheelRPM=148;pwmApplied=-255;yawRateFiltered=30;
 assert(!travelSaturated(1000) && !travelSaturated(2000));
 yawRateFiltered=20;assert(!travelSaturated(2100));assert(!travelSaturated(2599));assert(travelSaturated(2600));
 // Continued acceleration restarts the window instead of claiming saturation.
 reset();wheelRPM=80;pwmApplied=-255;yawRateFiltered=0;
 assert(!travelSaturated(1000));wheelRPM=83;assert(!travelSaturated(1400));
 assert(!travelSaturated(1899));assert(travelSaturated(1900));
 // The detector works at lower plateaus and both signs, not a fixed 150 RPM.
 reset();seekDirection=1;wheelRPM=-80;pwmApplied=255;yawRateFiltered=0;
 assert(!travelSaturated(1000));assert(travelSaturated(1500));
 for(int excluded:{FINE,FIRE}){state=excluded;assert(!travelSaturated(2000) && plateauAt==0);}
 reset();wheelRPM=148;pwmApplied=-240;yawRateFiltered=0;assert(!travelSaturated(1000));
 pwmApplied=-255;controlledDeparture=true;assert(!travelSaturated(2000));controlledDeparture=false;
 searchSpeedActive=true;assert(!travelSaturated(3000));
 // Recorded plateau replay: predict a recovery before the observed reversal.
 reset();pwmApplied=-255;
 const uint32_t times[]={70950,71150,71350,71550,71750,71950,72150,72350,72550,72750,72950,73150,73360};
 const float body[]={28.60f,26.43f,26.59f,27.24f,26.56f,24.71f,23.70f,24.27f,21.93f,20.94f,19.48f,18.92f,18.13f};
 const float rpm[]={147.7f,147.9f,147.7f,148.2f,148.6f,148.0f,148.3f,148.9f,148.3f,147.7f,148.8f,148.7f,148.3f};
 uint32_t first=0;
 for(int i=0;i<13;++i){yawRateFiltered=body[i];wheelRPM=rpm[i];if(travelSaturated(times[i])){first=times[i];break;}}
 assert(first==73150); // replay only; not a future physical timing guarantee
 // Wheel momentum, not nominal search direction, chooses the unload direction.
 for(float wheel:{-148.f,148.f})for(int direction:{-1,1}){
   reset();wheelRPM=wheel;seekDirection=direction;recoverWheelMomentum();
   assert(recovered && controlledDeparture && state==UNWIND);
   assert(seekDirection==(wheel>0?1:-1));
 }
 // Actual motor dispatcher ramps ARC_BRAKE rather than forcing motor zero.
 for(float wheel:{-123.f,123.f})for(float rate:{-30.f,30.f}){
   reset();state=ARC_BRAKE;wheelRPM=wheel;yawRateFiltered=rate;pwmApplied=wheel>0?-213:213;
   for(int i=1;i<=260;++i){int before=pwmApplied;motorUpdate(i*5);assert(abs(pwmApplied-before)<=2);}
   float ff=ENCODER_SIGN*wheelRPM*255.f/157.f;
   assert(((pwmApplied-ff)/BODY_SIGN)*rate<0); // opposing body-rate correction
   assert(pwmApplied!=0); // wheel feedforward remains, not wheel stop
 }
 // A stopped body with a spinning wheel receives maintaining feedforward.
 reset();state=ARC_BRAKE;wheelRPM=100;pwmApplied=-162;motorUpdate(5);assert(pwmApplied==-162);
 // Handoff waits for low slow-rate estimate and bounded fast ripple, 100 ms.
 reset();state=ARC_BRAKE;enteredMs=1000;pointingRate=1;yawRateFiltered=6;
 clockMs=1000;checkRecovery(1000);assert(state==ARC_BRAKE);
 clockMs=1099;checkRecovery(1099);assert(state==ARC_BRAKE);
 clockMs=1100;checkRecovery(1100);assert(state==AIM);
 // Timeout uses the same ramped momentum-aware recovery, not an instant flip.
 reset();state=ARC_BRAKE;enteredMs=1000;pointingRate=12;yawRateFiltered=15;wheelRPM=120;
 checkRecovery(2499);assert(!recovered);checkRecovery(2500);assert(recovered && seekDirection==1);
 // New headroom marker is emitted before invoking existing recovery.
 reset();wheelRPM=148;pwmApplied=-255;yawRateFiltered=0;
 checkRecovery(1000);checkRecovery(1500);assert(saturationEvents==1 && recovered);
 puts("PASS: plateau/maximum-drive detection, recorded replay, unload directions, bounded active body braking and settled/timeout handoff");
}
'''
with tempfile.TemporaryDirectory(prefix='hm32-recovery-') as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(pre+'\n'+functions+'\n'+test)
    subprocess.run(['clang++','-std=c++11',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: fine alignment, hold, firing, IR-entry speed continuity and existing ramped recovery retained')
