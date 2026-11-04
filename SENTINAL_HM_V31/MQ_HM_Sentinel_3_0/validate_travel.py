"""Production travel controller checks and HM 2.5 precision protection.
Usage: python3 validate_travel.py PATH_TO_HM_2_5_INO
No physical motor/string response model is assumed.
"""
from pathlib import Path
import re, subprocess, sys, tempfile
s=Path(__file__).with_name('MQ_HM_Sentinel_3_0.ino').read_text()
old=Path(sys.argv[1]).read_text()
def fn(text,name):
    m=re.search(r'^(?:bool|float|void) '+name+r'\([^;\n]*\)\{',text,re.M);assert m,name
    i=text.index('{',m.start())+1;depth=1
    while depth:
        depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[m.start():i]
protected=['fineHoldPWM','updateHoldBias','finePWM','motorUpdate','fireTolerance','maintainLock','fireCentred','updateLaserArming','sequence','trackingNeedsRecovery','searchMovingBackward','limitSearchPWM','motorHardware','completedTargetDeparture','controlledSearchRecovery','finishArc']
for name in protected:
    assert fn(s,name)==fn(old,name),name+' changed'
assert fn(s,'enter').replace('  if(s!=CLEAR && s!=SEEK && s!=AIM)resetTravelSpeed();\n','')==fn(old,'enter')
# No changes to precision thresholds, gyro/encoder calibration, scan validation,
# braking/departure timing, actuator limits or existing speed guard constants.
for line in old.splitlines():
    if line.startswith('const '):assert line in s,line
names=['SEEK_CRUISE_DPS','BODY_SIGN','ENCODER_SIGN','FINE_ENTER_DEG','PIN_IN1']
constants=[]
for name in names:
    line=next(line for line in s.splitlines() if line.startswith('const ') and re.search(r'\b'+name+r'\s*=',line))
    if line not in constants:constants.append(line)
pre=r'''
#include <cstdint>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
float absf(float v){return std::fabs(v);}
float clampf(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
float signf(float v){return v>0?1:v<0?-1:0;}
uint32_t clockMs=0;uint32_t millis(){return clockMs;}
uint32_t travelTrimAt=0;uint8_t travelMode=0;
float travelTrim=0,yawRateFiltered=0,wheelRPM=0,arcCentre=0,yaw=0;
bool controlledDeparture=false,searchSpeedActive=false,searchSpeedRelease=false;
const int CLEAR=2,SEEK=3,SLOW_SCAN=7,AIM=9,FINE=10,FIRE=11;
int state=SEEK,seekDirection=-1;float seekLevel=100;
const int MOTOR_DRIVE=1,MOTOR_IDLE=0;int motorMode=0,pinPWM[16]={0};
void analogWrite(uint8_t pin,int value){assert(pin<16 && value>=0 && value<=255);pinPWM[pin]=value;}
'''
functions='\n'.join(fn(s,n) for n in ['resetTravelSpeed','travelRatePWM','cruisePWM','coarsePWM','motorHardware'])
functions+='\n'+fn(old,'coarsePWM').replace('coarsePWM()','originalCoarsePWM()')
checks=r'''
void reset(){
 clockMs=0;travelTrimAt=0;travelMode=0;travelTrim=0;
 yawRateFiltered=wheelRPM=0;controlledDeparture=searchSpeedActive=searchSpeedRelease=false;
 seekDirection=-1;state=SEEK;
}
int main(){
 // Search applies correctly signed reaction-wheel torque at rest and increases
 // bounded effort when measured speed remains below the 30 deg/s request.
 reset();float first=cruisePWM();assert(first==-120);
 float out=0;
 for(int i=0;i<2000;++i){clockMs+=5;out=cruisePWM();assert(out>=-150 && out<=150);}
 assert(out<first && travelTrim>20 && travelTrim<=60);
 // Faster than requested: correction changes to braking (zero wheel RPM here).
 reset();yawRateFiltered=50;assert(cruisePWM()>0);
 reset();yawRateFiltered=-20;assert(cruisePWM()==-150 && travelTrim==0);
 // Braking a moving body is a signed correction about wheel feedforward;
 // it need not command zero motor voltage or stop a spinning wheel.
 for(float rpm:{-100.f,100.f})for(int direction:{-1,1}){
   reset();wheelRPM=rpm;seekDirection=direction;
   float target=BODY_SIGN*direction*SEEK_CRUISE_DPS;
   yawRateFiltered=target>0?50:-50;
   float ff=ENCODER_SIGN*wheelRPM*255.f/157.f;
   float command=cruisePWM();
   assert(((command-ff)/BODY_SIGN)*(target-yawRateFiltered)>0);
   assert(fabs(command)>1 && fabs(command)<=255);
 }
 // Hardware mapping matches TI's PWM-mode input truth table. The zero
 // command is explicit HIGH/HIGH electrical wheel braking, not LOW/LOW coast.
 for(int p=-255;p<=255;++p){
   motorHardware(p);
   if(p>0)assert(pinPWM[PIN_IN1]==255 && pinPWM[PIN_IN2]==255-p);
   else if(p<0)assert(pinPWM[PIN_IN2]==255 && pinPWM[PIN_IN1]==255+p);
   else assert(pinPWM[PIN_IN1]==255 && pinPWM[PIN_IN2]==255);
 }
 // Changing requested direction clears stale integral torque immediately.
 reset();for(int i=0;i<100;++i){clockMs+=5;cruisePWM();}
 assert(travelTrim>0);seekDirection=1;yawRateFiltered=0;
 assert(cruisePWM()==120 && travelTrim==0);
 // Guard/departure overrides and voltage saturation do not wind up the trim.
 for(int mode=0;mode<3;++mode){
   reset();controlledDeparture=mode==0;searchSpeedActive=mode==1;searchSpeedRelease=mode==2;
   for(int i=0;i<1000;++i){clockMs+=5;cruisePWM();}assert(travelTrim==0);
 }
 reset();wheelRPM=180;
 for(int i=0;i<1000;++i){clockMs+=5;assert(cruisePWM()==-255);}
 assert(travelTrim==0);
 reset();controlledDeparture=true;assert(cruisePWM()==-100 && travelMode==0);
 // Sweep motor command remains the old signed fixed duty, not search PI.
 state=SLOW_SCAN;seekLevel=117;assert(cruisePWM()==-117);
 // Coarse AIM builds load compensation above the boundary, fading to zero.
 reset();state=AIM;yaw=-4.2f;float start=coarsePWM();
 for(int i=0;i<400;++i){clockMs+=5;out=coarsePWM();}
 assert(out<start && travelTrim>0 && travelTrim<=12.51f);
 yaw=-3.3f;clockMs+=5;coarsePWM();assert(absf(travelTrim)<=1.251f);
 // Protected near-centre AIM is numerically identical to HM 2.5, including
 // speed feedback and wheel feedforward, even after a large travel trim.
 for(float error:{-3.2f,-2.f,-.5f,0.f,.5f,2.f,3.2f})
 for(float rate:{-15.f,0.f,15.f})
 for(float rpm:{-170.f,0.f,170.f}){
   yaw=-error;yawRateFiltered=rate;wheelRPM=rpm;travelTrim=60;travelMode=2;
   float expected=originalCoarsePWM(),actual=coarsePWM();
   assert(fabs(expected-actual)<.0001f && travelTrim==0 && travelMode==0);
 }
 puts("PASS: travel torque sign, speed error response, bounded load compensation, anti-windup, direction reset, scan preservation, spinning-wheel braking correction, hardware pin mapping and protected near-centre AIM");
}
'''
with tempfile.TemporaryDirectory(prefix='hm30-travel-') as d:
    cpp=Path(d)/'check.cpp';exe=Path(d)/'check'
    cpp.write_text(pre+'\n'+'\n'.join(constants)+'\n'+functions+'\n'+checks)
    subprocess.run(['clang++','-std=c++11',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: HM 2.5 fine/hold/firing/departure functions and all existing constants unchanged')
