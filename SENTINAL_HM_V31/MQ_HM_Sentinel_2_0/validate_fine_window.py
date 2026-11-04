from pathlib import Path
import subprocess
import sys
old=Path(sys.argv[1]).read_text()
new=Path(__file__).with_name('MQ_HM_Sentinel_2_0.ino').read_text()
def fn(s,name):
    import re
    m=re.search(r'^(?:bool|float|void) '+name+r'\([^;\n]*\)\{',s,re.M)
    assert m,name
    i=s.index('{',m.start()); level=1;j=i+1
    while level:
        level+=(s[j]=='{')-(s[j]=='}');j+=1
    return s[m.start():j]
for n in ['fireTolerance','fireCentred','maintainLock','updateLaserArming','coarsePWM','trackingNeedsRecovery','searchMovingBackward']:
    assert fn(old,n)==fn(new,n),n+' changed unexpectedly'
preamble=r'''
#include <cmath>
#include <cassert>
#include <cstdio>
using std::fmin;using std::fmax;
float absf(float x){return std::fabs(x);}
float clampf(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
float signf(float x){return x>0?1:x<0?-1:0;}
bool fresh=true, calibrated=true, arcValid=true, pointingRateReady=true;
bool irRecent(unsigned long){return fresh;}
bool timeElapsed(unsigned long now,unsigned long since,unsigned long wait){return now-since>=wait;}
void updateHoldBias(unsigned long){}
void recordEvent(int){}
float fineHoldPWM(){return 17;}
float arcCentre=0,yaw=0,arcWidth=4,yawRateFiltered=0,pointingRate=0,wheelRPM=0,holdBias=0;
const float FIRE_ANGLE_MIN_TOL_DEG=.85f,FIRE_ANGLE_TOL_DEG=2.2f,FIRE_RATE_TOL_DPS=3.2f,HOLD_RATE_HARD_DPS=8,MAX_FIRE_WHEEL_RPM=185;
const float ENCODER_SIGN=-1,BODY_SIGN=-1,FINE_MIN_ERROR=.18f,FINE_PULSE_RATE_LIMIT_DPS=3.5f,FINE_GAIN_DEG_PER_DUTY_MS=.122f;
const int MOTOR_PERIOD_MS=5,EV_DAMP=1,EV_PULSE=2;
bool finePulseActive=false;
unsigned long finePulseAt=0,fineBrakeAt=0;
unsigned short finePulseDuration=0;
float finePulseDirection=0,finePulsePwm=0;
'''.replace('unsigned long','uint32_t')
code='#include <cstdint>\n'+preamble+'\n'+'\n'.join(fn(new,n) for n in ['fireTolerance','insideFireWindow','fireCentred','finePWM'])+r'''
int main(){
 yaw=-1; yawRateFiltered=3.3f;pointingRate=3.3f;
 assert(insideFireWindow(1000)); assert(!fireCentred(1000));
 assert(finePWM(1000)==17 && !finePulseActive); // no new pulse during rate ripple
 finePulseActive=true;finePulseAt=1000;finePulseDuration=20;finePulseDirection=1;
 assert(finePWM(1005)==17 && !finePulseActive); // stop a pulse already inside window
 yaw=-2; yawRateFiltered=0; fineBrakeAt=0;
 finePWM(2000); assert(finePulseActive); assert(finePulsePwm==65);
 assert(finePulseDuration>=5 && finePulseDuration<=20 && finePulseDuration%5==0);
 yaw=0; finePWM(2005); assert(!finePulseActive); // crossing into window ends pulse
 yaw=-1;pointingRate=0; assert(fireCentred(3000));
 fresh=false; assert(!fireCentred(3000));fresh=true;
 wheelRPM=186;assert(!fireCentred(3000));wheelRPM=0;
 yawRateFiltered=8;assert(!fireCentred(3000));yawRateFiltered=0;
 yaw=-2;assert(!fireCentred(3000));
 puts("PASS: pulse settling, pulse bounds, and unchanged IR/rate/angle/RPM firing gates");
}
'''
p=Path('/private/tmp/test_hm20.cpp');p.write_text(code)
subprocess.run(['clang++','-std=c++11',str(p),'-o','/private/tmp/test_hm20'],check=True)
subprocess.run(['/private/tmp/test_hm20'],check=True)
print('PASS: V35 firing, return, and recovery functions preserved exactly')
