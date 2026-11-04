"""Exercise production FINE/FIRE motor dispatch and independent centering.
Usage: python3 validate_centering.py PATH_TO_HM_2_4_INO
Uses hardware stubs; does not simulate physical pointing response.
"""
from pathlib import Path
import re, subprocess, sys, tempfile
source=Path(__file__).with_name('MQ_HM_Sentinel_2_5.ino').read_text()
previous=Path(sys.argv[1]).read_text()
def fn(text,name):
    m=re.search(r'^(?:bool|float|void) '+name+r'\([^;\n]*\)\{',text,re.M);assert m,name
    i=text.index('{',m.start())+1;depth=1
    while depth:
        depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[m.start():i]
for name in ['fineHoldPWM','updateHoldBias','motorUpdate','fireTolerance','maintainLock','fireCentred','sequence','recordEvent','recorderService','recorderInit','recorderCommands','recorderObserve']:
    assert fn(source,name).replace('HM2.5','HM2.4')==fn(previous,name),name+' unexpectedly changed'
for name in ['FIRE_ANGLE_TOL_DEG','FIRE_ANGLE_MIN_TOL_DEG','FIRE_HOLD_EXTRA_DEG','FIRE_RATE_TOL_DPS','FIRE_CONFIRM_MS','FIRE_IR_HOLD_MS','FIRE_HOLD_RATE_DPS','HOLD_RATE_HARD_DPS','MAX_FIRE_WHEEL_RPM']:
    pattern=r'\b'+name+r'\s*=\s*([^,;]+)'
    assert re.search(pattern,source)[1]==re.search(pattern,previous)[1],name
names=['FINE_MIN_ERROR','FINE_PULSE_RATE_LIMIT_DPS','FINE_GAIN_DEG_PER_DUTY_MS','MOTOR_PERIOD_MS','ENCODER_SIGN','BODY_SIGN','MOTOR_PWM_LIMIT','DEPARTURE_ACCEL_DPS2','DEPARTURE_PWM_PER_SEC','FIRE_ANGLE_TOL_DEG','FIRE_ANGLE_MIN_TOL_DEG','MAX_FIRE_WHEEL_RPM','HOLD_RATE_HARD_DPS']
constants=[]
for name in names:
    line=next(line for line in source.splitlines() if line.startswith('const ') and re.search(r'\b'+name+r'\s*=',line))
    if line not in constants:constants.append(line)
pre=r'''
#include <cstdint>
#include <cmath>
#include <cassert>
#include <cstdio>
float absf(float x){return std::fabs(x);}
float clampf(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
float signf(float x){return x>0?1:x<0?-1:0;}
bool timeElapsed(uint32_t now,uint32_t since,uint32_t wait){return now-since>=wait;}
const int BOOT=0,CLEAR=2,SEEK=3,SLOW_SCAN=7,ARC_BRAKE=8,AIM=9,FINE=10,FIRE=11,UNWIND=12,STOP=13;
const int EV_DAMP=4,EV_PULSE=3;
int state=FINE;
bool calibrated=true,arcValid=true,pointingRateReady=true,laserHigh=true;
float arcCentre=0,yaw=0,arcWidth=4,yawRateFiltered=0,pointingRate=0,wheelRPM=0,holdBias=0;
bool irRecent(uint32_t){return true;}
bool finePulseActive=false,controlledDeparture=false;
uint32_t finePulseAt=0,fineBrakeAt=0,lastMotorMs=0;
uint16_t finePulseDuration=0;
float finePulseDirection=0,finePulsePwm=0,departureReference=0,departureApplied=0;
int16_t pwmApplied=0,hardwarePwm=0;
int updates=0;
void updateHoldBias(uint32_t){} // unchanged production bias function checked above
void recordEvent(uint8_t){}
float coarsePWM(){return 0;}
float cruisePWM(){return 0;}
float ratePWM(float,float){return 0;}
float limitSearchPWM(uint32_t,float p,float){return p;} // FIRE/FINE do not search
void motorHardware(int16_t p){hardwarePwm=p;++updates;}
'''
body='\n'.join(fn(source,name) for name in ['fireTolerance','fireCentred','fineHoldPWM','finePWM','motorUpdate'])
checks=r'''
void reset(int s,float error){
 state=s;yaw=-error;yawRateFiltered=pointingRate=wheelRPM=holdBias=0;
 finePulseActive=false;finePulseAt=fineBrakeAt=lastMotorMs=0;
 pwmApplied=hardwarePwm=0;updates=0;laserHigh=true;
}
int main(){
 for(int s : {FINE,FIRE}){
   // Laser-qualified position still receives a correction toward centre.
   reset(s,.5f);assert(fireCentred(1000));motorUpdate(1000);
   assert(laserHigh && finePulseActive && hardwarePwm<0 && finePulsePwm==45);
   assert(finePulseDuration>=5 && finePulseDuration<=20 && finePulseDuration%5==0);
   motorUpdate(1001);assert(updates==1); // respects motor service period
   motorUpdate(1005);assert(updates==2 && laserHigh);
   // Opposite drift gets opposite correction; no acceptance plateau at .9.
   reset(s,-.5f);assert(fireCentred(1000));motorUpdate(1000);
   assert(finePulseActive && hardwarePwm>0 && laserHigh);
   reset(s,.85f);assert(fireCentred(1000));motorUpdate(1000);assert(finePulseActive);
   // Reaching the inner tolerance ends a pulse without ending the shot.
   yaw=-.1f;motorUpdate(1005);assert(!finePulseActive && laserHigh);
   // Inside the pulse tolerance, active hold still damps rotation.
   reset(s,.1f);yawRateFiltered=1;motorUpdate(1000);
   assert(!finePulseActive && hardwarePwm>0 && laserHigh);
   reset(s,-.1f);yawRateFiltered=-1;motorUpdate(1000);
   assert(!finePulseActive && hardwarePwm<0 && laserHigh);
   // Between pulses: no rapid retrigger, while hold output still updates.
   reset(s,.5f);motorUpdate(1000);motorUpdate(1020);assert(!finePulseActive);
   motorUpdate(1100);assert(!finePulseActive && laserHigh);
   motorUpdate(1200);assert(finePulseActive && laserHigh);
   // High body rate inhibits a new pulse; damping remains active.
   reset(s,.5f);yawRateFiltered=4;motorUpdate(1000);
   assert(!finePulseActive && hardwarePwm>0 && laserHigh);
 }
 reset(FIRE,.91f);assert(!fireCentred(1000));
 puts("PASS: independent +/-0.18 centering, +/-0.9 laser entry, bidirectional FINE/FIRE corrections, pulse limits/spacing and active damping during laser command");
}
'''
# Include initializer_list explicitly for portable C++11 host compilation.
pre='#include <initializer_list>\n'+pre
with tempfile.TemporaryDirectory(prefix='hm25-centering-') as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test'
    cpp.write_text(pre+'\n'+'\n'.join(constants)+'\n'+body+'\n'+checks)
    subprocess.run(['clang++','-std=c++11',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: HM 2.4 firing gates, hold control, departure and EEPROM implementation unchanged')
