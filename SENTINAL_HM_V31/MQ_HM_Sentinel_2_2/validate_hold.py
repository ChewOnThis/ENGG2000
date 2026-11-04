"""Run extracted production control/gate functions with mocked hardware.
Usage: python3 validate_hold.py PATH_TO_HM_2_1_INO
Requires clang++; does not model physical suspension dynamics.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile
source = Path(__file__).with_name('MQ_HM_Sentinel_2_2.ino').read_text()
previous = Path(sys.argv[1]).read_text()
def function(text, name):
    match = re.search(r'^(?:bool|float|void) '+name+r'\([^;\n]*\)\{', text, re.M)
    assert match, name
    i = text.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (text[i] == '{') - (text[i] == '}')
        i += 1
    return text[match.start():i]
for name in ['coarsePWM', 'trackingNeedsRecovery', 'searchMovingBackward', 'insideFireWindow', 'finePWM', 'fireTolerance', 'fireCentred', 'maintainLock', 'updateLaserArming']:
    assert function(source, name) == function(previous, name), name
names = ['HOLD_KI', 'HOLD_POSITION_KP', 'FIRE_FAST_RATE_DPS', 'FIRE_ANGLE_TOL_DEG',
         'FIRE_ANGLE_MIN_TOL_DEG', 'FIRE_HOLD_EXTRA_DEG', 'FIRE_CONFIRM_MS',
         'MAX_FIRE_WHEEL_RPM', 'HOLD_RATE_CONFIRM_MS', 'HOLD_RATE_HARD_DPS',
         'BODY_SIGN', 'ENCODER_SIGN', 'FINE_MIN_ERROR', 'FINE_PULSE_RATE_LIMIT_DPS',
         'FINE_GAIN_DEG_PER_DUTY_MS', 'MOTOR_PERIOD_MS', 'FINE_REENTER_DEG']
constants = []
for name in names:
    line = next(line for line in source.splitlines() if line.startswith('const ') and re.search(r'\b'+name+r'\s*=', line))
    if line not in constants:
        constants.append(line)
preamble = r'''
#include <cstdint>
#include <cmath>
#include <cassert>
#include <cstdio>
#include <algorithm>
using std::min; using std::max;
#define F(x) x
struct SerialMock { void println(const char*){} } Serial;
float absf(float x){return std::fabs(x);}
float clampf(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
float signf(float x){return x>0?1:x<0?-1:0;}
bool fresh=true,calibrated=true,arcValid=true,pointingRateReady=true,irStuckReported=false;
struct {bool active=true;} ir;
bool irRecent(uint32_t){return fresh;}
bool timeElapsed(uint32_t now,uint32_t since,uint32_t wait){return now-since>=wait;}
void recordEvent(int){}
const int EV_DAMP=1,EV_PULSE=2,EV_LOCK_LOST=3,FIRE=11;
int state=10,fires=0;
int16_t pwmApplied=0;
uint32_t stableSince=0,lastLockIrAt=0,rateViolationAt=0,holdBiasAt=0;
uint16_t longestGateMs=0;
bool finePulseActive=false,laserHigh=false;
void laserOn(){laserHigh=true;}
void enter(int s){state=s;finePulseActive=false;stableSince=0;}
uint32_t finePulseAt=0,fineBrakeAt=0;
uint16_t finePulseDuration=0;
float finePulseDirection=0,finePulsePwm=0;
float arcCentre=0,yaw=0,arcWidth=4,yawRateFiltered=0,pointingRate=0,wheelRPM=0,holdBias=0;
'''
body = '\n'.join(function(source, name) for name in [
    'fireTolerance','insideFireWindow','holdError','fineHoldPWM','updateHoldBias','finePWM',
    'maintainLock','fireCentred','updateLaserArming'])
tests = r'''
int main(){
 // Quiet position band is continuous and retains learned balancing bias.
 yaw=-.15f;assert(holdError()==0 && fineHoldPWM()==0);
 yaw=.15f;assert(holdError()==0 && fineHoldPWM()==0);
 holdBias=1;pwmApplied=-1;assert(fabs(fineHoldPWM()+1)<.001f);
 holdBias=0;pwmApplied=0;
 yaw=-.3f;float small=fineHoldPWM();yaw=-.6f;float large=fineHoldPWM();
 assert(small<0 && large<small && fabs(large/small-4)<.001f);
 yaw=0;yawRateFiltered=1;pwmApplied=4;assert(fabs(fineHoldPWM()-4)<.001f);
 yawRateFiltered=-1;pwmApplied=-4;assert(fabs(fineHoldPWM()+4)<.001f);
 // High measured ripple cannot create a sudden near-centre voltage jump.
 pwmApplied=0;yawRateFiltered=10;
 for(int i=0;i<25;++i){float out=fineHoldPWM();assert(out-pwmApplied<=2.001f && out-pwmApplied>=-2.001f);pwmApplied=(int16_t)out;}
 assert(pwmApplied==40);
 yawRateFiltered=-10;assert(fineHoldPWM()==38);
 pwmApplied=0;
 // Near-centre rate ripple cannot retrigger torque pulses.
 yaw=-1;yawRateFiltered=3.3f;pointingRate=3.3f;
 finePWM(1000);assert(!finePulseActive && !fireCentred(1000));
 finePulseActive=true;finePulseAt=1000;finePulseDuration=20;finePulseDirection=1;
 finePWM(1005);assert(!finePulseActive);
 // Pulses still operate outside the positional window.
 yaw=-2;yawRateFiltered=0;fineBrakeAt=0;finePWM(2000);assert(finePulseActive);
 assert(finePulseDuration>=5 && finePulseDuration<=20 && finePulseDuration%5==0);
 yaw=0;finePWM(2005);assert(!finePulseActive);
 // Integral bias holds steady inside the quiet band, learns outside, stays bounded.
 yaw=-.1f;holdBias=1;holdBiasAt=2980;updateHoldBias(3000);assert(holdBias==1);
 // Integral bias learns slowly and remains bounded.
 yaw=-1;holdBias=0;holdBiasAt=3000;updateHoldBias(3020);
 assert(holdBias>0 && holdBias<=.017f);
 holdBias=20;holdBiasAt=3020;updateHoldBias(3040);assert(holdBias==20);
 holdBias=0;yaw=0;pointingRate=1.01f;assert(!fireCentred(4000));
 pointingRate=0;yawRateFiltered=2.01f;assert(!fireCentred(4000));
 yawRateFiltered=0;assert(fireCentred(4000));
 // Confirmation needs uninterrupted 140 ms; interrupted settling resets it.
 assert(!updateLaserArming(4000));assert(!updateLaserArming(4139));
 pointingRate=1.1f;assert(!updateLaserArming(4140));assert(stableSince==0);
 pointingRate=0;assert(!updateLaserArming(4200));assert(!updateLaserArming(4339));
 assert(updateLaserArming(4340));assert(laserHigh && fires==1 && state==FIRE);
 // IR, wheel and position criteria still reject invalid entry.
 fresh=false;assert(!fireCentred(4400));fresh=true;
 wheelRPM=186;assert(!fireCentred(4400));wheelRPM=0;
 yaw=-3;assert(!fireCentred(4400));yaw=0;
 // Hold tolerates brief ripple but ends for persistent motion or high speed.
 pointingRate=1.6f;yawRateFiltered=1;assert(maintainLock(5000));
 assert(maintainLock(5039));assert(!maintainLock(5040));
 pointingRate=0;assert(maintainLock(5050));assert(rateViolationAt==0);
 yawRateFiltered=4;assert(!maintainLock(5055));
 yawRateFiltered=0;fresh=false;ir.active=false;assert(!maintainLock(5060));
 puts("PASS: quiet band, gradual restoring, voltage slew, bias, pulses, firing confirmation and hold gates");
}
'''
with tempfile.TemporaryDirectory(prefix='hm21-check-') as directory:
    cpp=Path(directory)/'check.cpp';binary=Path(directory)/'check'
    cpp.write_text(preamble+'\n'+'\n'.join(constants)+'\n'+body+'\n'+tests)
    subprocess.run(['clang++','-std=c++11',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('PASS: coarse return, recovery, pulse algorithm and strict firing/hold gate functions preserved')
