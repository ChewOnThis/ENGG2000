"""Host checks for actual production control and EEPROM functions.
Usage: python3 validate.py PATH_TO_ORIGINAL_V35_INO
Requires clang++; hardware EEPROM timing and physical aiming are not simulated.
"""
from pathlib import Path
import re, subprocess, sys, tempfile
s=Path(__file__).with_name('MQ_HM_Sentinel_2_4.ino').read_text()
old=Path(sys.argv[1]).read_text()
def fn(text,name):
    m=re.search(r'^(?:bool|float|void|uint8_t|int16_t) '+name+r'\([^;\n]*\)\{',text,re.M)
    assert m,name
    i=text.index('{',m.start())+1;depth=1
    while depth:
        depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[m.start():i]
for n in ['fineHoldPWM','updateHoldBias','finePWM','coarsePWM','centred','fireCentred','trackingNeedsRecovery','searchMovingBackward']:
    assert fn(s,n)==fn(old,n),n
assert 'completedTargetDeparture();' in fn(s,'sequence')
assert 'targetHoldComplete' not in s
# Production logger constants, globals and queue, not a duplicate layout.
a=s.index('const uint8_t LOG_FORMAT=');b=s.index('void laserOff()',a)
declarations=s[a:b]
pre=r'''
#include <cstdint>
#include <cassert>
#include <cmath>
#include <string>
#include <algorithm>
#include <array>
#include <iostream>
using std::min;using std::max;
#define E2END 1023
#define F(x) x
#define _BV(x) (1U<<(x))
const int EEPE=0;int EECR=0;
uint32_t clockMs=0;uint32_t millis(){return clockMs;}
struct Memory {
 std::array<uint8_t,1024> bytes;unsigned high=0;
 Memory(){bytes.fill(255);}
 uint8_t read(unsigned a){assert(a<1024);high=max(high,a);return bytes[a];}
 void update(unsigned a,uint8_t v){assert(a<1024);high=max(high,a);bytes[a]=v;}
} EEPROM;
struct Port {
 std::string output;char input=0;
 int available(){return input!=0;}
 char read(){char c=input;input=0;return c;}
 void print(const char *s){output+=s;}
 void print(char c){output+=c;}
 template<class T>void print(T v){output+=std::to_string(v);}
 template<class T>void println(T v){print(v);output+='\n';}
 void println(){output+='\n';}
} Serial;
const int STOP=13,BOOT=0,AIM=9,FINE=10,FIRE=11,UNWIND=12;
uint8_t state=FINE,stopCode=0;
bool finePulseActive=false,arcValid=true,laserHigh=false,calibrated=true,pointingRateReady=true;
float arcCentre=0,yaw=0,yawRateFiltered=0,pointingRate=0,wheelRPM=0,holdBias=0;
int16_t pwmApplied=0;
const float FIRE_RATE_TOL_DPS=3.2f,HOLD_RATE_HARD_DPS=8,MAX_FIRE_WHEEL_RPM=185;
float absf(float x){return std::fabs(x);}
float clampf(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
float fireTolerance(){return .9f;}
bool fresh=true,irStuckReported=false;
struct {bool active=true;} ir;
uint32_t lastLockIrAt=0,rateViolationAt=0,fireStartedAt=0;
bool departed=false;
bool irRecent(uint32_t){return fresh;}
bool timeElapsed(uint32_t now,uint32_t since,uint32_t wait){return now-since>=wait;}
void enter(int value){state=value;}
void completedTargetDeparture(){departed=true;state=UNWIND;}
void motorOff(){pwmApplied=0;}
void laserOff(){laserHigh=false;}
bool writeEEPROMByte(uint16_t a,uint8_t v){EEPROM.update(a,v);return true;}
bool readRecord(uint8_t,uint8_t*);
'''
control_constants='\n'.join(line for line in s.splitlines() if line.startswith('const ') and any(name in line for name in ['FIRE_HOLD_EXTRA_DEG=', 'FIRE_CONFIRM_MS=', 'HOLD_RATE_CONFIRM_MS=']))
fire_section=fn(s,'sequence').split('    case FIRE:',1)[1].split('    case STOP:',1)[0]
fire_function='void checkFire(uint32_t now){switch(state){case FIRE:'+fire_section+'default:break;}}'
funcs='\n'.join(fn(s,n) for n in ['recordCRC','ringEvent','packSigned','entryFlags','recordEvent','recorderService','logFormatOK','readSigned','readRecord','recorderInit','recorderCommands','recorderObserve','maintainLock'])
checks=r'''
void resetRAM(){
 clockMs=0;recordStarted=0;recording=false;saved=summarySaved=sampleNext=qHead=qTail=qSize=writeByte=0;
 dropped=0;logPulsePair=false;speedLogEvents=0;finePulseActive=false;state=BOOT;Serial.output.clear();
}
void drain(){int guard=1000;while(qSize && --guard)recorderService();assert(guard>0);}
void send(char c){Serial.input=c;recorderCommands();}
int16_t word(const uint8_t*b,int i){return (int16_t)(b[i]|((uint16_t)b[i+1]<<8));}
void take(uint32_t t){clockMs=t;recordEvent(EV_AIM_WINDOW);drain();}
void sortedDump(){
 send('D');std::string out=Serial.output;size_t p=out.find("time_ms,event,");assert(p!=std::string::npos);
 p=out.find('\n',p)+1;long last=-1;
 while(p<out.size() && out[p]>='0' && out[p]<='9'){
   auto end=out.find('\n',p);long t=std::stol(out.substr(p,end-p));assert(t>=last);last=t;p=end+1;
 }
}
int main(){
 // Upload/init of a different format preserves old bytes until explicit A.
 auto untouched=EEPROM.bytes;recorderInit();assert(EEPROM.bytes==untouched && state==STOP);
 send('A');assert(logFormatOK() && EEPROM.read(3)==0xA1);
 resetRAM();recorderInit();assert(recording && EEPROM.read(3)==0);state=FINE;
 // Exact signed measurements, exact full-range PWM and 10 ms timestamps.
 yaw=-1.25f;yawRateFiltered=-2.5f;pointingRate=1.75f;wheelRPM=-12.5f;holdBias=-3.25f;
 uint8_t b[LOG_BYTES];
 for(int p=-255;p<=255;++p){
   pwmApplied=p;take(1000+(p+255)*200);
   assert(readRecord((sampleNext+LOG_SAMPLES-1)%LOG_SAMPLES,b));
   assert(word(b,2)==125 && word(b,4)==-250 && word(b,6)==175);
   assert(word(b,8)==-125 && word(b,10)==-325);
   assert((int)(b[12]|((uint16_t)(b[13]&128)<<1))-255==p);
   assert((b[13]&15)==FINE && ((b[13]>>4)&7)==0);
 }
 assert(summarySaved==40 && EEPROM.high<=1018);
 // Samples survive reboot and D prints direct columns in chronological order.
 resetRAM();recorderInit();assert(!recording && summarySaved==40);sortedDump();
 assert(Serial.output.find("125,-250,175,-125,")!=std::string::npos);
 // All single-bit record corruptions are rejected by CRC or commit marker.
 unsigned slot=(sampleNext+39)%40,base=LOG_BASE+slot*LOG_BYTES;
 auto good=EEPROM.bytes;
 for(int byte=0;byte<LOG_BYTES;++byte)for(int bit=0;bit<8;++bit){
   EEPROM.bytes=good;EEPROM.bytes[base+byte]^=1<<bit;assert(!readRecord(slot,b));
 }
 EEPROM.bytes=good;
 // Interrupted full-ring writes: retain old record or reject partial record,
 // never expose mixed values; rebuild order after payload/pointer interruption.
 for(int cut=0;cut<=23;++cut){
   EEPROM.bytes=good;resetRAM();recorderInit();recording=true;recordStarted=0;
   const uint8_t overwriting=sampleNext;clockMs=200000;recordEvent(EV_AIM_WINDOW);
   for(int i=0;i<cut && qSize;++i)recorderService();
   bool valid=readRecord(overwriting,b);
   if(cut>=1 && cut<=18)assert(!valid);
   else {assert(valid);if(cut>=19)assert((b[0]|((uint16_t)b[1]<<8))==20000);}
   resetRAM();recorderInit();sortedDump();
 }
 // Separate event quotas reserve five slots; samples still roll when full.
 send('A');resetRAM();recorderInit();state=FINE;
 for(int i=0;i<14;++i){clockMs=1000+i*100;recordEvent(EV_RECOVER);drain();}
 assert(saved==14);recordEvent(EV_RECOVER);assert(dropped==1);
 for(int i=0;i<5;++i){recordEvent(EV_FAULT);drain();}assert(saved==19);
 take(5000);assert(summarySaved==1 && saved==19);
 // A full queue reports drops without writing beyond allocated storage.
 for(int i=0;i<5;++i)recordEvent(EV_AIM_WINDOW);
 assert(qSize==4 && dropped==2);drain();assert(EEPROM.read(6)==2);
 // Fine pulses defer EEPROM writes; committing resumes after pulse end.
 recordEvent(EV_AIM_WINDOW);finePulseActive=true;auto before=EEPROM.bytes;
 recorderService();assert(EEPROM.bytes==before);finePulseActive=false;drain();
 // Stopping/dumping does not arm or clear a completed log. CRC failures reported.
 EEPROM.bytes[LOG_BASE+2]^=1;Serial.output.clear();sortedDump();
 assert(Serial.output.find("bad_crc=1")!=std::string::npos && EEPROM.read(3)==0);

 assert(LOG_BASE+(LOG_SAMPLES+LOG_EVENTS)*LOG_BYTES==1019);
 send('A');resetRAM();recorderInit();state=FINE;
 for(int i=0;i<10;++i){recordEvent(EV_SPEED_ON);drain();recordEvent(EV_SPEED_OFF);drain();}
 assert(speedLogEvents==2 && saved==2 && dropped==0);
 // Bounded jitter stays lit, but actual drift, lost IR and hard speed do not.
 state=FIRE;fresh=true;ir.active=true;calibrated=arcValid=pointingRateReady=true;
 yaw=-.3f;yawRateFiltered=6;pointingRate=1.2f;wheelRPM=70;
 assert(maintainLock(1000) && maintainLock(1500));
 yaw=-1.1f;assert(maintainLock(1600));assert(!maintainLock(1640));
 rateViolationAt=0;yaw=-.3f;pointingRate=2;assert(maintainLock(1700));assert(!maintainLock(1740));
 yawRateFiltered=8;pointingRate=0;assert(!maintainLock(1800));
 yawRateFiltered=0;yaw=-1.31f;assert(!maintainLock(1900));
 yaw=0;wheelRPM=186;assert(!maintainLock(2000));wheelRPM=70;
 fresh=false;ir.active=false;assert(!maintainLock(2100));fresh=true;ir.active=true;
 // Production FIRE branch completes only a full two seconds, then departs.
 laserHigh=true;state=FIRE;fireStartedAt=3000;departed=false;
 checkFire(4999);assert(state==FIRE && laserHigh && !departed);
 checkFire(5000);assert(state==UNWIND && !laserHigh && departed);drain();
 laserHigh=true;state=FIRE;fireStartedAt=6000;departed=false;fresh=false;ir.active=false;
 checkFire(6100);assert(state==FINE && !laserHigh && !departed);

 std::cout<<"PASS: signed codec, all PWM values, CRC, power-cut stages, chronological dump, ring wrap, event reserve, queue overflow, persistence and Nano EEPROM bounds, centred jitter and two-second departure\n";
}
'''
with tempfile.TemporaryDirectory(prefix='hm23-check-') as d:
    cpp=Path(d)/'test.cpp';exe=Path(d)/'test'
    cpp.write_text(pre+'\n'+declarations+'\n'+control_constants+'\n'+funcs+'\n'+fire_function+'\n'+checks)
    subprocess.run(['clang++','-std=c++11',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('PASS: V35 fine/hold/return/recovery functions restored; two-second departure path restored')

# Verify the companion decoder uses the explicit new schema and signed units.
import csv, io
with tempfile.TemporaryDirectory(prefix='hm23-decode-') as d:
    fixture=Path(d)/'dump.txt'
    fixture.write_text('EVENT_LOG_BEGIN,HM2.3\ntime_ms,event,state,error_cdeg,fast_cdps,pointing_cdps,wheel_rpm_d10,pwm,bias_cPWM,flags,stop\n1000,22,10,125,-250,175,-125,-255,-325,255,0\nEVENT_LOG_END,samples=1,events=0,bad_crc=0,dropped=0\n')
    result=subprocess.run([sys.executable,str(Path(__file__).with_name('decode_log.py')),str(fixture)],capture_output=True,text=True,check=True)
    row=list(csv.DictReader(io.StringIO(result.stdout)))[0]
    assert row['state']=='FINE' and float(row['angle_error_deg'])==1.25
    assert float(row['fast_rate_deg_s'])==-2.5 and float(row['pointing_rate_deg_s'])==1.75
    assert float(row['wheel_rpm'])==-12.5 and int(row['motor_pwm'])==-255
    assert float(row['hold_bias_pwm'])==-3.25 and row['laser_high']=='True'
print('PASS: direct-unit decoder and new log marker')
