#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES
#include <Arduino.h>
void UserIRReceiveTimerInterruptHandler();
#include <IRremote.hpp>
#include "IrSensors.h"
IrSensors *IrSensors::instance_=nullptr;
// Placeholder values: replace these after measuring the target's IR timing.
static const unsigned long IR_ACQUIRE_CONFIRM_MS = 300;
static const unsigned long IR_ACQUIRE_GAP_MS = 180;
static const unsigned long IR_REQUIRED_FRAMES = 2;
IrSensors::IrSensors(byte c,unsigned long h):centreReceiver_(new IRrecv(c)),centrePin_(c),holdMs_(h),centreSeen_(0),targetSeen_(0),zoneSince_(0),zone_(NONE),acquireConfirmMs_(IR_ACQUIRE_CONFIRM_MS),acquireGapMs_(IR_ACQUIRE_GAP_MS),acquireFirstSeen_(0),acquireLastSeen_(0),acquireFrames_(0),detectionSerial_(0),acquireConfirmed_(false){instance_=this;}
IrSensors::~IrSensors(){delete centreReceiver_;if(instance_==this)instance_=nullptr;}
void IrSensors::begin(){centreReceiver_->begin(centrePin_,false);}
void IrSensors::read(unsigned long now){
  unsigned long frames=0;
  if(centreReceiver_->decode()){centreSeen_=targetSeen_=now;centreReceiver_->resume();++frames;}
  if(frames){
    detectionSerial_ += frames;
    if(acquireFirstSeen_==0 || now-acquireLastSeen_>acquireGapMs_){
      acquireFirstSeen_=now;
      acquireFrames_=0;
      acquireConfirmed_=false;
    }
    acquireLastSeen_=now;
    acquireFrames_+=frames;
    if(acquireFrames_>=IR_REQUIRED_FRAMES && now-acquireFirstSeen_>=acquireConfirmMs_){
      acquireConfirmed_=true;
    }
  } else if(acquireLastSeen_!=0 && now-acquireLastSeen_>acquireGapMs_){
    acquireFirstSeen_=0;
    acquireLastSeen_=0;
    acquireFrames_=0;
    acquireConfirmed_=false;
  }
}
void IrSensors::updateZone(unsigned long now){Zone next=centreSeen_&&now-centreSeen_<=holdMs_?CENTRE_ONLY:NONE;if(next!=zone_){zone_=next;zoneSince_=now;}}
Zone IrSensors::zone()const{return zone_;} unsigned long IrSensors::zoneSince()const{return zoneSince_;} unsigned long IrSensors::targetLastSeen()const{return targetSeen_;}
bool IrSensors::acquisitionConfirmed(unsigned long now)const{return acquireConfirmed_&&acquireLastSeen_!=0&&now-acquireLastSeen_<=acquireGapMs_;}
unsigned long IrSensors::detectionSerial()const{return detectionSerial_;}
unsigned long IrSensors::acquisitionFirstSeen()const{return acquireFirstSeen_;}
unsigned long IrSensors::acquisitionLastSeen()const{return acquireLastSeen_;}
void IrSensors::timerInterrupt(){if(instance_){instance_->centreReceiver_->ReceiveInterruptHandler();}}
