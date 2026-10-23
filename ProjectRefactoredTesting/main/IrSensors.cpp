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
IrSensors::IrSensors(byte l,byte c,byte r,unsigned long h):centreReceiver_(new IRrecv(c)),rightReceiver_(new IRrecv(r)),leftPin_(l),centrePin_(c),rightPin_(r),holdMs_(h),leftSeen_(0),centreSeen_(0),rightSeen_(0),targetSeen_(0),zoneSince_(0),zone_(NONE),acquireConfirmMs_(IR_ACQUIRE_CONFIRM_MS),acquireGapMs_(IR_ACQUIRE_GAP_MS),acquireFirstSeen_(0),acquireLastSeen_(0),acquireFrames_(0),detectionSerial_(0),acquireConfirmed_(false){instance_=this;}
IrSensors::~IrSensors(){delete centreReceiver_;delete rightReceiver_;if(instance_==this)instance_=nullptr;}
void IrSensors::begin(){IrReceiver.begin(leftPin_,false);centreReceiver_->begin(centrePin_,false);rightReceiver_->begin(rightPin_,false);}
void IrSensors::read(unsigned long now){
  unsigned long frames=0;
  if(IrReceiver.decode()){leftSeen_=targetSeen_=now;IrReceiver.resume();++frames;}
  if(centreReceiver_->decode()){centreSeen_=targetSeen_=now;centreReceiver_->resume();++frames;}
  if(rightReceiver_->decode()){rightSeen_=targetSeen_=now;rightReceiver_->resume();++frames;}
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
Zone IrSensors::calculate(bool l,bool c,bool r){if(l&&c&&r)return ALL_THREE;if(l&&c)return LEFT_CENTRE;if(c&&r)return CENTRE_RIGHT;if(l&&!c&&!r)return LEFT_ONLY;if(!l&&!c&&r)return RIGHT_ONLY;if(!l&&c&&!r)return CENTRE_ONLY;if(l&&!c&&r)return LEFT_RIGHT;return NONE;}
void IrSensors::updateZone(unsigned long now){Zone next=calculate(leftSeen_&&now-leftSeen_<=holdMs_,centreSeen_&&now-centreSeen_<=holdMs_,rightSeen_&&now-rightSeen_<=holdMs_);if(next!=zone_){zone_=next;zoneSince_=now;}}
Zone IrSensors::zone()const{return zone_;} unsigned long IrSensors::zoneSince()const{return zoneSince_;} unsigned long IrSensors::targetLastSeen()const{return targetSeen_;} unsigned long IrSensors::lastLeft()const{return leftSeen_;} unsigned long IrSensors::lastCentre()const{return centreSeen_;} unsigned long IrSensors::lastRight()const{return rightSeen_;}
bool IrSensors::acquisitionConfirmed(unsigned long now)const{return acquireConfirmed_&&acquireLastSeen_!=0&&now-acquireLastSeen_<=acquireGapMs_;}
unsigned long IrSensors::detectionSerial()const{return detectionSerial_;}
unsigned long IrSensors::acquisitionFirstSeen()const{return acquireFirstSeen_;}
unsigned long IrSensors::acquisitionLastSeen()const{return acquireLastSeen_;}
void IrSensors::timerInterrupt(){if(instance_){instance_->centreReceiver_->ReceiveInterruptHandler();instance_->rightReceiver_->ReceiveInterruptHandler();}}
