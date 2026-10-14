#include "IrSensors.h"
IrSensors *IrSensors::instance_=nullptr;
IrSensors::IrSensors(byte l,byte c,byte r,unsigned long h):centreReceiver_(c),rightReceiver_(r),leftPin_(l),centrePin_(c),rightPin_(r),holdMs_(h),leftSeen_(0),centreSeen_(0),rightSeen_(0),targetSeen_(0),zoneSince_(0),zone_(NONE){instance_=this;}
void IrSensors::begin(){IrReceiver.begin(leftPin_,false);centreReceiver_.begin(centrePin_,false);rightReceiver_.begin(rightPin_,false);}
void IrSensors::read(unsigned long now){if(IrReceiver.decode()){leftSeen_=targetSeen_=now;IrReceiver.resume();}if(centreReceiver_.decode()){centreSeen_=targetSeen_=now;centreReceiver_.resume();}if(rightReceiver_.decode()){rightSeen_=targetSeen_=now;rightReceiver_.resume();}}
Zone IrSensors::calculate(bool l,bool c,bool r){if(l&&c&&r)return ALL_THREE;if(l&&c)return LEFT_CENTRE;if(c&&r)return CENTRE_RIGHT;if(l&&!c&&!r)return LEFT_ONLY;if(!l&&!c&&r)return RIGHT_ONLY;if(!l&&c&&!r)return CENTRE_ONLY;if(l&&!c&&r)return LEFT_RIGHT;return NONE;}
void IrSensors::updateZone(unsigned long now){Zone next=calculate(leftSeen_&&now-leftSeen_<=holdMs_,centreSeen_&&now-centreSeen_<=holdMs_,rightSeen_&&now-rightSeen_<=holdMs_);if(next!=zone_){zone_=next;zoneSince_=now;}}
Zone IrSensors::zone()const{return zone_;} unsigned long IrSensors::zoneSince()const{return zoneSince_;} unsigned long IrSensors::targetLastSeen()const{return targetSeen_;} unsigned long IrSensors::lastLeft()const{return leftSeen_;} unsigned long IrSensors::lastCentre()const{return centreSeen_;} unsigned long IrSensors::lastRight()const{return rightSeen_;}
void IrSensors::timerInterrupt(){if(instance_){instance_->centreReceiver_.ReceiveInterruptHandler();instance_->rightReceiver_.ReceiveInterruptHandler();}}
