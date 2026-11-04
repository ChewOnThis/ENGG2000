"""Convert a saved HM 2.3/2.4 Serial Monitor dump into labelled physical units.
Usage: python3 decode_log.py dump.txt > decoded.csv
Only accepts the new explicit HM2.3 log marker, not legacy packed V35 data.
"""
import csv
import sys
EVENTS={2:'scan_accepted',3:'pulse_start',4:'pulse_end',5:'laser_on',6:'laser_off',9:'fault',10:'log_end',14:'hold_complete',15:'recovery',16:'search_wrong',18:'scan_rejected',19:'speed_guard_on',20:'speed_guard_off',22:'sample'}
STATES={0:'BOOT',1:'CAL',2:'CLEAR',3:'SEEK',4:'FAST_BRAKE',5:'BACKOUT',6:'SETTLE',7:'SCAN',8:'ARC_BRAKE',9:'AIM',10:'FINE',11:'FIRE',12:'UNWIND',13:'STOP'}
writer=csv.writer(sys.stdout)
writer.writerow(['seconds','event','state','angle_error_deg','fast_rate_deg_s','pointing_rate_deg_s','wheel_rpm','motor_pwm','hold_bias_pwm','ir_recent','reference_valid','laser_high','angle_pass','entry_rate_pass','wheel_pass','pulse_active','calibrated','stop_code'])
inside=False
for line in open(sys.argv[1]):
    if 'EVENT_LOG_BEGIN,HM2.3' in line or 'EVENT_LOG_BEGIN,HM2.4' in line:
        inside=True
        continue
    if not inside:
        continue
    if 'EVENT_LOG_END,' in line:
        print(line.strip(),file=sys.stderr)
        inside=False
        continue
    fields=line.strip().split(',')
    if len(fields)!=11 or not fields[0].isdigit():
        continue
    values=list(map(int,fields))
    t,event,state,error,fast,pointing,rpm,pwm,bias,flags,stop=values
    writer.writerow([t/1000,EVENTS.get(event,event),STATES.get(state,state),error/100,fast/100,pointing/100,rpm/10,pwm,bias/100]+[bool(flags&(1<<bit)) for bit in range(8)]+[stop])
