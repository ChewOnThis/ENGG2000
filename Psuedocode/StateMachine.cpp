// PSEUDOCODE: supervisory state-machine implementation
//
// IF centre is detected
//     stop motor
//     start centre timer if this is a new alignment
//     after CENTRE_STABLE_MS, enter FIRING unless target is complete
//     after LASER_TIME_MS, turn laser off and enter TARGET_COMPLETE
// ELSE
//     reset centre timer
//     if firing, cancel firing and turn laser off
//     if only left is detected, enter TRACKING_LEFT
//     else if only right is detected, enter TRACKING_RIGHT
//     else enter WAITING_FOR_TARGET
// END IF
//
// IF no sensor is detected
//     clear target-complete latch
//     enter WAITING_FOR_TARGET
// END IF
