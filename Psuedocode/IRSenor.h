// PSEUDOCODE: IR sensor interface
//
// FUNCTION initialiseIRSensors()
//     configure left, centre, and right sensor pins as INPUT
// END FUNCTION
//
// FUNCTION readIRSensors(currentTime)
//     for each IR sensor
//         if sensor reads ACTIVE level
//             store currentTime as its last detection time
//         end if
//     end for
//
//     return each sensor as detected when:
//         it has previously detected a signal, and
//         currentTime - lastDetectionTime <= IR_HOLD_MS
// END FUNCTION
