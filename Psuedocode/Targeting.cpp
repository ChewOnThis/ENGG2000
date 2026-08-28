// PSEUDOCODE: pointing and targeting implementation
//
// FUNCTION chooseTargetDirection(readings)
//     give centre detection highest priority
//     otherwise select the only active side
//     if both side sensors or no sensors are active, return NONE
// END FUNCTION
//
// FUNCTION targetIsLost(readings)
//     return not readings.leftDetected
//            and not readings.centreDetected
//            and not readings.rightDetected
// END FUNCTION
