// PSEUDOCODE: pointing and targeting interface
//
// FUNCTION chooseTargetDirection(sensorReadings)
//     if centreDetected, return CENTRE
//     if leftDetected and not rightDetected, return LEFT
//     if rightDetected and not leftDetected, return RIGHT
//     otherwise return NONE
// END FUNCTION
//
// FUNCTION targetIsLost(sensorReadings)
//     return true when all three sensors are inactive
// END FUNCTION
