// PSEUDOCODE: IR sensor implementation
//
// keep lastLeftDetection, lastCentreDetection, and lastRightDetection
//
// FUNCTION updateSensorMemory(currentTime)
//     read each receiver with digitalRead
//     refresh the matching detection timestamp when the reading is active
// END FUNCTION
//
// FUNCTION getSensorReadings(currentTime)
//     call updateSensorMemory(currentTime)
//     apply the short pulse hold interval
//     return SensorReadings
// END FUNCTION
