// PSEUDOCODE: encoder implementation
//
// on every encoder interrupt:
//     determine direction from the second encoder channel
//     increment or decrement pulse count
//
// FUNCTION updateEncoderMeasurements(currentTime)
//     calculate pulses since the previous sample
//     calculate angular speed from pulses and elapsed time
// END FUNCTION
