// PSEUDOCODE: top-level application
//
// setup()
//     initialise serial output
//     initialise IR sensors
//     initialise motor driver
//     initialise laser
//     initialise encoder when encoder hardware is available
// END setup
//
// loop()
//     currentTime = millis()
//     readings = readIRSensors(currentTime)
//     direction = chooseTargetDirection(readings)
//     state = updateState(state, readings, currentTime)
//     executeState(state)
//     updateEncoderMeasurements(currentTime)
//     periodically print readings and state
// END loop
