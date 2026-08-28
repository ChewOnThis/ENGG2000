// PSEUDOCODE: encoder interface for future closed-loop control
//
// FUNCTION initialiseEncoder()
//     configure encoder channel pins as INPUT
//     reset pulse count and direction
// END FUNCTION
//
// FUNCTION updateEncoder()
//     detect encoder edges
//     update pulse count and measured direction
// END FUNCTION
//
// FUNCTION getMotorPosition()
//     return pulse count converted to motor angle
// END FUNCTION
//
// FUNCTION getMotorSpeed()
//     return change in pulse count over elapsed time
// END FUNCTION
