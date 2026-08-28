// PSEUDOCODE: motor driver interface
//
// FUNCTION initialiseMotor()
//     configure MOTOR_IN1_PIN and MOTOR_IN2_PIN as OUTPUT
//     stopMotor()
// END FUNCTION
//
// FUNCTION turnLeft()
//     command the motor driver for left rotation
//     reverse the two outputs when REVERSE_MOTOR is true
// END FUNCTION
//
// FUNCTION turnRight()
//     command the motor driver for right rotation
//     reverse the two outputs when REVERSE_MOTOR is true
// END FUNCTION
//
// FUNCTION stopMotor()
//     set MOTOR_IN1_PIN LOW
//     set MOTOR_IN2_PIN LOW
// END FUNCTION
