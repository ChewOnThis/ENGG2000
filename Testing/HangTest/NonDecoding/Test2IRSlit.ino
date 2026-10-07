
/*
  MQ SENTINEL - TEST 2B
  FINE SENSOR ANGULAR ACCURACY
  IRREMOTE DECODING VERSION

  Board: Arduino Nano

  CENTRE IR: D3
  MOTOR IN1: D6
  MOTOR IN2: D7
  LASER: D8 (OFF)
  STATUS LED: D13

  LED ON  = Decoded IR detected
  LED OFF = Decoded IR no longer detected

  Motor stops after exiting the detection window.

  No USB connection or Serial Monitor required.
*/

// ============================================================
// LIBRARY
// ============================================================

#include <IRremote.hpp>

// ============================================================
// PIN DEFINITIONS
// ============================================================

const byte CENTRE_SENSOR = 3;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte STATUS_LED = LED_BUILTIN;

// ============================================================
// SETTINGS
// ============================================================

const bool REVERSE_MOTOR = false;

// Camera preparation time.
const unsigned long START_DELAY_MS = 5000;

// Hold detection between received IR frames.
const unsigned long IR_HOLD_MS = 250;

// ============================================================
// TEST STATES
// ============================================================

enum TestState {
  STARTUP,
  SEARCHING,
  DETECTING,
  COMPLETE
};

TestState state = STARTUP;

// Time of most recent decoded IR frame.
unsigned long lastDetectionTime = 0;

// ============================================================
// MOTOR CONTROL
// ============================================================

void motorLeft() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

  } else {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

  }
}

void motorBrake() {

  // Preserve original braking command.
  // Verify HIGH/HIGH is braking on your driver.

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);

  pinMode(STATUS_LED, OUTPUT);

  // Laser always disabled.
  digitalWrite(LASER_PIN, LOW);

  // Detection indicator initially OFF.
  digitalWrite(STATUS_LED, LOW);

  // Keep motor stopped during startup.
  motorBrake();

  // ----------------------------------------------------------
  // INITIALISE CENTRE IR RECEIVER
  // ----------------------------------------------------------

  IrReceiver.begin(
    CENTRE_SENSOR,
    false
  );

  // Allow time for camera preparation.
  delay(START_DELAY_MS);

  // Discard any completed frame received during startup.
  // Keep the motor stationary if the beacon is already
  // being decoded before the test starts.

  if (IrReceiver.decode()) {

    IrReceiver.resume();

    state = COMPLETE;
    return;
  }

  // Begin searching.
  state = SEARCHING;

  motorLeft();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Test finished. Keep motor stationary.
  if (state == COMPLETE) {
    return;
  }

  unsigned long now = millis();

  // ==========================================================
  // CHECK FOR DECODED IR FRAME
  // ==========================================================

  if (IrReceiver.decode()) {

    // Any successful library decode is treated
    // as an IR detection for this test.

    lastDetectionTime = now;

    // --------------------------------------------------------
    // FIRST DECODED FRAME
    // --------------------------------------------------------

    if (state == SEARCHING) {

      // Visual marker for entry angle.
      digitalWrite(STATUS_LED, HIGH);

      // Continue rotating through the detection window.
      state = DETECTING;
    }

    // Prepare receiver for the next frame.
    IrReceiver.resume();
  }

  // ==========================================================
  // CHECK FOR DETECTION WINDOW EXIT
  // ==========================================================

  if (state == DETECTING) {

    // No decoded frames received within hold interval.

    if (now - lastDetectionTime >= IR_HOLD_MS) {

      // Brake immediately.
      motorBrake();

      // Visual marker for exit angle.
      digitalWrite(STATUS_LED, LOW);

      // Finish test.
      state = COMPLETE;
    }
  }
}