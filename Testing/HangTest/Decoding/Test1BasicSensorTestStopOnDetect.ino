
/*
  MQ SENTINEL - TEST 1B
  IRREMOTE DECODED DETECTION AND MOTOR STOP

  Board: Arduino Nano

  D2 = Left IR sensor
  D3 = Centre IR sensor
  D4 = Right IR sensor

  D6 = Motor IN1
  D7 = Motor IN2
  D8 = Laser (disabled)

  Any successfully decoded IR frame
  triggers immediate motor braking.
*/

// ============================================================
// IRREMOTE MULTIPLE RECEIVER SETUP
// ============================================================

// Must be declared before including IRremote.hpp.

#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES

void UserIRReceiveTimerInterruptHandler();

#include <IRremote.hpp>

// ============================================================
// PIN DEFINITIONS
// ============================================================

const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

// ============================================================
// IR RECEIVERS
// ============================================================

// LEFT uses the library's built-in IrReceiver.

IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

// ============================================================
// SETTINGS
// ============================================================

const bool REVERSE_MOTOR = false;

const unsigned long PRINT_INTERVAL_MS = 300;

// ============================================================
// VARIABLES
// ============================================================

bool targetDetected = false;

unsigned long lastPrint = 0;

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

  // Original MQ Sentinel electrical braking method.
  // Confirm HIGH/HIGH is braking for your motor driver.

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

// ============================================================
// DETECTION EVENT
// ============================================================

void reportDetection(const char* sensorName) {

  // Stop motor immediately.
  motorBrake();

  targetDetected = true;

  unsigned long detectionTime = millis();

  Serial.println();
  Serial.println("========================================");
  Serial.println("       IR FRAME DETECTED!");
  Serial.println("========================================");

  Serial.print("Sensor: ");
  Serial.println(sensorName);

  Serial.print("Detection time: ");
  Serial.print(detectionTime);
  Serial.println(" ms");

  Serial.println();

  Serial.println("MOTOR: ELECTRICALLY BRAKED");
  Serial.println("LASER: OFF");

  Serial.println();

  Serial.println("TEST COMPLETE.");
  Serial.println("Reset Arduino to restart.");
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);

  // Motor
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Laser
  pinMode(LASER_PIN, OUTPUT);

  digitalWrite(LASER_PIN, LOW);

  // Keep motor stopped during startup.
  motorBrake();

  // ----------------------------------------------------------
  // INITIALISE IR RECEIVERS
  // ----------------------------------------------------------

  // LEFT
  IrReceiver.begin(
    LEFT_SENSOR,
    false
  );

  // CENTRE
  centreReceiver.begin(
    CENTRE_SENSOR,
    false
  );

  // RIGHT
  rightReceiver.begin(
    RIGHT_SENSOR,
    false
  );

  // ----------------------------------------------------------
  // SERIAL OUTPUT
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("========================================");
  Serial.println(" MQ SENTINEL - DECODED IR TEST");
  Serial.println("========================================");

  Serial.println();

  Serial.println("D2 = LEFT");
  Serial.println("D3 = CENTRE");
  Serial.println("D4 = RIGHT");

  Serial.println();

  Serial.println("Waiting for decoded IR frame...");

  Serial.println();

  // Start searching.
  motorLeft();

  Serial.println("MOTOR: SEARCHING LEFT");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Once detected, motor remains stopped.
  if (targetDetected) {
    return;
  }

  // ----------------------------------------------------------
  // LEFT IR RECEIVER - D2
  // ----------------------------------------------------------

  if (IrReceiver.decode()) {

    reportDetection("LEFT");

    IrReceiver.resume();

    return;
  }

  // ----------------------------------------------------------
  // CENTRE IR RECEIVER - D3
  // ----------------------------------------------------------

  if (centreReceiver.decode()) {

    reportDetection("CENTRE");

    centreReceiver.resume();

    return;
  }

  // ----------------------------------------------------------
  // RIGHT IR RECEIVER - D4
  // ----------------------------------------------------------

  if (rightReceiver.decode()) {

    reportDetection("RIGHT");

    rightReceiver.resume();

    return;
  }

  // ----------------------------------------------------------
  // SERIAL STATUS
  // ----------------------------------------------------------

  unsigned long now = millis();

  if (now - lastPrint >= PRINT_INTERVAL_MS) {

    lastPrint = now;

    Serial.println(
      "LEFT: WAITING | CENTRE: WAITING | "
      "RIGHT: WAITING | MOTOR: SEARCHING LEFT"
    );
  }
}

// ============================================================
// MULTIPLE RECEIVER INTERRUPT HANDLER
// ============================================================

// Required by the original multi-receiver configuration.

void UserIRReceiveTimerInterruptHandler() {

  centreReceiver.ReceiveInterruptHandler();

  rightReceiver.ReceiveInterruptHandler();
}