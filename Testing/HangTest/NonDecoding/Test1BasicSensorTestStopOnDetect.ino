
/*
  MQ SENTINEL - TEST 1A
  RAW IR DETECTION AND MOTOR STOP

  Board: Arduino Nano

  D2 = Left IR sensor
  D3 = Centre IR sensor
  D4 = Right IR sensor

  D6 = Motor IN1
  D7 = Motor IN2
  D8 = Laser (disabled)

  LOW  = IR detected
  HIGH = No IR detected

  On detection:
  1. Stop motor using electrical braking.
  2. Report which sensor detected IR.
  3. Keep motor stopped until Arduino reset.
*/

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
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);

  // IR sensors
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(CENTRE_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  // Motor
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Laser
  pinMode(LASER_PIN, OUTPUT);

  digitalWrite(LASER_PIN, LOW);

  // Keep motor stopped during startup.
  motorBrake();

  Serial.println();
  Serial.println("========================================");
  Serial.println(" MQ SENTINEL - RAW IR DETECTION TEST");
  Serial.println("========================================");

  Serial.println();

  Serial.println("D2 = LEFT");
  Serial.println("D3 = CENTRE");
  Serial.println("D4 = RIGHT");

  Serial.println();

  Serial.println("LOW  = DETECTED");
  Serial.println("HIGH = NOT DETECTED");

  Serial.println();

  Serial.println("Starting motor search...");

  motorLeft();
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
  // READ RAW SENSOR STATES
  // ----------------------------------------------------------

  byte leftState = digitalRead(LEFT_SENSOR);
  byte centreState = digitalRead(CENTRE_SENSOR);
  byte rightState = digitalRead(RIGHT_SENSOR);

  // ----------------------------------------------------------
  // CHECK FOR IR DETECTION
  // ----------------------------------------------------------

  if (leftState == LOW ||
      centreState == LOW ||
      rightState == LOW) {

    // Stop immediately, before printing anything.
    motorBrake();

    targetDetected = true;

    unsigned long detectionTime = millis();

    Serial.println();
    Serial.println("========================================");
    Serial.println("       IR TARGET DETECTED!");
    Serial.println("========================================");

    Serial.print("Detection time: ");
    Serial.print(detectionTime);
    Serial.println(" ms");

    Serial.println();

    // Report all sensors detecting at this instant.

    if (leftState == LOW) {
      Serial.println("LEFT SENSOR: DETECTED (LOW)");
    }

    if (centreState == LOW) {
      Serial.println("CENTRE SENSOR: DETECTED (LOW)");
    }

    if (rightState == LOW) {
      Serial.println("RIGHT SENSOR: DETECTED (LOW)");
    }

    Serial.println();

    Serial.println("MOTOR: ELECTRICALLY BRAKED");
    Serial.println("LASER: OFF");

    Serial.println();
    Serial.println("TEST COMPLETE.");
    Serial.println("Reset Arduino to restart.");

    return;
  }

  // ----------------------------------------------------------
  // SERIAL MONITOR OUTPUT
  // ----------------------------------------------------------

  unsigned long now = millis();

  if (now - lastPrint >= PRINT_INTERVAL_MS) {

    lastPrint = now;

    Serial.print("LEFT: ");
    Serial.print(leftState == LOW ? "LOW" : "HIGH");

    Serial.print(" | CENTRE: ");
    Serial.print(centreState == LOW ? "LOW" : "HIGH");

    Serial.print(" | RIGHT: ");
    Serial.print(rightState == LOW ? "LOW" : "HIGH");

    Serial.println(" | MOTOR: SEARCHING LEFT");
  }
}