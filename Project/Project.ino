// MQ Sentinel - IR Tracking Controller
// Arduino Nano
//
// Wiring:
//   Left IR sensor   -> D2
//   Centre IR sensor -> D3
//   Right IR sensor  -> D4
//   Motor IN1        -> D6
//   Motor IN2        -> D7
//   Laser diode      -> D8
//   Encoder A        -> D9
//   Encoder B        -> D10
//
// Note:
// D7 is not PWM-capable on the Arduino Nano, so the motor is controlled
// using full-speed digital IN1/IN2 direction commands.

const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte ENCODER_A = 9;
const byte ENCODER_B = 10;

// FIT0186 approximate output-shaft encoder count
const long COUNTS_PER_REV = 700;

// 38 kHz IR receiver modules are normally active LOW.
const byte IR_ACTIVE = LOW;

// Stability / filtering values to reduce noisy oscillation and random-looking stops.
const unsigned long SENSOR_HOLD_MS = 80;
const unsigned long CENTRE_STABLE_MS = 150;
const unsigned long LASER_TIME_MS = 2000;
const unsigned long SENSOR_IDLE_RESET_MS = 1500;

// Encoder state
long encoderCount = 0;
byte previousEncoderA = LOW;

unsigned long lastLeft = 0;
unsigned long lastCentre = 0;
unsigned long lastRight = 0;
unsigned long centreStart = 0;
unsigned long laserStart = 0;
unsigned long lastSensorActivity = 0;

bool firing = false;
bool targetComplete = false;

// ------------------------------------------------------------
// MOTOR CONTROL
// ------------------------------------------------------------

void motorLeft() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
}

void motorRight() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

void motorStop() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH); // electircal braking
}

void resetTargetState() {
  centreStart = 0;
  firing = false;
  targetComplete = false;
  digitalWrite(LASER_PIN, LOW);
  motorStop();
}

// ------------------------------------------------------------
// ENCODER
// ------------------------------------------------------------

void updateEncoder() {
  byte currentA = digitalRead(ENCODER_A);

  // Count on rising edge of encoder channel A
  if (currentA == HIGH && previousEncoderA == LOW) {
    if (digitalRead(ENCODER_B) != currentA) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  previousEncoderA = currentA;
}

float getAngleDegrees() {
  return (encoderCount * 360.0) / COUNTS_PER_REV;
}

// ------------------------------------------------------------
// SENSOR FILTERING
// ------------------------------------------------------------

void updateSensorState(unsigned long now) {
  bool leftRaw = (digitalRead(LEFT_SENSOR) == IR_ACTIVE);
  bool centreRaw = (digitalRead(CENTRE_SENSOR) == IR_ACTIVE);
  bool rightRaw = (digitalRead(RIGHT_SENSOR) == IR_ACTIVE);

  if (leftRaw) lastLeft = now;
  if (centreRaw) lastCentre = now;
  if (rightRaw) lastRight = now;

  bool leftFiltered = (lastLeft != 0) && (now - lastLeft <= SENSOR_HOLD_MS);
  bool centreFiltered = (lastCentre != 0) && (now - lastCentre <= SENSOR_HOLD_MS);
  bool rightFiltered = (lastRight != 0) && (now - lastRight <= SENSOR_HOLD_MS);

  if (leftFiltered || centreFiltered || rightFiltered) {
    lastSensorActivity = now;
  }

  bool leftDetected = leftFiltered;
  bool centreDetected = centreFiltered;
  bool rightDetected = rightFiltered;

  if (!leftDetected && !centreDetected && !rightDetected) {
    targetComplete = false;
  }

  if (centreDetected) {
    motorStop();

    if (centreStart == 0) {
      centreStart = now;
    }

    if (!firing && !targetComplete && (now - centreStart >= CENTRE_STABLE_MS)) {
      firing = true;
      laserStart = now;
      digitalWrite(LASER_PIN, HIGH);
      Serial.println(">>> TARGET ALIGNED - LASER ON");
    }

    if (firing && (now - laserStart >= LASER_TIME_MS)) {
      digitalWrite(LASER_PIN, LOW);
      firing = false;
      targetComplete = true;
      Serial.println(">>> 2 SECOND TARGET HIT COMPLETE");
    }
  }

  else {
    centreStart = 0;

    if (firing) {
      firing = false;
      digitalWrite(LASER_PIN, LOW);
      Serial.println(">>> ALIGNMENT LOST - LASER OFF");
    }

    if (leftDetected && !rightDetected) {
      targetComplete = false;
      motorLeft();
    }
    else if (rightDetected && !leftDetected) {
      targetComplete = false;
      motorRight();
    }
    else if (!leftDetected && !rightDetected) {
      motorLeft();
    }
    else {
      motorStop();
    }
  }
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup() {
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(CENTRE_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  motorStop();
  digitalWrite(LASER_PIN, LOW);

  previousEncoderA = digitalRead(ENCODER_A);

  Serial.begin(9600);

  Serial.println("MQ Sentinel IR tracker started");
}

// ------------------------------------------------------------
// MAIN LOOP
// ------------------------------------------------------------

void loop() {
  updateEncoder();

  unsigned long now = millis();
  updateSensorState(now);

  if ((digitalRead(LEFT_SENSOR) == IR_ACTIVE) ||
      (digitalRead(CENTRE_SENSOR) == IR_ACTIVE) ||
      (digitalRead(RIGHT_SENSOR) == IR_ACTIVE)) {
    lastSensorActivity = now;
  }

  else if (lastSensorActivity != 0 && (now - lastSensorActivity >= SENSOR_IDLE_RESET_MS)) {
    resetTargetState();
    lastSensorActivity = 0;
    Serial.println(">>> SENSOR IDLE RESET - READY FOR NEXT TARGET");
  }

  // // Serial status output
  // static unsigned long lastPrint = 0;

  // if (millis() - lastPrint >= 200) {
  //   lastPrint = millis();

  //   bool leftDetected = (lastLeft != 0) && (millis() - lastLeft <= SENSOR_HOLD_MS);
  //   bool centreDetected = (lastCentre != 0) && (millis() - lastCentre <= SENSOR_HOLD_MS);
  //   bool rightDetected = (lastRight != 0) && (millis() - lastRight <= SENSOR_HOLD_MS);

  //   Serial.print("L:");
  //   Serial.print(leftDetected);

  //   Serial.print(" C:");
  //   Serial.print(centreDetected);

  //   Serial.print(" R:");
  //   Serial.print(rightDetected);

  //   Serial.print(" | Encoder:");
  //   Serial.print(encoderCount);

  //   Serial.print(" | Angle:");
  //   Serial.print(getAngleDegrees(), 1);

  //   Serial.print(" deg | Laser:");
  //   Serial.println(firing ? "ON" : "OFF");
  // }
}

