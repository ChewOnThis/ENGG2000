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

// Change this to LOW if your IR receivers output LOW when IR is detected.
const byte IR_ACTIVE = HIGH;

// Encoder state
long encoderCount = 0;
byte previousEncoderA = LOW;

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
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
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

  bool leftDetected   = (digitalRead(LEFT_SENSOR)   == IR_ACTIVE);
  bool centreDetected = (digitalRead(CENTRE_SENSOR) == IR_ACTIVE);
  bool rightDetected  = (digitalRead(RIGHT_SENSOR)  == IR_ACTIVE);

  // Centre sensor has priority.
  // When centred, stop rotation and activate the laser.
  if (centreDetected) {
    motorStop();
    digitalWrite(LASER_PIN, HIGH);
  }

  // Target is to the left.
  else if (leftDetected && !rightDetected) {
    digitalWrite(LASER_PIN, LOW);
    motorLeft();
  }

  // Target is to the right.
  else if (rightDetected && !leftDetected) {
    digitalWrite(LASER_PIN, LOW);
    motorRight();
  }

  // Ambiguous signal or no target.
  else {
    motorStop();
    digitalWrite(LASER_PIN, LOW);
  }

  // Serial status output
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    Serial.print("L:");
    Serial.print(leftDetected);

    Serial.print(" C:");
    Serial.print(centreDetected);

    Serial.print(" R:");
    Serial.print(rightDetected);

    Serial.print(" | Encoder:");
    Serial.print(encoderCount);

    Serial.print(" | Angle:");
    Serial.print(getAngleDegrees(), 1);

    Serial.print(" deg | Laser:");
    Serial.println(centreDetected ? "ON" : "OFF");
  }
}
