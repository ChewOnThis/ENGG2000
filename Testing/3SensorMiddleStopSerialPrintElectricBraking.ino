// ============================================================
// MQ SENTINEL - 3 SENSOR IR TRACKING CONTROLLER
// Arduino Nano
//
// D2  = Left IR sensor
// D3  = Centre IR sensor
// D4  = Right IR sensor
//
// D6  = Motor IN1
// D7  = Motor IN2
//
// D8  = Laser
//
// D9  = Encoder A
// D10 = Encoder B
// ============================================================


// ============================================================
// IRREMOTE MULTIPLE RECEIVER SETUP
// IMPORTANT: declaration MUST be before #include <IRremote.hpp>
// ============================================================

#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES

void UserIRReceiveTimerInterruptHandler();

#include <IRremote.hpp>


// ============================================================
// PINS
// ============================================================

const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte ENCODER_A = 9;
const byte ENCODER_B = 10;


// ============================================================
// IR RECEIVERS
//
// The library's built-in IrReceiver is used for LEFT.
//
// CENTRE and RIGHT are additional receiver instances.
// ============================================================

IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);


// ============================================================
// SETTINGS
// ============================================================

// A successful decode is considered active for this long.
//
// This is NOT a detection threshold.
// One successful decode immediately counts as detection.
//
// The hold simply prevents the motor restarting in the gap
// between consecutive IR frames.
const unsigned long IR_HOLD_MS = 250;


// If no target is detected, search in this direction.
//
// false = current search direction
// true  = reversed search direction
const bool REVERSE_MOTOR = false;


// ============================================================
// IR DETECTION TIMES
// ============================================================

unsigned long lastLeftIR = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR = 0;


// ============================================================
// ENCODER
// ============================================================

const long COUNTS_PER_REV = 700;

long encoderCount = 0;

byte previousEncoderA = LOW;


// ============================================================
// SERIAL
// ============================================================

unsigned long lastPrint = 0;

const unsigned long PRINT_INTERVAL_MS = 300;


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


void motorRight() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

  } else {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  }
}


void motorBrake() {

  // Electrical braking
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}


// ============================================================
// ENCODER
// ============================================================

void updateEncoder() {

  byte currentA = digitalRead(ENCODER_A);


  // Rising edge of encoder channel A
  if (currentA == HIGH &&
      previousEncoderA == LOW) {

    if (digitalRead(ENCODER_B) != currentA) {

      encoderCount++;

    } else {

      encoderCount--;
    }
  }


  previousEncoderA = currentA;
}


float getWheelAngle() {

  return
      (encoderCount * 360.0) /
      COUNTS_PER_REV;
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);


  // ----------------------------------------------------------
  // Motor
  // ----------------------------------------------------------

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);


  // ----------------------------------------------------------
  // Laser
  // ----------------------------------------------------------

  pinMode(LASER_PIN, OUTPUT);

  digitalWrite(LASER_PIN, LOW);


  // ----------------------------------------------------------
  // Encoder
  // ----------------------------------------------------------

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  previousEncoderA =
      digitalRead(ENCODER_A);


  // ----------------------------------------------------------
  // IR receivers
  //
  // LEFT uses the built-in IrReceiver object.
  // ----------------------------------------------------------

  IrReceiver.begin(
      LEFT_SENSOR,
      false
  );


  centreReceiver.begin(
      CENTRE_SENSOR,
      false
  );


  rightReceiver.begin(
      RIGHT_SENSOR,
      false
  );


  // ----------------------------------------------------------
  // Initial search
  // ----------------------------------------------------------

  motorLeft();


  Serial.println();
  Serial.println("========================================");
  Serial.println("      MQ SENTINEL IR TRACKER READY");
  Serial.println("========================================");

  Serial.println();

  Serial.println("D2 = LEFT");
  Serial.println("D3 = CENTRE");
  Serial.println("D4 = RIGHT");

  Serial.println();

  Serial.println("Searching for target...");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  unsigned long now = millis();


  // ----------------------------------------------------------
  // ENCODER
  // ----------------------------------------------------------

  updateEncoder();


  // ==========================================================
  // CHECK LEFT IR RECEIVER - D2
  // ==========================================================

  if (IrReceiver.decode()) {

    lastLeftIR = now;

    Serial.println(">>> LEFT IR FRAME DETECTED");

    IrReceiver.resume();
  }


  // ==========================================================
  // CHECK CENTRE IR RECEIVER - D3
  // ==========================================================

  if (centreReceiver.decode()) {

    lastCentreIR = now;

    Serial.println(">>> CENTRE IR FRAME DETECTED");

    centreReceiver.resume();
  }


  // ==========================================================
  // CHECK RIGHT IR RECEIVER - D4
  // ==========================================================

  if (rightReceiver.decode()) {

    lastRightIR = now;

    Serial.println(">>> RIGHT IR FRAME DETECTED");

    rightReceiver.resume();
  }


  // ==========================================================
  // DETECTION STATES
  // ==========================================================

  bool leftDetected =
      lastLeftIR != 0 &&
      now - lastLeftIR <= IR_HOLD_MS;


  bool centreDetected =
      lastCentreIR != 0 &&
      now - lastCentreIR <= IR_HOLD_MS;


  bool rightDetected =
      lastRightIR != 0 &&
      now - lastRightIR <= IR_HOLD_MS;


  // ==========================================================
  // CONTROL LOGIC
  //
  // CENTRE ALWAYS HAS HIGHEST PRIORITY.
  // ==========================================================


  // ----------------------------------------------------------
  // CENTRE
  // ----------------------------------------------------------

  if (centreDetected) {

    // Immediately electrically brake reaction wheel
    motorBrake();

    // Target considered aligned
    digitalWrite(LASER_PIN, HIGH);
  }


  // ----------------------------------------------------------
  // LEFT
  // ----------------------------------------------------------

  else if (leftDetected &&
           !rightDetected) {

    digitalWrite(LASER_PIN, LOW);

    motorLeft();
  }


  // ----------------------------------------------------------
  // RIGHT
  // ----------------------------------------------------------

  else if (rightDetected &&
           !leftDetected) {

    digitalWrite(LASER_PIN, LOW);

    motorRight();
  }


  // ----------------------------------------------------------
  // BOTH SIDE SENSORS
  //
  // Ambiguous signal -> brake instead of randomly choosing.
  // ----------------------------------------------------------

  else if (leftDetected &&
           rightDetected) {

    digitalWrite(LASER_PIN, LOW);

    motorBrake();
  }


  // ----------------------------------------------------------
  // NOTHING DETECTED
  //
  // Continue rotating to search for beacon.
  // ----------------------------------------------------------

  else {

    digitalWrite(LASER_PIN, LOW);

    motorLeft();
  }


  // ==========================================================
  // SERIAL STATUS
  // ==========================================================

  if (now - lastPrint >= PRINT_INTERVAL_MS) {

    lastPrint = now;


    Serial.print("LEFT: ");
    Serial.print(leftDetected ? "DETECTED" : "---");


    Serial.print(" | CENTRE: ");
    Serial.print(centreDetected ? "DETECTED" : "---");


    Serial.print(" | RIGHT: ");
    Serial.print(rightDetected ? "DETECTED" : "---");


    Serial.print(" | MOTOR: ");


    if (centreDetected) {

      Serial.print("BRAKED");

    }

    else if (leftDetected &&
             !rightDetected) {

      Serial.print("LEFT");

    }

    else if (rightDetected &&
             !leftDetected) {

      Serial.print("RIGHT");

    }

    else if (leftDetected &&
             rightDetected) {

      Serial.print("BRAKED - AMBIGUOUS");

    }

    else {

      Serial.print("SEARCHING LEFT");
    }


    Serial.print(" | LASER: ");

    Serial.print(
        centreDetected ? "ON" : "OFF"
    );


    Serial.print(" | Encoder: ");

    Serial.print(encoderCount);


    Serial.print(" | Wheel: ");

    Serial.print(
        getWheelAngle(),
        1
    );

    Serial.println(" deg");
  }
}


// ============================================================
// REQUIRED BY IRREMOTE FOR MULTIPLE RECEIVERS
//
// The IRremote timer automatically services IrReceiver (LEFT).
//
// This handler services the additional CENTRE and RIGHT
// instances.
//
// This function is REQUIRED when:
// SUPPORT_MULTIPLE_RECEIVER_INSTANCES is defined.
// ============================================================

void UserIRReceiveTimerInterruptHandler() {

  centreReceiver.ReceiveInterruptHandler();

  rightReceiver.ReceiveInterruptHandler();
}
