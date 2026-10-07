// ============================================================
// MQ SENTINEL - PRECISION REACTION-WHEEL IR TRACKER
// Arduino Nano
//
// D2  = Left IR sensor
// D3  = Centre IR sensor
// D4  = Right IR sensor
// D6  = Motor IN1 (PWM)
// D7  = Motor IN2
// D8  = Laser
// D9  = Encoder A
// D10 = Encoder B
//
// Strategy:
// FAST SEARCH -> SIDE ACQUIRE -> CENTRE COUNTER-TORQUE
// -> FINE ALIGN -> ACTIVE HOLD
//
// IMPORTANT:
// The motor brake brakes the reaction wheel, NOT the chassis.
// This controller therefore uses active counter-torque when the
// target is reached and small closed-loop torque pulses while holding.
// ============================================================

#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES
void UserIRReceiveTimerInterruptHandler();
#include <IRremote.hpp>

// ---------------- PINS ----------------
const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;
const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;
const byte LASER_PIN = 8;
const byte ENCODER_A = 9;
const byte ENCODER_B = 10;

// ---------------- IR RECEIVERS ----------------
IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

// ---------------- SETTINGS ----------------
const bool REVERSE_MOTOR = false;

// Short hold prevents stale centre detections at speed.
const unsigned long IR_HOLD_MS = 60;

// Require stable centre before firing laser.
const unsigned long LOCK_CONFIRM_MS = 220;

// If target disappears for this long while holding, reacquire.
const unsigned long TARGET_LOST_MS = 120;

// ---------------- ENCODER ----------------
const long COUNTS_PER_REV = 700;
long encoderCount = 0;
byte previousEncoderA = LOW;

long previousSpeedCount = 0;
unsigned long previousSpeedTime = 0;
float wheelRPM = 0.0;

// ---------------- IR TIMES ----------------
unsigned long lastLeftIR = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR = 0;

// Raw "new frame" event times used to estimate crossing speed.
unsigned long firstSideTime = 0;
int approachSide = 0;       // -1 left, +1 right
int lastKnownSide = 0;

// ---------------- STATE MACHINE ----------------
enum ControlState {
  SEARCHING,
  APPROACHING,
  COUNTER_TORQUE,
  FINE_ALIGN,
  TARGET_LOCKED
};

ControlState state = SEARCHING;
unsigned long stateStartTime = 0;
unsigned long centreStableSince = 0;
unsigned long targetLastSeen = 0;

// ---------------- SEARCH TUNING ----------------
// Search remains fast, but wheel RPM is capped to avoid endless acceleration.
const int SEARCH_POWER = 255;
const int SEARCH_CRUISE_POWER = 150;
const float MAX_SEARCH_RPM = 190.0;

// Once a side sensor sees the beacon, stop adding large momentum.
const int APPROACH_POWER = 90;

// ---------------- COUNTER-TORQUE TUNING ----------------
// These are starting calibration values.
// Shorter side->centre time means faster chassis motion and a stronger pulse.
const int COUNTER_VERY_FAST_POWER = 255;
const int COUNTER_FAST_POWER      = 230;
const int COUNTER_MED_POWER       = 180;
const int COUNTER_SLOW_POWER      = 120;

const unsigned long COUNTER_VERY_FAST_MS = 130;
const unsigned long COUNTER_FAST_MS      = 100;
const unsigned long COUNTER_MED_MS       = 70;
const unsigned long COUNTER_SLOW_MS      = 40;

// ---------------- FINE/HOLD TUNING ----------------
const int FINE_POWER = 70;
const unsigned long FINE_PULSE_MS = 22;
const unsigned long FINE_SETTLE_MS = 55;

const int HOLD_POWER = 55;
const unsigned long HOLD_PULSE_MS = 12;
const unsigned long HOLD_SETTLE_MS = 45;

unsigned long lastCorrectionTime = 0;

// Counter-torque command selected dynamically at centre crossing.
int counterPower = COUNTER_MED_POWER;
unsigned long counterDuration = COUNTER_MED_MS;
int counterDirection = 0; // -1 motorLeft, +1 motorRight

// ---------------- SERIAL ----------------
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 150;

// ============================================================
// MOTOR CONTROL
// D6 is PWM-capable; D7 is digital.
// ============================================================

void motorLeftPWM(int power) {
  power = constrain(power, 0, 255);

  if (!REVERSE_MOTOR) {
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_IN1, 255 - power);
  } else {
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, power);
  }
}

void motorRightPWM(int power) {
  power = constrain(power, 0, 255);

  if (!REVERSE_MOTOR) {
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, power);
  } else {
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_IN1, 255 - power);
  }
}

void motorBrake() {
  analogWrite(MOTOR_IN1, 255);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

// Removes drive torque without deliberately locking the wheel.
// Depending on DRV8874 mode/wiring, LOW/LOW is the intended coast command
// for the two-input abstraction used by this program.
void motorCoast() {
  analogWrite(MOTOR_IN1, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void driveDirection(int direction, int power) {
  if (direction < 0) motorLeftPWM(power);
  else if (direction > 0) motorRightPWM(power);
  else motorCoast();
}

// Short pulse without delay(), so IR decoding continues continuously.
void startPulse(int direction, int power) {
  driveDirection(direction, power);
}

// ============================================================
// ENCODER
// ============================================================

void updateEncoder() {
  byte currentA = digitalRead(ENCODER_A);

  if (currentA == HIGH && previousEncoderA == LOW) {
    if (digitalRead(ENCODER_B) != currentA) encoderCount++;
    else encoderCount--;
  }

  previousEncoderA = currentA;
}

void updateWheelSpeed(unsigned long now) {
  if (now - previousSpeedTime < 50) return;

  unsigned long dt = now - previousSpeedTime;
  long dc = encoderCount - previousSpeedCount;

  wheelRPM =
      ((float)dc / (float)COUNTS_PER_REV) *
      (60000.0 / (float)dt);

  previousSpeedCount = encoderCount;
  previousSpeedTime = now;
}

float getWheelAngle() {
  return ((float)encoderCount * 360.0) / COUNTS_PER_REV;
}

// ============================================================
// STATE HELPERS
// ============================================================

const char* stateName(ControlState s) {
  switch (s) {
    case SEARCHING:      return "SEARCH";
    case APPROACHING:    return "APPROACH";
    case COUNTER_TORQUE: return "COUNTER";
    case FINE_ALIGN:     return "FINE";
    case TARGET_LOCKED:  return "LOCKED";
  }
  return "?";
}

void changeState(ControlState next, unsigned long now) {
  if (state == next) return;

  state = next;
  stateStartTime = now;

  Serial.print("***** STATE -> ");
  Serial.println(stateName(state));
}

// Direction that produces corrective chassis torque is determined empirically.
// With the current naming convention, if target was first seen on LEFT, we
// drove LEFT toward it. At centre we command the OPPOSITE motor direction to
// oppose the chassis' continuing rotation.
int oppositeDirection(int direction) {
  return -direction;
}

void chooseCounterTorque(unsigned long crossingTimeMs) {
  if (crossingTimeMs <= 55) {
    counterPower = COUNTER_VERY_FAST_POWER;
    counterDuration = COUNTER_VERY_FAST_MS;
  } else if (crossingTimeMs <= 100) {
    counterPower = COUNTER_FAST_POWER;
    counterDuration = COUNTER_FAST_MS;
  } else if (crossingTimeMs <= 180) {
    counterPower = COUNTER_MED_POWER;
    counterDuration = COUNTER_MED_MS;
  } else {
    counterPower = COUNTER_SLOW_POWER;
    counterDuration = COUNTER_SLOW_MS;
  }

  counterDirection = oppositeDirection(approachSide);

  Serial.print(">>> CENTRE CROSSING: ");
  Serial.print(crossingTimeMs);
  Serial.print(" ms | COUNTER power=");
  Serial.print(counterPower);
  Serial.print(" duration=");
  Serial.print(counterDuration);
  Serial.print(" ms direction=");
  Serial.println(counterDirection < 0 ? "LEFT" : "RIGHT");
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorBrake();

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  previousEncoderA = digitalRead(ENCODER_A);
  previousSpeedTime = millis();

  IrReceiver.begin(LEFT_SENSOR, false);
  centreReceiver.begin(CENTRE_SENSOR, false);
  rightReceiver.begin(RIGHT_SENSOR, false);

  stateStartTime = millis();

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" MQ SENTINEL REACTION-WHEEL PRECISION TRACKER");
  Serial.println("==============================================");
  Serial.println("Fast acquisition + active counter-torque");
  Serial.println("+ fine alignment + active target hold");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  updateEncoder();
  updateWheelSpeed(now);

  // ---------------- READ IR ----------------
  bool newLeft = false;
  bool newCentre = false;
  bool newRight = false;

  if (IrReceiver.decode()) {
    lastLeftIR = now;
    targetLastSeen = now;
    newLeft = true;
    IrReceiver.resume();
  }

  if (centreReceiver.decode()) {
    lastCentreIR = now;
    targetLastSeen = now;
    newCentre = true;
    centreReceiver.resume();
  }

  if (rightReceiver.decode()) {
    lastRightIR = now;
    targetLastSeen = now;
    newRight = true;
    rightReceiver.resume();
  }

  bool L = lastLeftIR != 0 && (now - lastLeftIR <= IR_HOLD_MS);
  bool C = lastCentreIR != 0 && (now - lastCentreIR <= IR_HOLD_MS);
  bool R = lastRightIR != 0 && (now - lastRightIR <= IR_HOLD_MS);

  // ---------------- TARGET-SIDE MEMORY ----------------
  if (L && !R) lastKnownSide = -1;
  else if (R && !L) lastKnownSide = +1;

  // ==========================================================
  // SEARCHING
  // ==========================================================
  if (state == SEARCHING) {
    digitalWrite(LASER_PIN, LOW);
    centreStableSince = 0;

    // Centre can occasionally be acquired directly.
    if (C) {
      approachSide = (lastKnownSide != 0) ? lastKnownSide : -1;
      firstSideTime = now;

      // Unknown crossing speed: use conservative medium counter-torque.
      counterPower = COUNTER_MED_POWER;
      counterDuration = COUNTER_MED_MS;
      counterDirection = oppositeDirection(approachSide);

      changeState(COUNTER_TORQUE, now);
      driveDirection(counterDirection, counterPower);
    }
    else if (L || R) {
      approachSide = L ? -1 : +1;
      lastKnownSide = approachSide;
      firstSideTime = now;

      // Immediately stop adding search momentum.
      motorCoast();
      changeState(APPROACHING, now);
    }
    else {
      // Fast search but no unlimited flywheel acceleration.
      if (abs(wheelRPM) < MAX_SEARCH_RPM)
        motorLeftPWM(SEARCH_POWER);
      else
        motorLeftPWM(SEARCH_CRUISE_POWER);
    }
  }

  // ==========================================================
  // APPROACHING
  // ==========================================================
  else if (state == APPROACHING) {
    digitalWrite(LASER_PIN, LOW);

    if (C) {
      unsigned long crossingTime =
          (firstSideTime != 0) ? (now - firstSideTime) : 150;

      chooseCounterTorque(crossingTime);

      changeState(COUNTER_TORQUE, now);

      // ACTIVE braking of chassis rotation:
      // accelerate wheel in the direction that creates opposing chassis torque.
      driveDirection(counterDirection, counterPower);
    }
    else if (L && !R) {
      // Approach gently rather than continuing full-speed search.
      approachSide = -1;
      motorLeftPWM(APPROACH_POWER);
    }
    else if (R && !L) {
      approachSide = +1;
      motorRightPWM(APPROACH_POWER);
    }
    else {
      // Don't instantly launch into full search if the side signal flickers.
      motorCoast();

      if (now - stateStartTime > 450) {
        changeState(FINE_ALIGN, now);
        lastCorrectionTime = now;
      }
    }
  }

  // ==========================================================
  // ACTIVE COUNTER-TORQUE
  // ==========================================================
  else if (state == COUNTER_TORQUE) {
    digitalWrite(LASER_PIN, LOW);

    // Continue the calculated opposing torque pulse.
    if (now - stateStartTime < counterDuration) {
      driveDirection(counterDirection, counterPower);
    }
    else {
      // Do NOT simply lock the flywheel at high speed and assume
      // the chassis stopped. Remove drive and observe IR response.
      motorCoast();
      centreStableSince = 0;
      lastCorrectionTime = now;
      changeState(FINE_ALIGN, now);
    }
  }

  // ==========================================================
  // FINE ALIGN
  // ==========================================================
  else if (state == FINE_ALIGN) {
    digitalWrite(LASER_PIN, LOW);

    // Exact centre zone: do not unnecessarily disturb the wheel.
    if (C && !L && !R) {
      motorCoast();

      if (centreStableSince == 0)
        centreStableSince = now;

      if (now - centreStableSince >= LOCK_CONFIRM_MS) {
        changeState(TARGET_LOCKED, now);
      }
    }

    // LEFT or LEFT+CENTRE => small corrective torque toward left.
    else if (L && !R) {
      centreStableSince = 0;

      if (now - lastCorrectionTime >= FINE_SETTLE_MS) {
        startPulse(-1, FINE_POWER);
        lastCorrectionTime = now;
      }

      if (now - lastCorrectionTime >= FINE_PULSE_MS)
        motorCoast();
    }

    // RIGHT or RIGHT+CENTRE => small corrective torque toward right.
    else if (R && !L) {
      centreStableSince = 0;

      if (now - lastCorrectionTime >= FINE_SETTLE_MS) {
        startPulse(+1, FINE_POWER);
        lastCorrectionTime = now;
      }

      if (now - lastCorrectionTime >= FINE_PULSE_MS)
        motorCoast();
    }

    else {
      centreStableSince = 0;

      // No sensor currently sees it. Correct toward the last known side
      // instead of immediately returning to a violent search.
      if (lastKnownSide != 0 &&
          now - lastCorrectionTime >= FINE_SETTLE_MS) {
        startPulse(lastKnownSide, FINE_POWER);
        lastCorrectionTime = now;
      }

      if (now - lastCorrectionTime >= FINE_PULSE_MS)
        motorCoast();

      // If genuinely lost, resume search.
      if (targetLastSeen != 0 &&
          now - targetLastSeen > 1200) {
        approachSide = 0;
        firstSideTime = 0;
        lastKnownSide = 0;
        motorCoast();
        changeState(SEARCHING, now);
      }
    }
  }

  // ==========================================================
  // ACTIVE TARGET HOLD
  // ==========================================================
  else if (state == TARGET_LOCKED) {

    // Centre only = pointing correctly.
    if (C && !L && !R) {
      motorCoast();
      digitalWrite(LASER_PIN, HIGH);
    }

    // Drifting into LEFT zone: laser off and correct immediately.
    else if (L && !R) {
      digitalWrite(LASER_PIN, LOW);

      if (now - lastCorrectionTime >= HOLD_SETTLE_MS) {
        startPulse(-1, HOLD_POWER);
        lastCorrectionTime = now;
      }

      if (now - lastCorrectionTime >= HOLD_PULSE_MS)
        motorCoast();

      centreStableSince = 0;
    }

    // Drifting into RIGHT zone.
    else if (R && !L) {
      digitalWrite(LASER_PIN, LOW);

      if (now - lastCorrectionTime >= HOLD_SETTLE_MS) {
        startPulse(+1, HOLD_POWER);
        lastCorrectionTime = now;
      }

      if (now - lastCorrectionTime >= HOLD_PULSE_MS)
        motorCoast();

      centreStableSince = 0;
    }

    // Lost all target information.
    else {
      digitalWrite(LASER_PIN, LOW);
      motorCoast();

      if (targetLastSeen != 0 &&
          now - targetLastSeen > TARGET_LOST_MS) {
        centreStableSince = 0;
        changeState(FINE_ALIGN, now);
      }
    }
  }

  // ==========================================================
  // SERIAL TELEMETRY
  // ==========================================================
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    Serial.print("STATE:");
    Serial.print(stateName(state));

    Serial.print(" | IR:");
    Serial.print(L ? "L" : "-");
    Serial.print(C ? "C" : "-");
    Serial.print(R ? "R" : "-");

    Serial.print(" | RPM:");
    Serial.print(wheelRPM, 1);

    Serial.print(" | ENC:");
    Serial.print(encoderCount);

    Serial.print(" | WHEEL:");
    Serial.print(getWheelAngle(), 1);
    Serial.print("deg");

    Serial.print(" | APPROACH:");
    if (approachSide < 0) Serial.print("LEFT");
    else if (approachSide > 0) Serial.print("RIGHT");
    else Serial.print("NONE");

    Serial.print(" | LASER:");
    Serial.println(digitalRead(LASER_PIN) ? "ON" : "OFF");
  }
}

// ============================================================
// IRREMOTE MULTIPLE RECEIVER INTERRUPT HANDLER
// ============================================================

void UserIRReceiveTimerInterruptHandler() {
  centreReceiver.ReceiveInterruptHandler();
  rightReceiver.ReceiveInterruptHandler();
}
