// ============================================================
// MQ SENTINEL - STEP / SETTLE PRECISION TRACKER V5
// Arduino Nano
//
// Pins:
// D2 Left IR, D3 Centre IR, D4 Right IR
// D6 DRV8874 IN1 (PWM), D7 DRV8874 IN2
// D8 Laser
// D9 Encoder A, D10 Encoder B
//
// IMPORTANT: assumes DRV8874 is configured in IN/IN mode.
//
// Observed hang-test mapping:
//   motorLeftPWM()  -> flywheel CLOCKWISE -> chassis CCW torque
//   motorRightPWM() -> flywheel CCW       -> chassis CW torque
//
// Search no longer spins the flywheel continuously.
// Every move is:
//   finite encoder step -> opposite torque to stop wheel -> settle -> sense
//
// Laser fires ONLY when Left + Centre + Right are all active.
// After one uninterrupted 2.05 s hit, WAIT_TARGET_CLEAR prevents a
// continuously held beacon from causing repeated 360-degree reacquisition.
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

IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

// ---------------- SETTINGS ----------------
const bool REVERSE_MOTOR = false;

const unsigned long IR_HOLD_MS = 140;
const unsigned long ALIGN_CONFIRM_MS = 300;
const unsigned long LASER_DWELL_MS = 2050;
const unsigned long TARGET_CLEAR_MS = 350;
const unsigned long LOCAL_TARGET_LOST_MS = 900;
const unsigned long STARTUP_SETTLE_MS = 1500;

// Desired chassis directions.
// LEFT is CCW chassis torque, RIGHT is CW chassis torque.
const int BODY_LEFT  = -1;
const int BODY_RIGHT = +1;

// ---------------- ENCODER ----------------
const long COUNTS_PER_REV = 700;
long encoderCount = 0;
byte previousEncoderA = LOW;
long previousSpeedCount = 0;
unsigned long previousSpeedTime = 0;
float wheelRPM = 0.0;

// ---------------- IR TIMING ----------------
unsigned long lastLeftIR = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR = 0;
unsigned long targetLastSeen = 0;

// ---------------- SENSOR ZONES ----------------
enum SensorZone {
  ZONE_NONE,
  ZONE_LEFT,
  ZONE_LEFT_CENTRE,
  ZONE_CENTRE_ONLY,
  ZONE_ALIGNED_ALL3,
  ZONE_CENTRE_RIGHT,
  ZONE_RIGHT,
  ZONE_LEFT_RIGHT
};

SensorZone currentZone = ZONE_NONE;
SensorZone previousZone = ZONE_NONE;
unsigned long zoneSince = 0;

// ---------------- MAIN CONTROLLER ----------------
enum ControllerState {
  STARTUP_SETTLE,
  SEARCHING,
  LOCAL_ALIGN,
  ALIGN_CONFIRM,
  FIRING,
  WAIT_TARGET_CLEAR
};

ControllerState controllerState = STARTUP_SETTLE;
unsigned long stateStartTime = 0;
unsigned long alignmentSince = 0;
unsigned long laserStartedAt = 0;
unsigned long targetClearSince = 0;

// ---------------- STEP ENGINE ----------------
enum StepPhase {
  STEP_IDLE,
  STEP_DRIVE,
  STEP_STOP_WHEEL,
  STEP_SETTLE
};

enum StepPurpose {
  PURPOSE_NONE,
  PURPOSE_SEARCH,
  PURPOSE_FINE
};

StepPhase stepPhase = STEP_IDLE;
StepPurpose stepPurpose = PURPOSE_NONE;

int stepBodyDirection = 0;
long stepStartEncoder = 0;
long stepTargetCounts = 0;
int stepDrivePower = 0;
unsigned long stepSettleTime = 0;
unsigned long stepPhaseStartedAt = 0;
unsigned long wheelSlowSince = 0;
bool stepCompletedEvent = false;

// ---------------- STEP TUNING ----------------
// Encoder counts are reaction-wheel counts, not chassis degrees.
const long SEARCH_STEP_COUNTS = 80;
const int SEARCH_STEP_POWER = 165;
const unsigned long SEARCH_SETTLE_MS = 260;

const long FINE_OUTER_COUNTS = 28;
const int FINE_OUTER_POWER = 135;
const unsigned long FINE_OUTER_SETTLE_MS = 220;

const long FINE_INNER_COUNTS = 12;
const int FINE_INNER_POWER = 115;
const unsigned long FINE_INNER_SETTLE_MS = 240;

const long CENTRE_PROBE_COUNTS = 7;
const int CENTRE_PROBE_POWER = 105;
const unsigned long CENTRE_PROBE_SETTLE_MS = 260;

// Stop wheel with opposite reaction torque; do not seize it at speed.
const int WHEEL_STOP_POWER = 205;
const float WHEEL_STOP_RPM = 8.0;
const unsigned long WHEEL_STOP_STABLE_MS = 70;
const unsigned long STEP_DRIVE_TIMEOUT_MS = 600;
const unsigned long STEP_STOP_TIMEOUT_MS = 650;

// ---------------- BOUNDED SEARCH ----------------
// Initial half sweep, then alternate larger sweeps to limit string wind-up.
int searchDirection = BODY_LEFT;
const int SEARCH_INITIAL_HALF_SWEEP_STEPS = 18;
const int SEARCH_FULL_SWEEP_STEPS = 36;
int searchStepsRemaining = SEARCH_INITIAL_HALF_SWEEP_STEPS;

// ---------------- LOCAL TARGET MEMORY ----------------
int lastTargetDirection = 0;
int centreProbeDirection = BODY_LEFT;
int centreProbeAttempts = 0;

// ---------------- ADAPTIVE STRING HOLD ----------------
// Learn only after actual drift; no blind hold torque.
int learnedHoldDirection = 0;
int learnedHoldPower = 0;

const int HOLD_START_POWER = 42;
const int HOLD_MAX_POWER = 72;
const int HOLD_LEARN_INNER_STEP = 4;
const int HOLD_LEARN_OUTER_STEP = 7;

const unsigned long HOLD_PULSE_MS = 3;
const unsigned long HOLD_PERIOD_MS = 110;
bool holdPulseActive = false;
unsigned long holdPulseEnd = 0;
unsigned long lastHoldPulse = 0;
const float HOLD_WHEEL_RPM_LIMIT = 150.0;

// ---------------- SERIAL ----------------
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 180;

// ============================================================
// MOTOR CONTROL - DRV8874 IN/IN MODE
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

// Correct IN/IN coast state.
void motorCoast() {
  analogWrite(MOTOR_IN1, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

// Correct IN/IN brake state.
// Not used to stop chassis rotation at speed.
void motorBrake() {
  analogWrite(MOTOR_IN1, 255);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

void driveBodyTorque(int bodyDirection, int power) {
  if (bodyDirection == BODY_LEFT) {
    motorLeftPWM(power);
  } else if (bodyDirection == BODY_RIGHT) {
    motorRightPWM(power);
  } else {
    motorCoast();
  }
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
  if (now - previousSpeedTime < 30) return;

  unsigned long dt = now - previousSpeedTime;
  long delta = encoderCount - previousSpeedCount;

  wheelRPM =
    ((float)delta / (float)COUNTS_PER_REV) *
    (60000.0 / (float)dt);

  previousSpeedCount = encoderCount;
  previousSpeedTime = now;
}

float absFloat(float x) {
  return x < 0.0 ? -x : x;
}

// ============================================================
// IR
// ============================================================

void readIR(unsigned long now) {
  if (IrReceiver.decode()) {
    lastLeftIR = now;
    targetLastSeen = now;
    IrReceiver.resume();
  }

  if (centreReceiver.decode()) {
    lastCentreIR = now;
    targetLastSeen = now;
    centreReceiver.resume();
  }

  if (rightReceiver.decode()) {
    lastRightIR = now;
    targetLastSeen = now;
    rightReceiver.resume();
  }
}

SensorZone calculateZone(bool L, bool C, bool R) {
  if (L && C && R) return ZONE_ALIGNED_ALL3;
  if (L && C && !R) return ZONE_LEFT_CENTRE;
  if (!L && C && R) return ZONE_CENTRE_RIGHT;
  if (L && !C && !R) return ZONE_LEFT;
  if (!L && !C && R) return ZONE_RIGHT;
  if (!L && C && !R) return ZONE_CENTRE_ONLY;
  if (L && !C && R) return ZONE_LEFT_RIGHT;
  return ZONE_NONE;
}

const char* zoneName(SensorZone zone) {
  switch (zone) {
    case ZONE_NONE:          return "---";
    case ZONE_LEFT:          return "L";
    case ZONE_LEFT_CENTRE:   return "LC";
    case ZONE_CENTRE_ONLY:   return "C";
    case ZONE_ALIGNED_ALL3:  return "LCR";
    case ZONE_CENTRE_RIGHT:  return "CR";
    case ZONE_RIGHT:         return "R";
    case ZONE_LEFT_RIGHT:    return "LR";
  }
  return "?";
}

void updateSensorZone(bool L, bool C, bool R, unsigned long now) {
  SensorZone newZone = calculateZone(L, C, R);

  if (newZone != currentZone) {
    previousZone = currentZone;
    currentZone = newZone;
    zoneSince = now;
  }

  if (currentZone == ZONE_LEFT ||
      currentZone == ZONE_LEFT_CENTRE) {
    lastTargetDirection = BODY_LEFT;
    centreProbeDirection = BODY_LEFT;
  } else if (currentZone == ZONE_RIGHT ||
             currentZone == ZONE_CENTRE_RIGHT) {
    lastTargetDirection = BODY_RIGHT;
    centreProbeDirection = BODY_RIGHT;
  }
}

bool anyTargetDetected() {
  return currentZone != ZONE_NONE;
}

bool alignedAllThree() {
  return currentZone == ZONE_ALIGNED_ALL3;
}

// ============================================================
// ADAPTIVE HOLD
// ============================================================

void stopHoldPulse() {
  if (holdPulseActive) {
    holdPulseActive = false;
    motorCoast();
  }
}

void clearHoldLearning() {
  stopHoldPulse();
  learnedHoldDirection = 0;
  learnedHoldPower = 0;
}

void learnHoldFromCorrection(int correctionDirection, bool outerCorrection) {
  if (correctionDirection == 0) return;

  int step = outerCorrection
    ? HOLD_LEARN_OUTER_STEP
    : HOLD_LEARN_INNER_STEP;

  if (learnedHoldDirection != correctionDirection) {
    learnedHoldDirection = correctionDirection;
    learnedHoldPower = HOLD_START_POWER;
  } else {
    learnedHoldPower += step;
    learnedHoldPower = constrain(
      learnedHoldPower,
      HOLD_START_POWER,
      HOLD_MAX_POWER
    );
  }

  Serial.print(">>> HOLD LEARN ");
  Serial.print(learnedHoldDirection == BODY_LEFT ? "LEFT" : "RIGHT");
  Serial.print(" power=");
  Serial.println(learnedHoldPower);
}

void serviceLearnedHold(unsigned long now) {
  if (learnedHoldDirection == 0 || learnedHoldPower <= 0) return;

  // Do not let string compensation saturate the reaction wheel.
  if (absFloat(wheelRPM) >= HOLD_WHEEL_RPM_LIMIT) {
    stopHoldPulse();
    return;
  }

  if (holdPulseActive) {
    if (now >= holdPulseEnd) {
      holdPulseActive = false;
      motorCoast();
    }
    return;
  }

  if (now - lastHoldPulse >= HOLD_PERIOD_MS) {
    driveBodyTorque(learnedHoldDirection, learnedHoldPower);
    holdPulseActive = true;
    holdPulseEnd = now + HOLD_PULSE_MS;
    lastHoldPulse = now;
  }
}

// ============================================================
// STEP / SETTLE ENGINE
// ============================================================

bool stepIsBusy() {
  return stepPhase != STEP_IDLE;
}

void startBodyStep(
  int bodyDirection,
  long encoderCounts,
  int drivePower,
  unsigned long settleMs,
  StepPurpose purpose,
  unsigned long now
) {
  if (stepIsBusy()) return;

  stopHoldPulse();

  stepBodyDirection = bodyDirection;
  stepTargetCounts = encoderCounts;
  stepDrivePower = drivePower;
  stepSettleTime = settleMs;
  stepPurpose = purpose;
  stepStartEncoder = encoderCount;
  wheelSlowSince = 0;
  stepCompletedEvent = false;

  stepPhase = STEP_DRIVE;
  stepPhaseStartedAt = now;

  driveBodyTorque(stepBodyDirection, stepDrivePower);
}

void beginWheelStop(unsigned long now) {
  stepPhase = STEP_STOP_WHEEL;
  stepPhaseStartedAt = now;
  wheelSlowSince = 0;

  // Opposite reaction torque removes wheel momentum.
  driveBodyTorque(-stepBodyDirection, WHEEL_STOP_POWER);
}

void finishWheelStop(unsigned long now) {
  motorCoast();
  stepPhase = STEP_SETTLE;
  stepPhaseStartedAt = now;
}

void finishStep() {
  motorCoast();
  stepPhase = STEP_IDLE;
  stepPurpose = PURPOSE_NONE;
  stepCompletedEvent = true;
}

void serviceStepEngine(unsigned long now) {
  if (stepPhase == STEP_IDLE) return;

  if (stepPhase == STEP_DRIVE) {
    long movedCounts = labs(encoderCount - stepStartEncoder);

    // Any IR during broad search ends the drive impulse immediately.
    if (stepPurpose == PURPOSE_SEARCH && anyTargetDetected()) {
      beginWheelStop(now);
      return;
    }

    if (movedCounts >= stepTargetCounts) {
      beginWheelStop(now);
      return;
    }

    if (now - stepPhaseStartedAt >= STEP_DRIVE_TIMEOUT_MS) {
      Serial.println(">>> STEP DRIVE TIMEOUT");
      beginWheelStop(now);
      return;
    }
  }

  else if (stepPhase == STEP_STOP_WHEEL) {
    if (absFloat(wheelRPM) <= WHEEL_STOP_RPM) {
      if (wheelSlowSince == 0) wheelSlowSince = now;

      if (now - wheelSlowSince >= WHEEL_STOP_STABLE_MS) {
        finishWheelStop(now);
        return;
      }
    } else {
      wheelSlowSince = 0;
    }

    if (now - stepPhaseStartedAt >= STEP_STOP_TIMEOUT_MS) {
      Serial.println(">>> WHEEL STOP TIMEOUT");
      finishWheelStop(now);
      return;
    }
  }

  else if (stepPhase == STEP_SETTLE) {
    motorCoast();

    if (now - stepPhaseStartedAt >= stepSettleTime) {
      finishStep();
      return;
    }
  }
}

// ============================================================
// SEARCH
// ============================================================

void advanceSearchSweep() {
  searchStepsRemaining--;

  if (searchStepsRemaining <= 0) {
    searchDirection = -searchDirection;
    searchStepsRemaining = SEARCH_FULL_SWEEP_STEPS;

    Serial.print(">>> SEARCH SWEEP REVERSE -> ");
    Serial.println(searchDirection == BODY_LEFT ? "LEFT" : "RIGHT");
  }
}

void startSearchStep(unsigned long now) {
  startBodyStep(
    searchDirection,
    SEARCH_STEP_COUNTS,
    SEARCH_STEP_POWER,
    SEARCH_SETTLE_MS,
    PURPOSE_SEARCH,
    now
  );
}

// ============================================================
// LOCAL ALIGNMENT
// ============================================================

void startFineStepForCurrentZone(unsigned long now) {
  if (currentZone == ZONE_LEFT ||
      currentZone == ZONE_LEFT_CENTRE) {

    bool outer = currentZone == ZONE_LEFT;
    learnHoldFromCorrection(BODY_LEFT, outer);

    startBodyStep(
      BODY_LEFT,
      outer ? FINE_OUTER_COUNTS : FINE_INNER_COUNTS,
      outer ? FINE_OUTER_POWER : FINE_INNER_POWER,
      outer ? FINE_OUTER_SETTLE_MS : FINE_INNER_SETTLE_MS,
      PURPOSE_FINE,
      now
    );
    return;
  }

  if (currentZone == ZONE_RIGHT ||
      currentZone == ZONE_CENTRE_RIGHT) {

    bool outer = currentZone == ZONE_RIGHT;
    learnHoldFromCorrection(BODY_RIGHT, outer);

    startBodyStep(
      BODY_RIGHT,
      outer ? FINE_OUTER_COUNTS : FINE_INNER_COUNTS,
      outer ? FINE_OUTER_POWER : FINE_INNER_POWER,
      outer ? FINE_OUTER_SETTLE_MS : FINE_INNER_SETTLE_MS,
      PURPOSE_FINE,
      now
    );
    return;
  }

  if (currentZone == ZONE_CENTRE_ONLY) {
    if (centreProbeAttempts >= 2) {
      centreProbeDirection = -centreProbeDirection;
      centreProbeAttempts = 0;
    }

    centreProbeAttempts++;

    startBodyStep(
      centreProbeDirection,
      CENTRE_PROBE_COUNTS,
      CENTRE_PROBE_POWER,
      CENTRE_PROBE_SETTLE_MS,
      PURPOSE_FINE,
      now
    );
    return;
  }

  if (currentZone == ZONE_LEFT_RIGHT) {
    int dir = lastTargetDirection != 0
      ? lastTargetDirection
      : centreProbeDirection;

    startBodyStep(
      dir,
      CENTRE_PROBE_COUNTS,
      CENTRE_PROBE_POWER,
      CENTRE_PROBE_SETTLE_MS,
      PURPOSE_FINE,
      now
    );
  }
}

// ============================================================
// STATE HELPERS
// ============================================================

const char* controllerStateName() {
  switch (controllerState) {
    case STARTUP_SETTLE:    return "STARTUP";
    case SEARCHING:         return "SEARCH";
    case LOCAL_ALIGN:       return "LOCAL_ALIGN";
    case ALIGN_CONFIRM:     return "ALIGN_CONFIRM";
    case FIRING:            return "FIRING";
    case WAIT_TARGET_CLEAR: return "WAIT_CLEAR";
  }
  return "?";
}

void changeControllerState(ControllerState next, unsigned long now) {
  if (controllerState == next) return;

  controllerState = next;
  stateStartTime = now;

  Serial.print("***** STATE -> ");
  Serial.println(controllerStateName());
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Correct IN/IN brake only at startup.
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
  zoneSince = millis();

  Serial.println();
  Serial.println("================================================");
  Serial.println(" MQ SENTINEL STEP / SETTLE TRACKER V5");
  Serial.println("================================================");
  Serial.println("DRV8874 assumed IN/IN mode");
  Serial.println("Finite step -> active wheel stop -> settle -> sense");
  Serial.println("Laser requires L+C+R continuously for 2.05 s");
  Serial.println("Held beacon after hit enters WAIT_CLEAR");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  updateEncoder();
  updateWheelSpeed(now);
  readIR(now);

  bool L = lastLeftIR != 0 && now - lastLeftIR <= IR_HOLD_MS;
  bool C = lastCentreIR != 0 && now - lastCentreIR <= IR_HOLD_MS;
  bool R = lastRightIR != 0 && now - lastRightIR <= IR_HOLD_MS;

  updateSensorZone(L, C, R, now);

  // Always finish any in-progress finite movement first.
  serviceStepEngine(now);

  // ---------------- STARTUP ----------------
  if (controllerState == STARTUP_SETTLE) {
    digitalWrite(LASER_PIN, LOW);
    motorCoast();

    if (now - stateStartTime >= STARTUP_SETTLE_MS) {
      clearHoldLearning();
      changeControllerState(SEARCHING, now);
    }
  }

  // ---------------- SEARCH ----------------
  else if (controllerState == SEARCHING) {
    digitalWrite(LASER_PIN, LOW);

    if (anyTargetDetected()) {
      // Search step may still be actively stopping/settling.
      if (!stepIsBusy()) {
        centreProbeAttempts = 0;
        changeControllerState(LOCAL_ALIGN, now);
      }
    }

    else if (!stepIsBusy()) {
      if (stepCompletedEvent) {
        stepCompletedEvent = false;
        advanceSearchSweep();
      }

      startSearchStep(now);
    }
  }

  // ---------------- LOCAL ALIGN ----------------
  else if (controllerState == LOCAL_ALIGN) {
    digitalWrite(LASER_PIN, LOW);

    if (stepIsBusy()) {
      // Wait until wheel is stopped and body has settled.
    }

    else if (alignedAllThree()) {
      alignmentSince = now;
      centreProbeAttempts = 0;
      changeControllerState(ALIGN_CONFIRM, now);
    }

    else if (currentZone != ZONE_NONE) {
      stepCompletedEvent = false;
      startFineStepForCurrentZone(now);
    }

    else {
      // Do not instantly resume a full search after a brief loss.
      if (targetLastSeen != 0 &&
          now - targetLastSeen > LOCAL_TARGET_LOST_MS) {

        clearHoldLearning();
        centreProbeAttempts = 0;
        lastTargetDirection = 0;
        changeControllerState(SEARCHING, now);
      }
    }
  }

  // ---------------- ALIGN CONFIRM ----------------
  else if (controllerState == ALIGN_CONFIRM) {
    digitalWrite(LASER_PIN, LOW);

    if (alignedAllThree() && !stepIsBusy()) {
      // Apply bias only if a real drift direction has already been learned.
      serviceLearnedHold(now);

      if (now - alignmentSince >= ALIGN_CONFIRM_MS &&
          !holdPulseActive) {

        laserStartedAt = now;
        digitalWrite(LASER_PIN, HIGH);
        changeControllerState(FIRING, now);
      }
    }

    else {
      stopHoldPulse();
      alignmentSince = 0;

      if (currentZone == ZONE_LEFT ||
          currentZone == ZONE_LEFT_CENTRE) {

        learnHoldFromCorrection(
          BODY_LEFT,
          currentZone == ZONE_LEFT
        );
      }

      else if (currentZone == ZONE_RIGHT ||
               currentZone == ZONE_CENTRE_RIGHT) {

        learnHoldFromCorrection(
          BODY_RIGHT,
          currentZone == ZONE_RIGHT
        );
      }

      changeControllerState(LOCAL_ALIGN, now);
    }
  }

  // ---------------- FIRING ----------------
  else if (controllerState == FIRING) {
    if (alignedAllThree() && !stepIsBusy()) {
      digitalWrite(LASER_PIN, HIGH);

      // Compensate only a drift direction already observed.
      serviceLearnedHold(now);

      if (now - laserStartedAt >= LASER_DWELL_MS) {
        digitalWrite(LASER_PIN, LOW);
        stopHoldPulse();

        Serial.println(
          ">>> TARGET COMPLETE: 2.05 s uninterrupted L+C+R"
        );

        targetClearSince = 0;
        changeControllerState(WAIT_TARGET_CLEAR, now);
      }
    }

    else {
      // Losing any sensor immediately invalidates the continuous dwell.
      digitalWrite(LASER_PIN, LOW);
      stopHoldPulse();

      Serial.println(
        ">>> ALIGNMENT LOST - laser dwell reset"
      );

      laserStartedAt = 0;
      alignmentSince = 0;

      if (currentZone == ZONE_LEFT ||
          currentZone == ZONE_LEFT_CENTRE) {

        learnHoldFromCorrection(
          BODY_LEFT,
          currentZone == ZONE_LEFT
        );
      }

      else if (currentZone == ZONE_RIGHT ||
               currentZone == ZONE_CENTRE_RIGHT) {

        learnHoldFromCorrection(
          BODY_RIGHT,
          currentZone == ZONE_RIGHT
        );
      }

      changeControllerState(LOCAL_ALIGN, now);
    }
  }

  // ---------------- WAIT TARGET CLEAR ----------------
  else if (controllerState == WAIT_TARGET_CLEAR) {
    digitalWrite(LASER_PIN, LOW);
    stopHoldPulse();
    motorCoast();

    // If the same remote stays held, remain here indefinitely.
    // No repeated 360-degree redetection loop.
    if (currentZone == ZONE_NONE) {
      if (targetClearSince == 0) targetClearSince = now;

      if (now - targetClearSince >= TARGET_CLEAR_MS) {
        clearHoldLearning();
        centreProbeAttempts = 0;
        lastTargetDirection = 0;

        // Reverse the next search direction to reduce string winding.
        searchDirection = -searchDirection;
        searchStepsRemaining = SEARCH_INITIAL_HALF_SWEEP_STEPS;

        changeControllerState(SEARCHING, now);
      }
    } else {
      targetClearSince = 0;
    }
  }

  // ---------------- TELEMETRY ----------------
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    Serial.print("STATE:");
    Serial.print(controllerStateName());

    Serial.print(" | IR:");
    Serial.print(zoneName(currentZone));

    Serial.print(" | ALL3:");
    Serial.print(alignedAllThree() ? "YES" : "NO");

    Serial.print(" | STEP:");
    switch (stepPhase) {
      case STEP_IDLE:       Serial.print("IDLE"); break;
      case STEP_DRIVE:      Serial.print("DRIVE"); break;
      case STEP_STOP_WHEEL: Serial.print("STOP_WHEEL"); break;
      case STEP_SETTLE:     Serial.print("SETTLE"); break;
    }

    Serial.print(" | RPM:");
    Serial.print(wheelRPM, 1);

    Serial.print(" | ENC:");
    Serial.print(encoderCount);

    Serial.print(" | SEARCH:");
    Serial.print(searchDirection == BODY_LEFT ? "LEFT" : "RIGHT");

    Serial.print(" | HOLD:");
    if (learnedHoldDirection == 0) {
      Serial.print("---");
    } else {
      Serial.print(learnedHoldDirection == BODY_LEFT ? "L@" : "R@");
      Serial.print(learnedHoldPower);
    }

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
