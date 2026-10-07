// ============================================================
// MQ SENTINEL - HYBRID CONTINUOUS SEARCH TRACKER V6.2
// V6.2: modulated continuous search + rapid micro-alignment.
//
// PINS
// D2  Left IR
// D3  Centre IR
// D4  Right IR
// D6  Motor IN1 / PWM
// D7  Motor IN2
// D8  Laser
// D9  Encoder A
// D10 Encoder B
// D13 Built-in status LED
//
// OBSERVED HARDWARE MOTION:
//   motorLeftPWM()  -> flywheel CLOCKWISE
//                   -> chassis receives COUNTER-CLOCKWISE torque
//
//   motorRightPWM() -> flywheel COUNTER-CLOCKWISE
//                   -> chassis receives CLOCKWISE torque
//
// Therefore:
//   BODY_CCW torque = motorLeftPWM()
//   BODY_CW  torque = motorRightPWM()
//
// IMPORTANT:
// - We preserve the two motor direction functions from the old program.
// - We DO NOT rely on the old "coast" or "brake" functions to stop motion.
// - Stopping is active: command the opposite motor direction until the
//   encoder shows the flywheel has returned close to its pre-move RPM.
// - Search uses continuously modulated motor power instead of pulses.
// - ANY IR detection immediately stops adding search torque and begins
//   active momentum recovery before precision alignment.
// - Laser requires LEFT + CENTRE + RIGHT continuously.
// - After a successful 2 second shot, a continuously held remote cannot
//   trigger another 360 degree acquisition loop.
// ============================================================

#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES
void UserIRReceiveTimerInterruptHandler();
#include <IRremote.hpp>

// -------------------- Pins --------------------
const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;
const byte LASER_PIN = 8;
const byte ENCODER_A = 9;
const byte ENCODER_B = 10;
const byte STATUS_LED = 13;

// -------------------- Proven motor convention --------------------
const bool REVERSE_MOTOR = false;

// Desired chassis torque directions.
const int BODY_CCW = -1;
const int BODY_CW  = +1;

// -------------------- IR --------------------
IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

unsigned long lastLeftIR = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR = 0;
unsigned long targetLastSeen = 0;

// Remote repeat frames can be spaced tens of ms apart.
const unsigned long IR_HOLD_MS = 140;

// -------------------- Encoder --------------------
const long COUNTS_PER_REV = 700;

long encoderCount = 0;
byte previousEncoderA = LOW;

long previousSpeedCount = 0;
unsigned long previousSpeedTime = 0;
float wheelRPM = 0.0;

// -------------------- Zones --------------------
enum Zone {
  NONE,
  LEFT_ONLY,
  LEFT_CENTRE,
  CENTRE_ONLY,
  ALL_THREE,
  CENTRE_RIGHT,
  RIGHT_ONLY,
  LEFT_RIGHT
};

Zone zone = NONE;
Zone previousZone = NONE;
unsigned long zoneSince = 0;

// -------------------- Main state --------------------
enum MainState {
  STARTUP,
  SEARCH,
  CAPTURE_SEARCH_MOMENTUM,
  ALIGN,
  CONFIRM,
  FIRE,
  WAIT_CLEAR
};

MainState state = STARTUP;
unsigned long stateSince = 0;

const unsigned long STARTUP_MS = 1000;
const unsigned long ALIGN_CONFIRM_MS = 220;
const unsigned long FIRE_TIME_MS = 2100;
const unsigned long CLEAR_CONFIRM_MS = 350;
const unsigned long TARGET_LOST_MS = 900;

unsigned long alignedSince = 0;
unsigned long fireSince = 0;
unsigned long clearSince = 0;

// -------------------- Movement controller --------------------
//
// Each move records the wheel RPM BEFORE the move.
// After the finite drive pulse, V6 commands the opposite motor direction
// until the wheel returns close to that baseline RPM.
//
// Why:
// A finite CHANGE in wheel angular momentum turns the chassis.
// Returning the wheel approximately to its pre-move momentum supplies
// the opposite impulse and removes most of the chassis angular velocity.
//
// The suspension string can still disturb the body, so we then SETTLE
// and use the IR zones for the next correction.
//
enum MovePhase {
  MOVE_IDLE,
  MOVE_DRIVE,
  MOVE_RECOVER,
  MOVE_SETTLE
};

enum MoveType {
  MOVE_NONE,
  MOVE_FINE
};

MovePhase movePhase = MOVE_IDLE;
MoveType moveType = MOVE_NONE;

int bodyDirection = 0;
int drivePower = 0;
long targetEncoderCounts = 0;
long moveStartEncoder = 0;
int moveWheelSign = 0;
Zone moveStartZone = NONE;

float baselineWheelRPM = 0.0;

unsigned long movePhaseSince = 0;
unsigned long settleDuration = 0;
unsigned long baselineStableSince = 0;


// -------------------- Continuous search tuning --------------------
// Search stays powered continuously. Power reduces as the flywheel gains
// speed so the initial acceleration is useful without becoming excessively
// aggressive near higher wheel RPM.
int searchDirection = BODY_CCW;

float searchBaselineRPM = 0.0;
long searchStartEncoder = 0;
int searchWheelSign = 0;
unsigned long searchStarted = 0;
bool searchRunning = false;

const int SEARCH_POWER_MAX  = 150;
const int SEARCH_POWER_HIGH = 130;
const int SEARCH_POWER_MED  = 108;
const int SEARCH_POWER_LOW  = 90;
const int SEARCH_POWER_MIN  = 76;

const float SEARCH_RPM_1 = 25.0;
const float SEARCH_RPM_2 = 55.0;
const float SEARCH_RPM_3 = 85.0;
const float SEARCH_RPM_4 = 110.0;

const unsigned long SEARCH_RAMP_MS = 250;

// On first IR detection, recover the wheel toward its pre-search RPM before
// fine alignment. This removes much of the search-induced body momentum.
unsigned long searchCaptureStarted = 0;
unsigned long searchCaptureStableSince = 0;

const int SEARCH_CAPTURE_POWER_FAST = 165;
const int SEARCH_CAPTURE_POWER_SLOW = 92;
const float SEARCH_CAPTURE_SLOW_BAND = 28.0;
const float SEARCH_CAPTURE_RPM_TOLERANCE = 8.0;
const unsigned long SEARCH_CAPTURE_STABLE_MS = 45;
const unsigned long SEARCH_CAPTURE_TIMEOUT_MS = 480;

// -------------------- Fine tuning --------------------
const long OUTER_COUNTS = 9;
const int OUTER_POWER = 100;
const unsigned long OUTER_SETTLE_MS = 65;

const long INNER_COUNTS = 3;
const int INNER_POWER = 82;
const unsigned long INNER_SETTLE_MS = 50;

const long CENTRE_PROBE_COUNTS = 2;
const int CENTRE_PROBE_POWER = 74;
const unsigned long CENTRE_PROBE_SETTLE_MS = 55;

// -------------------- Recovery tuning --------------------
const int RECOVER_POWER_FAST = 150;
const int RECOVER_POWER_SLOW = 84;

const float BASELINE_RPM_TOLERANCE = 8.0;
const float SLOW_RECOVER_BAND_RPM = 24.0;
const unsigned long BASELINE_STABLE_MS = 35;

const unsigned long DRIVE_TIMEOUT_MS = 260;
const unsigned long RECOVER_TIMEOUT_MS = 340;

// -------------------- Alignment memory --------------------
int lastTargetDirection = 0;
int centreProbeDirection = BODY_CCW;
int centreProbeCount = 0;

// Edge-cone / bad-swivel recovery:
// if IR repeatedly appears then disappears, do NOT resume a large search pulse.
// Reverse the last approach and use tiny rapid reacquisition nudges instead.
int reacquireDirection = 0;
byte edgeLossCount = 0;
unsigned long lastEdgeLossTime = 0;

const unsigned long EDGE_REACQUIRE_WINDOW_MS = 1000;
const long EDGE_REACQUIRE_COUNTS = 2;
const int EDGE_REACQUIRE_POWER = 76;
const unsigned long EDGE_REACQUIRE_SETTLE_MS = 60;
const byte EDGE_REVERSE_AFTER = 2;

// -------------------- String bias learning --------------------
//
// This is deliberately NOT applied while initially searching.
// It is learned only when the body has actually drifted away from ALL3.
//
// A reaction wheel cannot oppose an external string torque indefinitely;
// this is intended only to stabilise the ~2 s firing window.
//
int holdDirection = 0;
int holdPower = 0;

const int HOLD_START_POWER = 24;
const int HOLD_MAX_POWER = 40;
const unsigned long HOLD_PULSE_MS = 2;
const unsigned long HOLD_PERIOD_MS = 50;
const float HOLD_RPM_LIMIT = 105.0;

bool holdPulseActive = false;
unsigned long holdPulseEnd = 0;
unsigned long lastHoldPulse = 0;

// -------------------- LED --------------------
unsigned long lastLedToggle = 0;
bool ledState = false;

// ============================================================
// MOTOR
//
// These two functions are intentionally copied from the old program.
// They are the part we KNOW physically drives your hardware.
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

void driveBodyTorque(int direction, int power) {
  if (direction == BODY_CCW) {
    // Flywheel CW -> body CCW.
    motorLeftPWM(power);
  } else if (direction == BODY_CW) {
    // Flywheel CCW -> body CW.
    motorRightPWM(power);
  }
}

// IMPORTANT:
// We do not use a guessed driver "coast" state between normal moves.
// After active recovery, the motor is left in a zero-torque-ish state
// using the same side of the proven direction interface at power 0.
// If your driver mode makes this behave unexpectedly, only this helper
// needs changing; the actual CW/CCW drive mapping remains proven.
void motorNeutral() {
  motorLeftPWM(0);
}

// ============================================================
// ENCODER
// ============================================================

void updateEncoder() {
  byte currentA = digitalRead(ENCODER_A);

  if (currentA == HIGH && previousEncoderA == LOW) {
    if (digitalRead(ENCODER_B) != currentA) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  previousEncoderA = currentA;
}

void updateWheelSpeed(unsigned long now) {
  if (now - previousSpeedTime < 30) return;

  unsigned long dt = now - previousSpeedTime;
  long delta = encoderCount - previousSpeedCount;

  wheelRPM =
    ((float)delta / (float)COUNTS_PER_REV) *
    (60000.0f / (float)dt);

  previousSpeedCount = encoderCount;
  previousSpeedTime = now;
}

float absf(float x) {
  return x < 0.0f ? -x : x;
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

Zone calculateZone(bool L, bool C, bool R) {
  if (L && C && R) return ALL_THREE;
  if (L && C && !R) return LEFT_CENTRE;
  if (!L && C && R) return CENTRE_RIGHT;
  if (L && !C && !R) return LEFT_ONLY;
  if (!L && !C && R) return RIGHT_ONLY;
  if (!L && C && !R) return CENTRE_ONLY;
  if (L && !C && R) return LEFT_RIGHT;
  return NONE;
}

void updateZone(bool L, bool C, bool R, unsigned long now) {
  Zone newZone = calculateZone(L, C, R);

  if (newZone != zone) {
    previousZone = zone;
    zone = newZone;
    zoneSince = now;
  }

  if (zone == LEFT_ONLY || zone == LEFT_CENTRE) {
    lastTargetDirection = BODY_CCW;
    centreProbeDirection = BODY_CCW;
  } else if (zone == RIGHT_ONLY || zone == CENTRE_RIGHT) {
    lastTargetDirection = BODY_CW;
    centreProbeDirection = BODY_CW;
  }
}

bool targetVisible() {
  return zone != NONE;
}

bool perfectlyAligned() {
  return zone == ALL_THREE;
}

// ============================================================
// MOVE ENGINE
// ============================================================

bool moveBusy() {
  return movePhase != MOVE_IDLE;
}

void startMove(
  int desiredBodyDirection,
  long encoderCounts,
  int power,
  unsigned long settleMs,
  MoveType type,
  unsigned long now
) {
  if (moveBusy()) return;

  bodyDirection = desiredBodyDirection;
  targetEncoderCounts = encoderCounts;
  drivePower = power;
  settleDuration = settleMs;
  moveType = type;

  moveStartEncoder = encoderCount;
  moveWheelSign = 0;
  moveStartZone = zone;

  // Critical difference from simply "stopping the wheel":
  // recover to the momentum state that existed before THIS correction.
  baselineWheelRPM = wheelRPM;

  baselineStableSince = 0;

  movePhase = MOVE_DRIVE;
  movePhaseSince = now;

  driveBodyTorque(bodyDirection, drivePower);
}

void startRecovery(unsigned long now) {
  movePhase = MOVE_RECOVER;
  movePhaseSince = now;
  baselineStableSince = 0;

  // Opposite BODY torque means opposite motor/flywheel acceleration.
  driveBodyTorque(-bodyDirection, RECOVER_POWER_FAST);
}

void startSettle(unsigned long now) {
  motorNeutral();

  movePhase = MOVE_SETTLE;
  movePhaseSince = now;
}

void finishMove() {
  motorNeutral();

  movePhase = MOVE_IDLE;
  moveType = MOVE_NONE;
}

void serviceMove(unsigned long now) {
  if (movePhase == MOVE_IDLE) return;

  if (movePhase == MOVE_DRIVE) {
    long displacement = encoderCount - moveStartEncoder;
    long moved = labs(displacement);

    if (displacement > 0) {
      moveWheelSign = +1;
    } else if (displacement < 0) {
      moveWheelSign = -1;
    }

    // Fine pulses are intentionally interruptible. As soon as the sensor
    // geometry changes (especially if ALL3 appears), stop adding torque and
    // recover the wheel instead of finishing the entire requested pulse.
    if (
      moveType == MOVE_FINE &&
      (perfectlyAligned() || (targetVisible() && zone != moveStartZone))
    ) {
      startRecovery(now);
      return;
    }

    if (moved >= targetEncoderCounts) {
      startRecovery(now);
      return;
    }

    if (now - movePhaseSince >= DRIVE_TIMEOUT_MS) {
      startRecovery(now);
      return;
    }
  }

  else if (movePhase == MOVE_RECOVER) {
    float error = baselineWheelRPM - wheelRPM;

    // If wheel has returned close to its pre-move RPM, keep it there
    // briefly before releasing the motor.
    if (absf(error) <= BASELINE_RPM_TOLERANCE) {
      motorNeutral();

      if (baselineStableSince == 0) {
        baselineStableSince = now;
      }

      if (now - baselineStableSince >= BASELINE_STABLE_MS) {
        startSettle(now);
        return;
      }
    } else {
      baselineStableSince = 0;

      // Determine which known motor direction changes wheel speed toward
      // the recorded baseline. The sign mapping is inferred from the
      // original observed move direction:
      //
      // current correction used BODY_CCW -> motorLeft -> wheel CW.
      // To undo it, BODY_CW -> motorRight.
      //
      // If recovery overshoots the baseline, switch back.
      bool needUndoOriginalChange;

      // We do not need to know whether encoder CW is + or -. The drive
      // phase stores the encoder sign before recovery starts.
      if (moveWheelSign == 0) {
        // Encoder has not provided useful direction information yet.
        driveBodyTorque(-bodyDirection, RECOVER_POWER_FAST);
      } else {
        float signedErrorAlongMove = error * (float)moveWheelSign;

        // error opposite the move-wheel sign => need to undo move.
        needUndoOriginalChange = signedErrorAlongMove < 0.0f;

        int recoveryBodyDirection =
          needUndoOriginalChange
          ? -bodyDirection
          : bodyDirection;

        int recoveryPower =
          absf(error) > SLOW_RECOVER_BAND_RPM
          ? RECOVER_POWER_FAST
          : RECOVER_POWER_SLOW;

        driveBodyTorque(recoveryBodyDirection, recoveryPower);
      }
    }

    if (now - movePhaseSince >= RECOVER_TIMEOUT_MS) {
      // Do not keep reversing forever if encoder feedback is imperfect.
      startSettle(now);
      return;
    }
  }

  else if (movePhase == MOVE_SETTLE) {
    motorNeutral();

    if (now - movePhaseSince >= settleDuration) {
      finishMove();
      return;
    }
  }
}

// ============================================================
// CONTINUOUS SEARCH
// ============================================================

int searchPowerForCurrentSpeed(unsigned long now) {
  float deltaRPM = absf(wheelRPM - searchBaselineRPM);
  int requestedPower;

  if (deltaRPM < SEARCH_RPM_1) {
    requestedPower = SEARCH_POWER_MAX;
  } else if (deltaRPM < SEARCH_RPM_2) {
    requestedPower = SEARCH_POWER_HIGH;
  } else if (deltaRPM < SEARCH_RPM_3) {
    requestedPower = SEARCH_POWER_MED;
  } else if (deltaRPM < SEARCH_RPM_4) {
    requestedPower = SEARCH_POWER_LOW;
  } else {
    requestedPower = SEARCH_POWER_MIN;
  }

  // Ramp from the minimum power to the requested power over the first
  // quarter second so search begins smoothly rather than with one hard kick.
  unsigned long elapsed = now - searchStarted;
  if (elapsed < SEARCH_RAMP_MS) {
    long rampPower = map(
      (long)elapsed,
      0L,
      (long)SEARCH_RAMP_MS,
      (long)SEARCH_POWER_MIN,
      (long)requestedPower
    );

    requestedPower = constrain(
      (int)rampPower,
      SEARCH_POWER_MIN,
      SEARCH_POWER_MAX
    );
  }

  return requestedPower;
}

void beginContinuousSearch(unsigned long now) {
  searchBaselineRPM = wheelRPM;
  searchStartEncoder = encoderCount;
  searchWheelSign = 0;
  searchStarted = now;
  searchRunning = true;
}

void serviceContinuousSearch(unsigned long now) {
  if (!searchRunning) {
    beginContinuousSearch(now);
  }

  long displacement = encoderCount - searchStartEncoder;
  if (displacement > 0) {
    searchWheelSign = +1;
  } else if (displacement < 0) {
    searchWheelSign = -1;
  }

  driveBodyTorque(
    searchDirection,
    searchPowerForCurrentSpeed(now)
  );
}

void beginSearchCapture(unsigned long now) {
  searchRunning = false;
  searchCaptureStarted = now;
  searchCaptureStableSince = 0;

  // Immediately oppose the search-induced body torque.
  driveBodyTorque(
    -searchDirection,
    SEARCH_CAPTURE_POWER_FAST
  );
}

bool serviceSearchCapture(unsigned long now) {
  float error = searchBaselineRPM - wheelRPM;

  if (absf(error) <= SEARCH_CAPTURE_RPM_TOLERANCE) {
    motorNeutral();

    if (searchCaptureStableSince == 0) {
      searchCaptureStableSince = now;
    }

    if (
      now - searchCaptureStableSince >=
      SEARCH_CAPTURE_STABLE_MS
    ) {
      motorNeutral();
      return true;
    }
  } else {
    searchCaptureStableSince = 0;

    if (searchWheelSign == 0) {
      driveBodyTorque(
        -searchDirection,
        SEARCH_CAPTURE_POWER_FAST
      );
    } else {
      float signedErrorAlongSearch =
        error * (float)searchWheelSign;

      bool undoSearchChange =
        signedErrorAlongSearch < 0.0f;

      int correctionDirection =
        undoSearchChange
        ? -searchDirection
        : searchDirection;

      int power =
        absf(error) > SEARCH_CAPTURE_SLOW_BAND
        ? SEARCH_CAPTURE_POWER_FAST
        : SEARCH_CAPTURE_POWER_SLOW;

      driveBodyTorque(correctionDirection, power);
    }
  }

  if (
    now - searchCaptureStarted >=
    SEARCH_CAPTURE_TIMEOUT_MS
  ) {
    motorNeutral();
    return true;
  }

  return false;
}

// ============================================================
// HOLD LEARNING
// ============================================================

void stopHoldPulse() {
  if (holdPulseActive) {
    holdPulseActive = false;
    motorNeutral();
  }
}

void clearHold() {
  stopHoldPulse();
  holdDirection = 0;
  holdPower = 0;
}

void learnHold(int correctionDirection, bool outer) {
  if (correctionDirection == 0) return;

  if (holdDirection != correctionDirection) {
    holdDirection = correctionDirection;
    holdPower = HOLD_START_POWER;
  } else {
    holdPower += outer ? 4 : 2;
    holdPower = constrain(holdPower, HOLD_START_POWER, HOLD_MAX_POWER);
  }
}

void serviceHold(unsigned long now) {
  if (holdDirection == 0 || holdPower <= 0) return;

  if (absf(wheelRPM) >= HOLD_RPM_LIMIT) {
    stopHoldPulse();
    return;
  }

  if (holdPulseActive) {
    if (now >= holdPulseEnd) {
      holdPulseActive = false;
      motorNeutral();
    }
    return;
  }

  if (now - lastHoldPulse >= HOLD_PERIOD_MS) {
    driveBodyTorque(holdDirection, holdPower);
    holdPulseActive = true;
    holdPulseEnd = now + HOLD_PULSE_MS;
    lastHoldPulse = now;
  }
}

// ============================================================
// EDGE REACQUISITION
// ============================================================

void rememberEdgeLoss(unsigned long now) {
  if (now - lastEdgeLossTime > EDGE_REACQUIRE_WINDOW_MS) {
    edgeLossCount = 0;
  }

  lastEdgeLossTime = now;
  edgeLossCount++;

  // The string often springs the body back across the edge of the cone.
  // First try back toward the last known target. If that same edge is lost
  // repeatedly, reverse the tiny nudge instead of pumping the oscillation.
  int towardTarget =
    lastTargetDirection != 0
    ? lastTargetDirection
    : centreProbeDirection;

  reacquireDirection =
    edgeLossCount >= EDGE_REVERSE_AFTER
    ? -towardTarget
    : towardTarget;
}

void startEdgeReacquire(unsigned long now) {
  int dir =
    reacquireDirection != 0
    ? reacquireDirection
    : (lastTargetDirection != 0 ? lastTargetDirection : centreProbeDirection);

  startMove(
    dir,
    EDGE_REACQUIRE_COUNTS,
    EDGE_REACQUIRE_POWER,
    EDGE_REACQUIRE_SETTLE_MS,
    MOVE_FINE,
    now
  );
}

// ============================================================
// FINE ALIGNMENT
// ============================================================

void startFineCorrection(unsigned long now) {
  if (zone == LEFT_ONLY) {
    learnHold(BODY_CCW, true);

    startMove(
      BODY_CCW,
      OUTER_COUNTS,
      OUTER_POWER,
      OUTER_SETTLE_MS,
      MOVE_FINE,
      now
    );
    return;
  }

  if (zone == LEFT_CENTRE) {
    learnHold(BODY_CCW, false);

    startMove(
      BODY_CCW,
      INNER_COUNTS,
      INNER_POWER,
      INNER_SETTLE_MS,
      MOVE_FINE,
      now
    );
    return;
  }

  if (zone == RIGHT_ONLY) {
    learnHold(BODY_CW, true);

    startMove(
      BODY_CW,
      OUTER_COUNTS,
      OUTER_POWER,
      OUTER_SETTLE_MS,
      MOVE_FINE,
      now
    );
    return;
  }

  if (zone == CENTRE_RIGHT) {
    learnHold(BODY_CW, false);

    startMove(
      BODY_CW,
      INNER_COUNTS,
      INNER_POWER,
      INNER_SETTLE_MS,
      MOVE_FINE,
      now
    );
    return;
  }

  if (zone == CENTRE_ONLY) {
    // Centre-only is close, but DOES NOT fire.
    // Probe a tiny distance. If repeated probes fail, reverse direction.
    centreProbeCount++;

    if (centreProbeCount > 2) {
      centreProbeDirection = -centreProbeDirection;
      centreProbeCount = 1;
    }

    startMove(
      centreProbeDirection,
      CENTRE_PROBE_COUNTS,
      CENTRE_PROBE_POWER,
      CENTRE_PROBE_SETTLE_MS,
      MOVE_FINE,
      now
    );
    return;
  }

  if (zone == LEFT_RIGHT) {
    int dir =
      lastTargetDirection != 0
      ? lastTargetDirection
      : centreProbeDirection;

    startMove(
      dir,
      CENTRE_PROBE_COUNTS,
      CENTRE_PROBE_POWER,
      CENTRE_PROBE_SETTLE_MS,
      MOVE_FINE,
      now
    );
  }
}

// ============================================================
// STATE / LED
// ============================================================

void changeState(MainState next, unsigned long now) {
  if (state == next) return;

  state = next;
  stateSince = now;
}

void updateStatusLED(unsigned long now) {
  // Solid = ALL3 / firing.
  if (state == CONFIRM || state == FIRE) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }

  // Fast blink = local target alignment.
  unsigned long interval = 450;
  if (state == ALIGN) {
    interval = 100;
  } else if (state == CAPTURE_SEARCH_MOMENTUM) {
    interval = 200;
  }

  if (state == STARTUP) {
    digitalWrite(STATUS_LED, LOW);
    return;
  }

  if (now - lastLedToggle >= interval) {
    lastLedToggle = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  digitalWrite(LASER_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);

  previousEncoderA = digitalRead(ENCODER_A);
  previousSpeedTime = millis();

  // Use only the proven motor interface.
  motorNeutral();

  IrReceiver.begin(LEFT_SENSOR, false);
  centreReceiver.begin(CENTRE_SENSOR, false);
  rightReceiver.begin(RIGHT_SENSOR, false);

  stateSince = millis();
  zoneSince = millis();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  updateEncoder();
  updateWheelSpeed(now);
  readIR(now);

  bool L =
    lastLeftIR != 0 &&
    now - lastLeftIR <= IR_HOLD_MS;

  bool C =
    lastCentreIR != 0 &&
    now - lastCentreIR <= IR_HOLD_MS;

  bool R =
    lastRightIR != 0 &&
    now - lastRightIR <= IR_HOLD_MS;

  updateZone(L, C, R, now);

  serviceMove(now);
  updateStatusLED(now);

  // ---------------- STARTUP ----------------
  if (state == STARTUP) {
    digitalWrite(LASER_PIN, LOW);
    motorNeutral();

    if (now - stateSince >= STARTUP_MS) {
      beginContinuousSearch(now);
      changeState(SEARCH, now);
    }
  }

  // ---------------- SEARCH ----------------
  else if (state == SEARCH) {
    digitalWrite(LASER_PIN, LOW);

    if (targetVisible()) {
      // Detection instantly stops adding search torque. First recover the
      // flywheel toward its pre-search RPM, then begin fine alignment.
      beginSearchCapture(now);
      changeState(CAPTURE_SEARCH_MOMENTUM, now);
    } else {
      serviceContinuousSearch(now);
    }
  }

  // ---------------- CAPTURE SEARCH MOMENTUM ----------------
  else if (state == CAPTURE_SEARCH_MOMENTUM) {
    digitalWrite(LASER_PIN, LOW);

    if (serviceSearchCapture(now)) {
      centreProbeCount = 0;
      changeState(ALIGN, now);
    }
  }

  // ---------------- ALIGN ----------------
  else if (state == ALIGN) {
    digitalWrite(LASER_PIN, LOW);

    if (moveBusy()) {
      // Wait until drive + recovery + settling are complete.
    } else if (perfectlyAligned()) {
      alignedSince = now;
      centreProbeCount = 0;
      edgeLossCount = 0;
      reacquireDirection = 0;
      changeState(CONFIRM, now);
    } else if (targetVisible()) {
      startFineCorrection(now);
    } else {
      // Signal flicker at the cone edge is common with the wound suspension.
      // Stay local and use a tiny nudge instead of falling straight back into
      // a large search step, which can reinforce the oscillation.
      if (
        targetLastSeen != 0 &&
        now - targetLastSeen >= 90 &&
        now - targetLastSeen < TARGET_LOST_MS
      ) {
        rememberEdgeLoss(now);
        startEdgeReacquire(now);
      } else if (
        targetLastSeen != 0 &&
        now - targetLastSeen >= TARGET_LOST_MS
      ) {
        clearHold();
        edgeLossCount = 0;
        reacquireDirection = 0;
        lastTargetDirection = 0;
        centreProbeCount = 0;

        // Alternate direction after a failed local reacquisition to avoid
        // repeatedly winding the suspension in the same direction.
        searchDirection = -searchDirection;
        beginContinuousSearch(now);
        changeState(SEARCH, now);
      }
    }
  }

  // ---------------- CONFIRM ----------------
  else if (state == CONFIRM) {
    digitalWrite(LASER_PIN, LOW);

    if (perfectlyAligned() && !moveBusy()) {
      serviceHold(now);

      if (
        now - alignedSince >= ALIGN_CONFIRM_MS &&
        !holdPulseActive
      ) {
        fireSince = now;
        digitalWrite(LASER_PIN, HIGH);
        changeState(FIRE, now);
      }
    } else {
      stopHoldPulse();
      alignedSince = 0;
      rememberEdgeLoss(now);

      if (zone == LEFT_ONLY || zone == LEFT_CENTRE) {
        learnHold(BODY_CCW, zone == LEFT_ONLY);
      } else if (zone == RIGHT_ONLY || zone == CENTRE_RIGHT) {
        learnHold(BODY_CW, zone == RIGHT_ONLY);
      }

      changeState(ALIGN, now);
    }
  }

  // ---------------- FIRE ----------------
  else if (state == FIRE) {
    // Hard safety/accuracy gate:
    // laser can only remain on while ALL THREE are active.
    if (perfectlyAligned() && !moveBusy()) {
      digitalWrite(LASER_PIN, HIGH);
      serviceHold(now);

      if (now - fireSince >= FIRE_TIME_MS) {
        digitalWrite(LASER_PIN, LOW);
        stopHoldPulse();
        clearSince = 0;
        changeState(WAIT_CLEAR, now);
      }
    } else {
      // The assignment requires uninterrupted alignment.
      // Losing one receiver resets the 2 second dwell.
      digitalWrite(LASER_PIN, LOW);
      stopHoldPulse();
      fireSince = 0;
      alignedSince = 0;
      rememberEdgeLoss(now);

      if (zone == LEFT_ONLY || zone == LEFT_CENTRE) {
        learnHold(BODY_CCW, zone == LEFT_ONLY);
      } else if (zone == RIGHT_ONLY || zone == CENTRE_RIGHT) {
        learnHold(BODY_CW, zone == RIGHT_ONLY);
      }

      changeState(ALIGN, now);
    }
  }

  // ---------------- WAIT CLEAR ----------------
  else if (state == WAIT_CLEAR) {
    digitalWrite(LASER_PIN, LOW);
    stopHoldPulse();
    motorNeutral();

    // This prevents a held remote from causing:
    // detect -> rotate 360 -> detect -> rotate 360 -> ...
    if (zone == NONE) {
      if (clearSince == 0) clearSince = now;

      if (now - clearSince >= CLEAR_CONFIRM_MS) {
        clearHold();

        // Reverse the next continuous scan to reduce repeated string wind-up.
        searchDirection = -searchDirection;

        lastTargetDirection = 0;
        centreProbeCount = 0;
        edgeLossCount = 0;
        reacquireDirection = 0;

        beginContinuousSearch(now);
        changeState(SEARCH, now);
      }
    } else {
      clearSince = 0;
    }
  }

  // Optional telemetry remains available if USB serial is ever usable.
  static unsigned long lastPrint = 0;

  if (now - lastPrint >= 200) {
    lastPrint = now;

    Serial.print("STATE:");
    Serial.print((int)state);

    Serial.print(" ZONE:");
    Serial.print((int)zone);

    Serial.print(" MOVE:");
    Serial.print((int)movePhase);

    Serial.print(" RPM:");
    Serial.print(wheelRPM, 1);

    Serial.print(" FINE_BASE:");
    Serial.print(baselineWheelRPM, 1);

    Serial.print(" SEARCH_BASE:");
    Serial.print(searchBaselineRPM, 1);

    Serial.print(" ENC:");
    Serial.print(encoderCount);

    Serial.print(" LASER:");
    Serial.println(digitalRead(LASER_PIN) ? "ON" : "OFF");
  }
}

// ============================================================
// MULTIPLE IR RECEIVER INTERRUPT HANDLER
// ============================================================

void UserIRReceiveTimerInterruptHandler() {
  centreReceiver.ReceiveInterruptHandler();
  rightReceiver.ReceiveInterruptHandler();
}
