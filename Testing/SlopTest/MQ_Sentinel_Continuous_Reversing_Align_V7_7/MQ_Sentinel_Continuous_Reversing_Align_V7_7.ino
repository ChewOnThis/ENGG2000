// ============================================================
// MQ SENTINEL - CONTINUOUS REVERSING ALIGN V7.7
//
// Based on the working V6 motor/sensor layout.
//
// CONTROL STRATEGY
// 1. SEARCH: slow, continuous, mid-strength rotation.
//    No large search pulses.
// 2. ANY IR DETECTION: immediately remove most search momentum.
// 3. LEFT sensor: ALWAYS use the same 1-count CCW micro-correction.
// 4. RIGHT sensor: ALWAYS use the same 1-count CW micro-correction.
//    Acquisition and fine tracking now use the SAME control law.
// 5. LEFT+CENTRE / CENTRE+RIGHT: even smaller corrections.
// 6. ALL THREE: stop correcting, confirm alignment, then fire laser.
// 7. If the string/swivel pulls the Sentinel away, use tiny local
//    reacquisition corrections instead of returning immediately to search.
// 8. After a successful shot, wait for the target to disappear before
//    searching again.
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
// PHYSICALLY OBSERVED MOTOR MAPPING FROM V6
// motorLeftPWM()  -> flywheel CLOCKWISE
//                 -> chassis COUNTER-CLOCKWISE torque
//
// motorRightPWM() -> flywheel COUNTER-CLOCKWISE
//                 -> chassis CLOCKWISE torque
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
const byte STATUS_LED = 13;

// ============================================================
// DIRECTION CONVENTION
// ============================================================

const bool REVERSE_MOTOR = false;

const int BODY_CCW = -1;
const int BODY_CW  = +1;

// Current physical sensor geometry from V6:
// LEFT target requires CCW chassis correction.
// RIGHT target requires CW chassis correction.
const int LEFT_CORRECTION_DIRECTION  = BODY_CCW;
const int RIGHT_CORRECTION_DIRECTION = BODY_CW;

// ============================================================
// IR RECEIVERS
// ============================================================

IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

unsigned long lastLeftIR = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR = 0;
unsigned long targetLastSeen = 0;

// Keeps decoded 38 kHz remote frames "present" briefly between frames.
const unsigned long IR_HOLD_MS = 140;

// ============================================================
// ENCODER
// ============================================================

const long COUNTS_PER_REV = 700;

long encoderCount = 0;
byte previousEncoderA = LOW;

long previousSpeedCount = 0;
unsigned long previousSpeedTime = 0;
float wheelRPM = 0.0;

// ============================================================
// SENSOR ZONES
// ============================================================

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

// ============================================================
// MAIN STATES
// ============================================================

enum MainState {
  STARTUP,
  SEARCH,
  CAPTURE_SEARCH,
  ALIGN,
  CONFIRM,
  FIRE,
  WAIT_CLEAR
};

MainState state = STARTUP;
unsigned long stateSince = 0;

const unsigned long STARTUP_MS = 0;

// Require a short stable ALL_THREE period before firing.
const unsigned long ALIGN_CONFIRM_MS = 220;

// Assignment firing dwell.
const unsigned long FIRE_TIME_MS = 2100;

// Same target must disappear before another search.
const unsigned long CLEAR_CONFIRM_MS = 350;

// Only return to broad search after the target has really been lost.
const unsigned long TARGET_LOST_MS = 900;

unsigned long alignedSince = 0;
unsigned long fireSince = 0;
unsigned long clearSince = 0;

// ============================================================
// SEARCH TUNING
// ============================================================
//
// Search is CONTINUOUS PWM, not a sequence of encoder pulses.
//
// As wheel speed builds relative to the RPM at which search began,
// power is reduced. This keeps the seek motion slow instead of allowing
// the reaction wheel to keep accelerating aggressively.
//
// Start here for testing:
//   118 = moderate search torque
//   102 = medium-low
//    88 = gentle once wheel momentum has built
//
// If search is still too fast, reduce all three by ~5-10.
// ============================================================

int searchDirection = BODY_CCW;

float searchBaselineRPM = 0.0;
long searchStartEncoder = 0;
int searchWheelSign = 0;
unsigned long searchStarted = 0;
bool searchRunning = false;

const int SEARCH_POWER_START = 105;
const int SEARCH_POWER_MID   = 94;
const int SEARCH_POWER_LOW   = 84;

const float SEARCH_RPM_MID = 35.0;
const float SEARCH_RPM_LOW = 65.0;

// Smooth the first instant of search so the chassis is not kicked.
const unsigned long SEARCH_RAMP_MS = 500;
const int SEARCH_RAMP_START_POWER = 70;

// No timed search reversal.
// V7 reversed after 5.5 s; on the physical Sentinel this stored wheel/string
// energy and produced the sudden rapid multi-revolution behaviour.
// Search now remains a gentle continuous sweep until IR is actually seen.
bool captureForTarget = true;

// ============================================================
// SEARCH-TO-ALIGN TRANSITION
// ============================================================
//
// V7 attempted to return the reaction wheel all the way to its pre-search
// RPM when a sensor was first detected. After several seconds of continuous
// search this could dump a large amount of stored wheel momentum into the
// chassis and cause the sudden fast spin.
//
// V7.1 does NOT perform that large opposite-direction recovery.
// On first IR detection it removes search drive, gives the chassis a short
// neutral settling window, then lets the sensor-directed micro controller
// decide the next direction.
//
// This is much safer for the wound-string suspension.
// ============================================================

unsigned long searchCaptureStarted = 0;
const unsigned long SEARCH_TO_ALIGN_SETTLE_MS = 220;

// ============================================================
// CONTINUOUS ALIGNMENT TUNING - V7.7
// ============================================================
//
// No alignment pulses. No 14 ms / 2 ms drive-gap cycle.
//
// LEFT and RIGHT use exactly equal power:
//   LEFT_ONLY  / RIGHT_ONLY  = 72
//   LEFT+C     / C+RIGHT     = 54
//
// If the opposite side sensor becomes active, V7.7 continuously commands
// the new direction at REVERSAL power until the encoder shows that the old
// flywheel motion has reached/crossed zero. It then returns to gentle power.
//
// CENTRE_ONLY / ALL_THREE actively oppose the most recent steering direction
// to arrest overshoot, then release the motor.
// ============================================================

const int ALIGN_OUTER_POWER = 72;
const int ALIGN_INNER_POWER = 54;
const int ALIGN_REVERSAL_POWER = 105;
const int CENTRE_ARREST_POWER = 72;

const float REVERSAL_ZERO_RPM = 7.0f;
const unsigned long REVERSAL_TIMEOUT_MS = 260;

const float CENTRE_STOP_RPM = 7.0f;
const unsigned long CENTRE_ARREST_TIMEOUT_MS = 220;
const unsigned long CENTRE_STABLE_MS = 70;

const unsigned long ZONE_DECISION_STABLE_MS = 22;

// Exact software symmetry.
const float LEFT_TORQUE_SCALE  = 1.00f;
const float RIGHT_TORQUE_SCALE = 1.00f;

enum AlignDriveMode {
  ALIGN_DRIVE_IDLE,
  ALIGN_DRIVE_TRACK,
  ALIGN_DRIVE_REVERSING,
  ALIGN_DRIVE_CENTRE_ARREST
};

AlignDriveMode alignDriveMode = ALIGN_DRIVE_IDLE;

int alignCommandDirection = 0;
int alignRequestedDirection = 0;
int alignRequestedPower = 0;
int lastSteeringDirection = 0;

float reversalStartRPM = 0.0f;
unsigned long alignModeSince = 0;
unsigned long centreStableSince = 0;

// ============================================================
// LEGACY MICRO ALIGNMENT ENGINE
// ============================================================
// Retained below only so the older helper functions still compile.
// It is NOT serviced in the main loop in V7.7.

// ============================================================
// MICRO ALIGNMENT ENGINE
// ============================================================
//
// This is intentionally MUCH smaller than V6.
//
// LEFT ONLY / RIGHT ONLY:
//   2 encoder counts.
//
// LEFT+CENTRE / CENTRE+RIGHT:
//   1 encoder count.
//
// CENTRE ONLY / edge reacquisition:
//   1 encoder count.
//
// Power is still sufficient to overcome motor/gear friction, but each
// correction is extremely short. The controller then actively returns the
// flywheel close to the pre-correction RPM.
//
// The next correction can begin almost immediately.
// ============================================================

enum MicroPhase {
  MICRO_IDLE,
  MICRO_DRIVE,
  MICRO_RECOVER,
  MICRO_SETTLE
};

MicroPhase microPhase = MICRO_IDLE;

int microBodyDirection = 0;
long microTargetCounts = 0;
int microPower = 0;
unsigned long microSettleMs = 0;

long microStartEncoder = 0;
int microWheelSign = 0;
float microBaselineRPM = 0.0;

Zone microStartZone = NONE;

// Smooth torque ramping for micro corrections.
// The previous versions applied the entire PWM command instantly, then
// applied a strong opposite recovery command. On the suspended chassis
// that looks like a sharp kick in each direction.
//
// V7.3 ramps into each correction and into recovery instead.
int microAppliedPower = 0;
const int MICRO_DRIVE_START_POWER = 54;
const int MICRO_RECOVER_START_POWER = 48;
const unsigned long MICRO_DRIVE_RAMP_MS = 32;
const unsigned long MICRO_RECOVER_RAMP_MS = 45;

unsigned long microPhaseSince = 0;
unsigned long microStableSince = 0;

// Outer sensor: target is definitely off centre.
const long OUTER_MICRO_COUNTS = 1;
const int OUTER_MICRO_POWER = 72;
const unsigned long OUTER_MICRO_SETTLE_MS = 30;

// Two sensors: already close, so correction is half as large.
const long INNER_MICRO_COUNTS = 1;
const int INNER_MICRO_POWER = 72;
const unsigned long INNER_MICRO_SETTLE_MS = 30;

// Centre only / ambiguous: smallest possible encoder correction.
const long PINHOLE_MICRO_COUNTS = 1;
const int PINHOLE_MICRO_POWER = 68;
const unsigned long PINHOLE_MICRO_SETTLE_MS = 30;

// Active recovery after every tiny correction.
const int MICRO_RECOVER_POWER_FAST = 82;
const int MICRO_RECOVER_POWER_SLOW = 60;

const float MICRO_RPM_TOLERANCE = 7.0;
const float MICRO_RECOVER_SLOW_BAND_RPM = 18.0;

const unsigned long MICRO_BASELINE_STABLE_MS = 24;
const unsigned long MICRO_DRIVE_TIMEOUT_MS = 150;
const unsigned long MICRO_RECOVER_TIMEOUT_MS = 260;

// ============================================================
// ALIGNMENT MEMORY
// ============================================================

int lastTargetDirection = 0;

// Used when CENTRE_ONLY is detected and neither side tells us which way
// to make the final one-count probe.
int centreProbeDirection = BODY_CCW;
byte centreProbeAttempts = 0;

// IMPORTANT PHYSICAL OBSERVATION:
// Before the centre receiver has ever seen the target, the left/right
// acquisition directions on this suspended chassis are opposite to the
// directions that work once centre has been acquired.
//
// false:
//   LEFT only  -> BODY_CW
//   RIGHT only -> BODY_CCW
//
// true:
//   LEFT side  -> BODY_CCW
//   RIGHT side -> BODY_CW
//
// This flag changes automatically as soon as any zone containing CENTRE
// is observed.
bool centreSeenThisAcquisition = false;

// ------------------------------------------------------------
// CENTRE-CROSSING MEMORY
// ------------------------------------------------------------
// The real chassis has inertia and string torsion. A static rule such as
// "LEFT means one direction, RIGHT means the other" is not enough.
//
// Example observed physically:
//   RIGHT -> CENTRE -> LEFT
//
// That sequence means we crossed the target centre and overshot.
// The correct action on LEFT is therefore to REVERSE the direction that
// carried us from RIGHT into CENTRE.
//
// We remember the direction used to approach centre and, for a short
// window after centre is first crossed, use the transition history to
// choose the correction direction.
//
enum ApproachSide {
  APPROACH_NONE,
  APPROACH_FROM_LEFT,
  APPROACH_FROM_RIGHT
};

ApproachSide approachSide = APPROACH_NONE;
int approachDirection = 0;

bool centreCrossingGuard = false;
unsigned long centreCrossedAt = 0;

const unsigned long CENTRE_CROSSING_MEMORY_MS = 750;
const unsigned long CENTRE_CAPTURE_DWELL_MS = 160;

// Ignore very short zone-edge flicker before starting another micro move.
const unsigned long ZONE_DECISION_STABLE_MS = 32;

// ============================================================
// EDGE-OF-CONE REACQUISITION
// ============================================================
//
// Bad swivel/string behaviour often causes:
// detect -> spring away -> detect -> spring away.
//
// When IR disappears briefly during alignment, V7 stays local and performs
// only ONE-count corrections. It does NOT immediately resume broad search.
// ============================================================

unsigned long lastEdgeMicro = 0;
const unsigned long EDGE_MICRO_PERIOD_MS = 55;
const unsigned long EDGE_WAIT_BEFORE_CORRECTION_MS = 70;

byte repeatedEdgeLosses = 0;
unsigned long lastEdgeLossTime = 0;
const unsigned long EDGE_LOSS_WINDOW_MS = 1000;

// ============================================================
// SHORT-TERM HOLD DURING ALL-THREE
// ============================================================
//
// We only learn this from an actual drift direction.
// Once the target is recovered, very small hold pulses can oppose the
// torsion in the suspension string during the required ~2 second shot.
//
// Keep these deliberately weak.
// ============================================================

int holdDirection = 0;
int holdPower = 0;

const int HOLD_START_POWER = 14;
const int HOLD_MAX_POWER = 24;

const unsigned long HOLD_PULSE_MS = 1;
const unsigned long HOLD_PERIOD_MS = 65;

const float HOLD_RPM_LIMIT = 100.0;

bool holdPulseActive = false;
unsigned long holdPulseEnd = 0;
unsigned long lastHoldPulse = 0;

// ============================================================
// STATUS LED
// ============================================================

unsigned long lastLedToggle = 0;
bool ledState = false;

// ============================================================
// MOTOR FUNCTIONS
// PRESERVED FROM THE WORKING V6
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
    motorLeftPWM(power);
  } else if (direction == BODY_CW) {
    motorRightPWM(power);
  }
}

void driveBodyTorqueBalanced(int direction, int power) {
  int scaledPower = power;

  if (direction == BODY_CCW) {
    scaledPower = (int)(power * LEFT_TORQUE_SCALE);
  } else if (direction == BODY_CW) {
    scaledPower = (int)(power * RIGHT_TORQUE_SCALE);
  }

  driveBodyTorque(
    direction,
    constrain(scaledPower, 0, 255)
  );
}

// Same requested torque on both sides.
// Scales default to 1.00 / 1.00 so LEFT and RIGHT receive equal command power.
// Same neutral behaviour as the known-working V6.
void motorNeutral() {
  motorLeftPWM(0);
}

// IMPORTANT:
// motorLeftPWM(0) is not used as the rapid alignment "neutral" anymore.
// With IN/IN wiring it corresponds to HIGH/HIGH and behaves like braking,
// which is exactly the kind of repeated braking that caused the stutter.
//
// LOW/LOW is used as a true motor release/coast command during alignment.
void motorRelease() {
  analogWrite(MOTOR_IN1, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
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
  if (now - previousSpeedTime < 30) {
    return;
  }

  unsigned long dt = now - previousSpeedTime;
  long delta = encoderCount - previousSpeedCount;

  wheelRPM =
    ((float)delta / (float)COUNTS_PER_REV) *
    (60000.0f / (float)dt);

  previousSpeedCount = encoderCount;
  previousSpeedTime = now;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
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
    Zone oldZone = zone;

    previousZone = zone;
    zone = newZone;
    zoneSince = now;

    // Remember only the side we crossed from so CENTRE_ONLY can use a tiny
    // braking probe. This memory NEVER changes what LEFT or RIGHT mean.
    if (oldZone == LEFT_ONLY || oldZone == LEFT_CENTRE) {
      approachSide = APPROACH_FROM_LEFT;
      approachDirection = BODY_CCW;
    } else if (oldZone == RIGHT_ONLY || oldZone == CENTRE_RIGHT) {
      approachSide = APPROACH_FROM_RIGHT;
      approachDirection = BODY_CW;
    }

    bool newHasCentre =
      newZone == LEFT_CENTRE ||
      newZone == CENTRE_ONLY ||
      newZone == ALL_THREE ||
      newZone == CENTRE_RIGHT;

    bool oldWasSide =
      oldZone == LEFT_ONLY ||
      oldZone == RIGHT_ONLY;

    if (newHasCentre && oldWasSide && approachDirection != 0) {
      centreCrossingGuard = true;
      centreCrossedAt = now;

      // If we enter centre from a side, the optional one-count centre probe
      // is opposite the direction that carried us into the centre cone.
      centreProbeDirection = -approachDirection;
    }
  }

  if (
    zone == LEFT_CENTRE ||
    zone == CENTRE_ONLY ||
    zone == ALL_THREE ||
    zone == CENTRE_RIGHT
  ) {
    centreSeenThisAcquisition = true;
  }

  // Unified funnel rule. Acquisition and fine tracking are identical.
  if (zone == LEFT_ONLY || zone == LEFT_CENTRE) {
    lastTargetDirection = BODY_CCW;
  } else if (zone == RIGHT_ONLY || zone == CENTRE_RIGHT) {
    lastTargetDirection = BODY_CW;
  }

  if (
    centreCrossingGuard &&
    now - centreCrossedAt > CENTRE_CROSSING_MEMORY_MS
  ) {
    centreCrossingGuard = false;
    approachSide = APPROACH_NONE;
    approachDirection = 0;
  }
}

bool targetVisible() {
  return zone != NONE;
}

bool perfectlyAligned() {
  return zone == ALL_THREE;
}

// Direction observed on the real suspended Sentinel:
// before CENTRE has been acquired, side-only acquisition needs the opposite
// direction. Once CENTRE has been seen, the original V6 directions are correct.
int leftCorrectionDirection(unsigned long now) {
  (void)now;
  // LEFT always means the target is left of the laser centreline.
  // Apply the same CCW micro correction whether this is the first sensor
  // detected or we are already fine-tracking around centre.
  return BODY_CCW;
}

int rightCorrectionDirection(unsigned long now) {
  (void)now;
  // RIGHT always means the target is right of the laser centreline.
  // Apply the same CW micro correction whether this is acquisition or
  // fine tracking.
  return BODY_CW;
}

bool currentZoneDemandsOppositeDirection(int currentDirection) {
  if (zone == LEFT_ONLY || zone == LEFT_CENTRE) {
    return currentDirection != BODY_CCW;
  }

  if (zone == RIGHT_ONLY || zone == CENTRE_RIGHT) {
    return currentDirection != BODY_CW;
  }

  return false;
}

// ============================================================
// CONTINUOUS SEARCH
// ============================================================

void beginContinuousSearch(unsigned long now) {
  searchBaselineRPM = wheelRPM;
  searchStartEncoder = encoderCount;
  searchWheelSign = 0;
  searchStarted = now;
  searchRunning = true;
}

int getContinuousSearchPower(unsigned long now) {
  float deltaRPM = absFloat(wheelRPM - searchBaselineRPM);

  int targetPower;

  if (deltaRPM < SEARCH_RPM_MID) {
    targetPower = SEARCH_POWER_START;
  }

  else if (deltaRPM < SEARCH_RPM_LOW) {
    targetPower = SEARCH_POWER_MID;
  }

  else {
    targetPower = SEARCH_POWER_LOW;
  }

  // Smooth startup from a gentle torque to the requested search power.
  unsigned long elapsed = now - searchStarted;

  if (elapsed < SEARCH_RAMP_MS) {
    long ramped = map(
      (long)elapsed,
      0L,
      (long)SEARCH_RAMP_MS,
      (long)SEARCH_RAMP_START_POWER,
      (long)targetPower
    );

    targetPower = constrain(
      (int)ramped,
      SEARCH_RAMP_START_POWER,
      SEARCH_POWER_START
    );
  }

  return targetPower;
}

void serviceContinuousSearch(unsigned long now) {
  if (!searchRunning) {
    beginContinuousSearch(now);
  }

  long displacement = encoderCount - searchStartEncoder;

  if (displacement > 0) {
    searchWheelSign = +1;
  }

  else if (displacement < 0) {
    searchWheelSign = -1;
  }

  driveBodyTorque(
    searchDirection,
    getContinuousSearchPower(now)
  );
}

// ============================================================
// SEARCH MOMENTUM CAPTURE
// ============================================================

void beginSearchCapture(unsigned long now, bool targetWasDetected) {
  searchRunning = false;
  captureForTarget = targetWasDetected;
  searchCaptureStarted = now;

  // Do NOT kick the wheel in the opposite direction here.
  // The sensors will choose the next correction after this short settle.
  motorNeutral();
}

bool serviceSearchCapture(unsigned long now) {
  motorRelease();

  return (
    now - searchCaptureStarted >=
    SEARCH_TO_ALIGN_SETTLE_MS
  );
}

// ============================================================
// SMOOTH PWM RAMP
// ============================================================

int rampPower(
  int startPower,
  int targetPower,
  unsigned long elapsed,
  unsigned long rampMs
) {
  if (rampMs == 0 || elapsed >= rampMs) {
    return targetPower;
  }

  long value = map(
    (long)elapsed,
    0L,
    (long)rampMs,
    (long)startPower,
    (long)targetPower
  );

  return constrain(
    (int)value,
    min(startPower, targetPower),
    max(startPower, targetPower)
  );
}

// ============================================================
// MICRO MOVEMENT ENGINE
// ============================================================

bool microBusy() {
  return microPhase != MICRO_IDLE;
}

void startMicroMove(
  int direction,
  long counts,
  int power,
  unsigned long settleMs,
  unsigned long now
) {
  if (microBusy()) {
    return;
  }

  microBodyDirection = direction;
  microTargetCounts = counts;
  microPower = power;
  microSettleMs = settleMs;

  microStartEncoder = encoderCount;
  microWheelSign = 0;

  microBaselineRPM = wheelRPM;

  microStartZone = zone;

  microStableSince = 0;
  microPhaseSince = now;
  microPhase = MICRO_DRIVE;

  // Start with a gentle torque instead of an instantaneous full-power kick.
  microAppliedPower = min(
    microPower,
    MICRO_DRIVE_START_POWER
  );

  driveBodyTorque(
    microBodyDirection,
    microAppliedPower
  );
}

void beginMicroRecovery(unsigned long now) {
  microPhase = MICRO_RECOVER;
  microPhaseSince = now;
  microStableSince = 0;

  // Recovery is deliberately soft. A hard opposite command creates a
  // second chassis kick and was a major source of the jumpy behaviour.
  microAppliedPower = MICRO_RECOVER_START_POWER;

  driveBodyTorque(
    -microBodyDirection,
    microAppliedPower
  );
}

void beginMicroSettle(unsigned long now) {
  motorNeutral();

  microPhase = MICRO_SETTLE;
  microPhaseSince = now;
}

void finishMicroMove() {
  motorNeutral();
  microAppliedPower = 0;
  microPhase = MICRO_IDLE;
}

void serviceMicroMove(unsigned long now) {
  if (microPhase == MICRO_IDLE) {
    return;
  }

  // ----------------------------------------------------------
  // HARD SENSOR FUNNEL OVERRIDE
  // ----------------------------------------------------------
  // This is the key V7.4 fix.
  //
  // If we were moving because RIGHT was detected (CW correction), but the
  // LEFT sensor becomes active, the target has crossed past the centre.
  // Do NOT finish the old move or its recovery in the old direction.
  //
  // Likewise for LEFT -> RIGHT.
  //
  // Abort immediately, neutral the motor, and let the ALIGN state issue
  // the correction required by the sensor that is active NOW.
  if (
    currentZoneDemandsOppositeDirection(
      microBodyDirection
    )
  ) {
    motorNeutral();

    microAppliedPower = 0;
    microPhase = MICRO_IDLE;

    // Give the new zone a fresh stability timer so the next correction
    // is based on a real sensor state rather than one noisy frame.
    zoneSince = now;

    return;
  }

  // ----------------------------------------------------------
  // Tiny drive
  // ----------------------------------------------------------
  if (microPhase == MICRO_DRIVE) {
    long displacement =
      encoderCount - microStartEncoder;

    long moved = labs(displacement);

    // Smoothly increase torque while this microscopic correction is active.
    // If one encoder count happens quickly, the motor may never even reach
    // the full requested power, which is exactly what we want near centre.
    int requestedPower = rampPower(
      min(microPower, MICRO_DRIVE_START_POWER),
      microPower,
      now - microPhaseSince,
      MICRO_DRIVE_RAMP_MS
    );

    if (requestedPower != microAppliedPower) {
      microAppliedPower = requestedPower;
      driveBodyTorque(
        microBodyDirection,
        microAppliedPower
      );
    }

    if (displacement > 0) {
      microWheelSign = +1;
    }

    else if (displacement < 0) {
      microWheelSign = -1;
    }

    // Most important precision feature:
    // if the sensor geometry changes before the requested encoder count
    // completes, stop pushing immediately.
    if (perfectlyAligned()) {
      beginMicroRecovery(now);
      return;
    }

    if (
      targetVisible() &&
      zone != microStartZone
    ) {
      // If the new zone is on the opposite side, drop the previous move
      // completely. The new side sensor should immediately determine the
      // next direction.
      if (
        currentZoneDemandsOppositeDirection(
          microBodyDirection
        )
      ) {
        motorNeutral();
        microAppliedPower = 0;
        microPhase = MICRO_IDLE;
        zoneSince = now;
        return;
      }

      // Same-side improvement (e.g. RIGHT -> CENTRE_RIGHT) should stop
      // driving and smoothly recover before issuing the finer correction.
      beginMicroRecovery(now);
      return;
    }

    if (moved >= microTargetCounts) {
      beginMicroRecovery(now);
      return;
    }

    if (
      now - microPhaseSince >=
      MICRO_DRIVE_TIMEOUT_MS
    ) {
      beginMicroRecovery(now);
      return;
    }
  }

  // ----------------------------------------------------------
  // Return wheel close to pre-correction RPM
  // ----------------------------------------------------------
  else if (microPhase == MICRO_RECOVER) {
    float error =
      microBaselineRPM - wheelRPM;

    if (
      absFloat(error) <=
      MICRO_RPM_TOLERANCE
    ) {
      motorNeutral();

      if (microStableSince == 0) {
        microStableSince = now;
      }

      if (
        now - microStableSince >=
        MICRO_BASELINE_STABLE_MS
      ) {
        beginMicroSettle(now);
        return;
      }
    }

    else {
      microStableSince = 0;

      if (microWheelSign == 0) {
        int power = rampPower(
          MICRO_RECOVER_START_POWER,
          MICRO_RECOVER_POWER_FAST,
          now - microPhaseSince,
          MICRO_RECOVER_RAMP_MS
        );

        microAppliedPower = power;

        driveBodyTorque(
          -microBodyDirection,
          power
        );
      }

      else {
        float errorAlongMove =
          error * (float)microWheelSign;

        bool undoOriginalMove =
          errorAlongMove < 0.0f;

        int direction =
          undoOriginalMove
          ? -microBodyDirection
          : microBodyDirection;

        int targetRecoveryPower =
          absFloat(error) >
          MICRO_RECOVER_SLOW_BAND_RPM
          ? MICRO_RECOVER_POWER_FAST
          : MICRO_RECOVER_POWER_SLOW;

        // Ramp the opposite torque as well. This avoids the characteristic
        // correction-kick / recovery-kick pair.
        int power = rampPower(
          MICRO_RECOVER_START_POWER,
          targetRecoveryPower,
          now - microPhaseSince,
          MICRO_RECOVER_RAMP_MS
        );

        microAppliedPower = power;
        driveBodyTorque(direction, power);
      }
    }

    if (
      now - microPhaseSince >=
      MICRO_RECOVER_TIMEOUT_MS
    ) {
      beginMicroSettle(now);
      return;
    }
  }

  // ----------------------------------------------------------
  // Very short settle before next micro correction
  // ----------------------------------------------------------
  else if (microPhase == MICRO_SETTLE) {
    motorNeutral();

    if (
      now - microPhaseSince >=
      microSettleMs
    ) {
      finishMicroMove();
      return;
    }
  }
}

// ============================================================
// CONTINUOUS SENSOR-FUNNEL CONTROL
// ============================================================

int trackingPowerForZone(Zone z) {
  if (z == LEFT_ONLY || z == RIGHT_ONLY) {
    return ALIGN_OUTER_POWER;
  }

  if (z == LEFT_CENTRE || z == CENTRE_RIGHT) {
    return ALIGN_INNER_POWER;
  }

  return 0;
}

int trackingDirectionForZone(Zone z) {
  if (z == LEFT_ONLY || z == LEFT_CENTRE) {
    return BODY_CCW;
  }

  if (z == RIGHT_ONLY || z == CENTRE_RIGHT) {
    return BODY_CW;
  }

  return 0;
}

void resetContinuousAlignment() {
  motorRelease();

  alignDriveMode = ALIGN_DRIVE_IDLE;
  alignCommandDirection = 0;
  alignRequestedDirection = 0;
  alignRequestedPower = 0;
  centreStableSince = 0;
}

void beginDirectionReversal(
  int newDirection,
  int finalPower,
  unsigned long now
) {
  reversalStartRPM = wheelRPM;

  alignRequestedDirection = newDirection;
  alignRequestedPower = finalPower;
  alignCommandDirection = newDirection;

  alignDriveMode = ALIGN_DRIVE_REVERSING;
  alignModeSince = now;

  // Do not insert a brake pulse. Command the new direction continuously.
  driveBodyTorqueBalanced(
    newDirection,
    ALIGN_REVERSAL_POWER
  );
}

void beginCentreArrest(unsigned long now) {
  if (lastSteeringDirection == 0) {
    motorRelease();
    alignDriveMode = ALIGN_DRIVE_IDLE;
    return;
  }

  alignDriveMode = ALIGN_DRIVE_CENTRE_ARREST;
  alignModeSince = now;
  centreStableSince = 0;

  alignCommandDirection = -lastSteeringDirection;

  // Oppose the direction that carried the chassis into centre.
  driveBodyTorqueBalanced(
    alignCommandDirection,
    CENTRE_ARREST_POWER
  );
}

void serviceContinuousAlignment(unsigned long now) {
  // ----------------------------------------------------------
  // CENTRE / ALL THREE: actively arrest overshoot.
  // ----------------------------------------------------------
  if (zone == CENTRE_ONLY || zone == ALL_THREE) {
    if (alignDriveMode != ALIGN_DRIVE_CENTRE_ARREST) {
      beginCentreArrest(now);
    }

    if (alignDriveMode == ALIGN_DRIVE_CENTRE_ARREST) {
      if (
        absFloat(wheelRPM) <= CENTRE_STOP_RPM ||
        now - alignModeSince >= CENTRE_ARREST_TIMEOUT_MS
      ) {
        motorRelease();

        if (centreStableSince == 0) {
          centreStableSince = now;
        }

        if (now - centreStableSince >= CENTRE_STABLE_MS) {
          alignDriveMode = ALIGN_DRIVE_IDLE;
          alignCommandDirection = 0;
        }
      } else {
        centreStableSince = 0;

        driveBodyTorqueBalanced(
          -lastSteeringDirection,
          CENTRE_ARREST_POWER
        );
      }
    }

    return;
  }

  // ----------------------------------------------------------
  // LEFT / RIGHT tracking.
  // ----------------------------------------------------------
  int desiredDirection = trackingDirectionForZone(zone);
  int desiredPower = trackingPowerForZone(zone);

  if (desiredDirection == 0 || desiredPower == 0) {
    motorRelease();
    return;
  }

  // If the target crossed from one side to the other, the new side must win.
  if (
    lastSteeringDirection != 0 &&
    desiredDirection != lastSteeringDirection &&
    alignDriveMode != ALIGN_DRIVE_REVERSING
  ) {
    beginDirectionReversal(
      desiredDirection,
      desiredPower,
      now
    );

    lastSteeringDirection = desiredDirection;
    return;
  }

  // Continue a real reversal until old flywheel motion is gone.
  if (alignDriveMode == ALIGN_DRIVE_REVERSING) {
    bool crossedZero = false;

    if (absFloat(reversalStartRPM) > REVERSAL_ZERO_RPM) {
      crossedZero =
        (reversalStartRPM > 0.0f && wheelRPM <= 0.0f) ||
        (reversalStartRPM < 0.0f && wheelRPM >= 0.0f);
    }

    if (
      absFloat(wheelRPM) <= REVERSAL_ZERO_RPM ||
      crossedZero ||
      now - alignModeSince >= REVERSAL_TIMEOUT_MS
    ) {
      alignDriveMode = ALIGN_DRIVE_TRACK;
      alignCommandDirection = alignRequestedDirection;

      driveBodyTorqueBalanced(
        alignRequestedDirection,
        alignRequestedPower
      );
    } else {
      driveBodyTorqueBalanced(
        alignRequestedDirection,
        ALIGN_REVERSAL_POWER
      );
    }

    return;
  }

  // Normal continuous steering.
  alignDriveMode = ALIGN_DRIVE_TRACK;
  alignCommandDirection = desiredDirection;
  alignRequestedDirection = desiredDirection;
  alignRequestedPower = desiredPower;
  lastSteeringDirection = desiredDirection;

  driveBodyTorqueBalanced(
    desiredDirection,
    desiredPower
  );
}

// ============================================================
// FINE ALIGNMENT DECISION
// ============================================================
// ============================================================
// FINE ALIGNMENT DECISION
// ============================================================

void startAlignmentCorrection(unsigned long now) {
  if (zone == LEFT_ONLY) {
    startMicroMove(
      leftCorrectionDirection(now),
      OUTER_MICRO_COUNTS,
      OUTER_MICRO_POWER,
      OUTER_MICRO_SETTLE_MS,
      now
    );

    return;
  }

  if (zone == LEFT_CENTRE) {
    startMicroMove(
      leftCorrectionDirection(now),
      INNER_MICRO_COUNTS,
      INNER_MICRO_POWER,
      INNER_MICRO_SETTLE_MS,
      now
    );

    return;
  }

  if (zone == RIGHT_ONLY) {
    startMicroMove(
      rightCorrectionDirection(now),
      OUTER_MICRO_COUNTS,
      OUTER_MICRO_POWER,
      OUTER_MICRO_SETTLE_MS,
      now
    );

    return;
  }

  if (zone == CENTRE_RIGHT) {
    startMicroMove(
      rightCorrectionDirection(now),
      INNER_MICRO_COUNTS,
      INNER_MICRO_POWER,
      INNER_MICRO_SETTLE_MS,
      now
    );

    return;
  }

  if (zone == CENTRE_ONLY) {
    // If we JUST arrived at centre from a side, wait briefly.
    // The chassis is still carrying angular velocity/string energy.
    // Immediately commanding another probe is exactly what caused it to
    // continue through centre toward the opposite sensor.
    if (
      centreCrossingGuard &&
      now - centreCrossedAt < CENTRE_CAPTURE_DWELL_MS
    ) {
      motorNeutral();
      return;
    }

    centreProbeAttempts++;

    // After a side -> centre transition, centreProbeDirection was already
    // set to the OPPOSITE of the approach direction. This gently arrests
    // the crossing rather than continuing the overshoot.
    //
    // If centre was the first sensor detected, use the normal tiny
    // exploratory behavior and reverse after two unsuccessful probes.
    if (
      !centreCrossingGuard &&
      centreProbeAttempts > 2
    ) {
      centreProbeDirection =
        -centreProbeDirection;

      centreProbeAttempts = 1;
    }

    startMicroMove(
      centreProbeDirection,
      PINHOLE_MICRO_COUNTS,
      PINHOLE_MICRO_POWER,
      PINHOLE_MICRO_SETTLE_MS,
      now
    );

    return;
  }

  if (zone == LEFT_RIGHT) {
    // Rare/ambiguous pattern. Use only a one-count correction.
    int direction =
      lastTargetDirection != 0
      ? lastTargetDirection
      : centreProbeDirection;

    startMicroMove(
      direction,
      PINHOLE_MICRO_COUNTS,
      PINHOLE_MICRO_POWER,
      PINHOLE_MICRO_SETTLE_MS,
      now
    );
  }
}

// ============================================================
// EDGE REACQUISITION
// ============================================================

void recordEdgeLoss(unsigned long now) {
  if (
    now - lastEdgeLossTime >
    EDGE_LOSS_WINDOW_MS
  ) {
    repeatedEdgeLosses = 0;
  }

  repeatedEdgeLosses++;
  lastEdgeLossTime = now;
}

void serviceLocalReacquire(unsigned long now) {
  if (microBusy()) {
    return;
  }

  if (
    now - lastEdgeMicro <
    EDGE_MICRO_PERIOD_MS
  ) {
    return;
  }

  int direction =
    lastTargetDirection != 0
    ? lastTargetDirection
    : centreProbeDirection;

  // If it repeatedly springs out of the same edge, reverse one tiny probe
  // so we do not continuously feed the oscillation.
  if (repeatedEdgeLosses >= 2) {
    direction = -direction;
  }

  startMicroMove(
    direction,
    PINHOLE_MICRO_COUNTS,
    PINHOLE_MICRO_POWER,
    PINHOLE_MICRO_SETTLE_MS,
    now
  );

  lastEdgeMicro = now;
}

// ============================================================
// STRING-TORQUE HOLD LEARNING
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

void learnHoldFromDrift(
  int requiredCorrectionDirection,
  bool largeDrift
) {
  if (requiredCorrectionDirection == 0) {
    return;
  }

  if (holdDirection != requiredCorrectionDirection) {
    holdDirection = requiredCorrectionDirection;
    holdPower = HOLD_START_POWER;
  }

  else {
    holdPower += largeDrift ? 3 : 1;

    holdPower = constrain(
      holdPower,
      HOLD_START_POWER,
      HOLD_MAX_POWER
    );
  }
}

void serviceHold(unsigned long now) {
  if (
    holdDirection == 0 ||
    holdPower <= 0
  ) {
    return;
  }

  if (absFloat(wheelRPM) >= HOLD_RPM_LIMIT) {
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

  if (
    now - lastHoldPulse >=
    HOLD_PERIOD_MS
  ) {
    driveBodyTorque(
      holdDirection,
      holdPower
    );

    holdPulseActive = true;
    holdPulseEnd = now + HOLD_PULSE_MS;
    lastHoldPulse = now;
  }
}

// ============================================================
// STATE / LED
// ============================================================

void changeState(
  MainState nextState,
  unsigned long now
) {
  if (state == nextState) {
    return;
  }

  state = nextState;
  stateSince = now;
}

void updateStatusLED(unsigned long now) {
  // Solid means alignment confirmation / firing.
  if (state == CONFIRM || state == FIRE) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }

  if (state == STARTUP) {
    digitalWrite(STATUS_LED, LOW);
    return;
  }

  unsigned long interval = 450;

  if (state == ALIGN) {
    interval = 90;
  }

  else if (state == CAPTURE_SEARCH) {
    interval = 180;
  }

  if (now - lastLedToggle >= interval) {
    lastLedToggle = now;
    ledState = !ledState;

    digitalWrite(
      STATUS_LED,
      ledState ? HIGH : LOW
    );
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

  // Legacy pulse/micro engine disabled: V7.7 alignment is continuous.
  updateStatusLED(now);

  // ==========================================================
  // STARTUP
  // ==========================================================

  if (state == STARTUP) {
    digitalWrite(LASER_PIN, LOW);
    resetContinuousAlignment();
    motorRelease();

    if (now - stateSince >= STARTUP_MS) {
      centreSeenThisAcquisition = false;
      centreCrossingGuard = false;
      approachSide = APPROACH_NONE;
      approachDirection = 0;
      beginContinuousSearch(now);
      changeState(SEARCH, now);
    }
  }

  // ==========================================================
  // SLOW CONTINUOUS SEARCH
  // ==========================================================

  else if (state == SEARCH) {
    digitalWrite(LASER_PIN, LOW);
    resetContinuousAlignment();

    if (targetVisible()) {
      // IR found: remove continuous search drive without a large reversal.
      beginSearchCapture(now, true);
      changeState(CAPTURE_SEARCH, now);
    }

    else {
      // Continuous slow seek. No automatic 5-second direction reversal.
      serviceContinuousSearch(now);
    }
  }

  // ==========================================================
  // REMOVE SEARCH MOMENTUM
  // ==========================================================

  else if (state == CAPTURE_SEARCH) {
    digitalWrite(LASER_PIN, LOW);
    resetContinuousAlignment();

    // If the sensor pattern is still changing while search momentum/string
    // motion settles, restart the quiet timer. Alignment only begins after
    // the current detected zone has remained stable for the full settle.
    static Zone captureStableZone = NONE;

    if (zone != captureStableZone) {
      captureStableZone = zone;
      searchCaptureStarted = now;
      motorNeutral();
    }

    if (serviceSearchCapture(now)) {
      centreProbeAttempts = 0;

      if (targetVisible()) {
        repeatedEdgeLosses = 0;
      } else {
        recordEdgeLoss(now);
      }

      captureStableZone = NONE;
      changeState(ALIGN, now);
    }
  }

  // ==========================================================
  // MICRO ALIGN
  // ==========================================================

  else if (state == ALIGN) {
    digitalWrite(LASER_PIN, LOW);

    if (
      perfectlyAligned() &&
      now - zoneSince >= ZONE_DECISION_STABLE_MS
    ) {
      serviceContinuousAlignment(now);

      if (
        alignDriveMode == ALIGN_DRIVE_IDLE ||
        absFloat(wheelRPM) <= CENTRE_STOP_RPM
      ) {
        motorRelease();

        alignedSince = now;

        centreProbeAttempts = 0;
        repeatedEdgeLosses = 0;

        centreCrossingGuard = false;
        approachSide = APPROACH_NONE;
        approachDirection = 0;

        changeState(CONFIRM, now);
      }
    }

    else if (
      targetVisible() &&
      now - zoneSince >= ZONE_DECISION_STABLE_MS
    ) {
      // Fully continuous sensor funnel.
      // LEFT -> CCW, RIGHT -> CW.
      // If the opposite side appears, the controller remains in the new
      // direction until encoder RPM confirms the old flywheel motion is gone.
      serviceContinuousAlignment(now);
    }

    else {
      // Brief target loss: release the wheel instead of injecting pulses.
      motorRelease();

      unsigned long lostFor =
        targetLastSeen == 0
        ? TARGET_LOST_MS
        : now - targetLastSeen;

      if (lostFor >= TARGET_LOST_MS) {
        clearHold();

        centreProbeAttempts = 0;
        repeatedEdgeLosses = 0;

        if (lastTargetDirection != 0) {
          searchDirection = lastTargetDirection;
        }

        lastTargetDirection = 0;
        centreSeenThisAcquisition = false;
        centreCrossingGuard = false;
        approachSide = APPROACH_NONE;
        approachDirection = 0;

        resetContinuousAlignment();
        lastSteeringDirection = 0;

        beginContinuousSearch(now);
        changeState(SEARCH, now);
      }
    }
  }

  // ==========================================================
  // CONFIRM ALL THREE
  // ==========================================================

  else if (state == CONFIRM) {
    digitalWrite(LASER_PIN, LOW);

    if (
      perfectlyAligned()
    ) {
      motorRelease();

      if (
        now - alignedSince >=
        ALIGN_CONFIRM_MS &&
        true
      ) {
        fireSince = now;

        digitalWrite(LASER_PIN, HIGH);

        changeState(FIRE, now);
      }
    }

    else {
      digitalWrite(LASER_PIN, LOW);
      stopHoldPulse();
      resetContinuousAlignment();

      alignedSince = 0;
      recordEdgeLoss(now);

      if (
        zone == LEFT_ONLY ||
        zone == LEFT_CENTRE
      ) {
        learnHoldFromDrift(
          leftCorrectionDirection(now),
          zone == LEFT_ONLY
        );
      }

      else if (
        zone == RIGHT_ONLY ||
        zone == CENTRE_RIGHT
      ) {
        learnHoldFromDrift(
          rightCorrectionDirection(now),
          zone == RIGHT_ONLY
        );
      }

      changeState(ALIGN, now);
    }
  }

  // ==========================================================
  // FIRE
  // ==========================================================

  else if (state == FIRE) {
    if (
      perfectlyAligned()
    ) {
      digitalWrite(LASER_PIN, HIGH);

      motorRelease();

      if (
        now - fireSince >=
        FIRE_TIME_MS
      ) {
        digitalWrite(LASER_PIN, LOW);

        stopHoldPulse();
        clearSince = 0;

        changeState(WAIT_CLEAR, now);
      }
    }

    else {
      // ANY loss of the all-three condition immediately switches laser off.
      digitalWrite(LASER_PIN, LOW);

      stopHoldPulse();
      resetContinuousAlignment();

      fireSince = 0;
      alignedSince = 0;

      recordEdgeLoss(now);

      if (
        zone == LEFT_ONLY ||
        zone == LEFT_CENTRE
      ) {
        learnHoldFromDrift(
          leftCorrectionDirection(now),
          zone == LEFT_ONLY
        );
      }

      else if (
        zone == RIGHT_ONLY ||
        zone == CENTRE_RIGHT
      ) {
        learnHoldFromDrift(
          rightCorrectionDirection(now),
          zone == RIGHT_ONLY
        );
      }

      changeState(ALIGN, now);
    }
  }

  // ==========================================================
  // WAIT FOR TARGET TO CLEAR
  // ==========================================================

  else if (state == WAIT_CLEAR) {
    digitalWrite(LASER_PIN, LOW);

    stopHoldPulse();
    resetContinuousAlignment();
    motorRelease();

    if (zone == NONE) {
      if (clearSince == 0) {
        clearSince = now;
      }

      if (
        now - clearSince >=
        CLEAR_CONFIRM_MS
      ) {
        clearHold();

        // Alternate direction after each completed target.
        searchDirection = -searchDirection;

        lastTargetDirection = 0;
        centreProbeAttempts = 0;
        repeatedEdgeLosses = 0;
        centreSeenThisAcquisition = false;
        centreCrossingGuard = false;
        approachSide = APPROACH_NONE;
        approachDirection = 0;

        beginContinuousSearch(now);
        changeState(SEARCH, now);
      }
    }

    else {
      clearSince = 0;
    }
  }

  // ==========================================================
  // OPTIONAL SERIAL TELEMETRY
  // ==========================================================

  static unsigned long lastPrint = 0;

  if (now - lastPrint >= 200) {
    lastPrint = now;

    Serial.print("STATE:");
    Serial.print((int)state);

    Serial.print(" ZONE:");
    Serial.print((int)zone);

    Serial.print(" RPM:");
    Serial.print(wheelRPM, 1);

    Serial.print(" MICRO:");
    Serial.print((int)microPhase);

    Serial.print(" ENC:");
    Serial.print(encoderCount);

    Serial.print(" LASER:");
    Serial.println(
      digitalRead(LASER_PIN)
      ? "ON"
      : "OFF"
    );
  }
}

// ============================================================
// MULTIPLE IR RECEIVER TIMER HANDLER
// ============================================================

void UserIRReceiveTimerInterruptHandler() {
  centreReceiver.ReceiveInterruptHandler();
  rightReceiver.ReceiveInterruptHandler();
}
