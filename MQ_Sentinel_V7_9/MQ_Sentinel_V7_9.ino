// ============================================================
// MQ SENTINEL - CONTINUOUS REVERSING ALIGN V7.9
//
// Based on V7.7b (and the working V6 motor/sensor layout).
//
// V7.9 CHANGES FROM V7.8
//   A. SEARCH THAT ACTUALLY TURNS THE SATELLITE ("push and ease").
//      V7.7b drove the wheel at a constant power. The chassis only feels
//      a torque while the wheel is SPEEDING UP, so it got one short shove
//      (~0.5 s) and then nothing - friction held it and it did not rotate.
//      V7.9 repeats: PUSH (power keeps rising -> wheel keeps accelerating
//      -> chassis keeps turning), EASE (power falls slowly, too gently to
//      turn the chassis back), SETTLE (wheel stops). Always the same
//      direction, so each cycle turns the satellite further.
//      Any IR seen -> alignment immediately, handing over smoothly from
//      the search power (no release/brake kick).
//
// V7.8 CHANGES FROM V7.7b (kept)
//   B. APPROACH PROFILE: quick at first, progressively finer near centre.
//        LEFT_ONLY / RIGHT_ONLY   start strong (OUTER_START_POWER), then
//                                 taper to OUTER_END_POWER
//        LEFT+C / C+RIGHT         drop smoothly to INNER_START_POWER, then
//                                 taper to the very fine INNER_END_POWER
//        CENTRE / ALL THREE       arrest whatever wheel motion is left,
//                                 with force scaled to how fast it is
//      If the chassis stalls in a zone (friction/string), power creeps up
//      slowly so it never gets stuck. Power changes are slew-limited so
//      there are no steps or kicks.
//   C. Centre arrest fixes: it no longer keeps driving after the wheel has
//      passed through zero, and it no longer re-fires every ~70 ms while
//      sitting at centre (both caused small wrong-way kicks).
//   D. During the 22 ms zone-change debounce the motor keeps its current
//      command instead of briefly releasing.
//   Motor functions, encoder polling, IR setup, LEFT/RIGHT reversal logic,
//   ALL-THREE rule, 2.1 s dwell and wait-for-clear are unchanged.
//
// CONTROL STRATEGY
// 1. No IR: push-and-ease search in one direction.
// 2. LEFT sensor: CCW correction. RIGHT sensor: CW correction.
// 3. Far from centre (one side sensor only): quick correction.
// 4. Close to centre (side + centre): gentle, tapering to very fine.
// 5. Crossing to the other side: reverse immediately (V7.7 reversal).
// 6. ALL THREE: stop correcting, confirm alignment, then fire laser.
// 7. After a successful shot, wait for the target to disappear, then
//    search again in the opposite direction.
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

// Numbering kept the same as V7.7b so serial STATE numbers mean the same:
//   0 STARTUP  1 SEARCH  2 (unused)  3 ALIGN
//   4 CONFIRM  5 FIRE  6 WAIT_CLEAR
enum MainState {
  STARTUP,
  SEARCH,
  CAPTURE_SEARCH,   // unused (search hands over to ALIGN directly)
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
// SEARCH TUNING - V7.9 "PUSH AND EASE"
// ============================================================
//
//  power
//   ^        PUSH            EASE                 SETTLE
//  END |        /|\
//      |       / |  \___
//      |      /  |      \_____
// START|     |   |            \_______
//      |     |   |                    \___ (released)   repeat
//    0 +-----+---+------------------------+--------------+------->
//              PUSH_MS        EASE_MS          until wheel stopped
//
// PUSH:   power steps to SEARCH_PUSH_START_POWER (breaks swivel friction,
//         same idea as the alignment step that is known to move the
//         chassis) and keeps rising to SEARCH_PUSH_END_POWER. The wheel keeps
//         accelerating, so the chassis keeps turning for the whole push.
// EASE:   power falls slowly to zero. The wheel slows gently, so the
//         reverse torque is too small to turn the chassis back.
// SETTLE: motor released until the wheel has (almost) stopped.
//
// TUNING
//   Satellite does not move during PUSH  -> raise SEARCH_PUSH_START_POWER
//                                           (and/or SEARCH_PUSH_END_POWER)
//   Satellite turns back during EASE     -> make SEARCH_EASE_MS longer
//   Turns too far / too fast per push    -> lower SEARCH_PUSH_END_POWER
//   Search too slow overall              -> shorten SEARCH_EASE_MS a little

int searchDirection = BODY_CCW;

const int SEARCH_PUSH_START_POWER = 75;
const int SEARCH_PUSH_END_POWER   = 170;
const unsigned long SEARCH_PUSH_MS = 1200;

const unsigned long SEARCH_EASE_MS = 5000;
const int SEARCH_EASE_RELEASE_POWER = 35;   // below this, just release

const float SEARCH_SETTLE_RPM = 10.0f;
const unsigned long SEARCH_SETTLE_MAX_MS = 1500;

enum SearchPhase {
  SEARCH_SETTLE,
  SEARCH_PUSH,
  SEARCH_EASE
};

SearchPhase searchPhase = SEARCH_SETTLE;
unsigned long searchPhaseSince = 0;
int searchPower = 0;    // power currently applied in searchDirection

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

// Crossing to the other side (overshoot): reverse at once. Force scales
// with how fast the wheel is still turning the old way (max = the V7.7b
// value), and hands over to the gentle approach profile slightly before
// the wheel reaches zero so the reversal does not overshoot.
const int ALIGN_REVERSAL_POWER = 105;
const int REVERSAL_MIN_POWER = 60;
const float REVERSAL_PWM_PER_RPM = 0.8f;
const float REVERSAL_HANDOVER_RPM = 20.0f;

// ============================================================
// APPROACH PROFILE - V7.8  (quick at first, finer near centre)
// ============================================================
//
// Power for a side zone = time-based taper from START to END power,
// counted from when the target entered that zone:
//
//   power
//    ^
//  START |\
//        | \_____________  END             ___/  creep (only if stuck)
//        |               \_______________/
//        +-----------------------------------------> time in zone
//           TAPER_MS        STALL_CREEP_DELAY_MS
//
// OUTER (LEFT_ONLY / RIGHT_ONLY): far from centre.
//   OUTER_START_POWER is used only on the FIRST approach after the beacon
//   appears. Drifting back out from the inner zone uses OUTER_END_POWER.
const int OUTER_START_POWER = 96;
const int OUTER_END_POWER   = 72;
const unsigned long OUTER_TAPER_MS = 700;

// INNER (LEFT_CENTRE / CENTRE_RIGHT): close to centre.
const int INNER_START_POWER = 56;
const int INNER_END_POWER   = 40;     // the "very fine" level
const unsigned long INNER_TAPER_MS = 800;

// If the target is still in the same zone this long after the taper ends,
// friction/string has stopped the chassis: raise power slowly (never above
// that zone's START power) until the zone changes.
const unsigned long STALL_CREEP_DELAY_MS = 400;
const float STALL_CREEP_PWM_PER_S = 20.0f;

// Smoothness: maximum rate the applied power may change (PWM per second).
// Going down gradually is what gently brakes the approach.
const float POWER_SLEW_UP_PER_S   = 300.0f;
const float POWER_SLEW_DOWN_PER_S = 200.0f;

// Starting from a released motor, begin here instead of 0 so the first
// movement is immediate (below this the gearmotor does not turn).
const int START_FLOOR_POWER = 40;

// Centre arrest: force scales with remaining wheel speed, capped at
// CENTRE_ARREST_POWER (the V7.7b value). Slow approach -> gentle arrest.
const int CENTRE_ARREST_POWER = 72;
const int CENTRE_ARREST_MIN_POWER = 45;
const float CENTRE_ARREST_PWM_PER_RPM = 0.6f;

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

// V7.8 approach-profile state
enum ApproachBand {
  BAND_NONE,
  BAND_OUTER,
  BAND_INNER
};

ApproachBand currentBand = BAND_NONE;
int currentBandSide = 0;          // BODY_CW / BODY_CCW, 0 = fresh acquisition
unsigned long bandSince = 0;
int bandStartPower = 0;

float alignAppliedPower = 0.0f;   // slew-limited power actually applied
unsigned long lastPowerUpdate = 0;

float arrestStartRPM = 0.0f;

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
// ZONE_DECISION_STABLE_MS is defined once in the V7.7 tuning block above (22 ms).

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
// PUSH-AND-EASE SEARCH (V7.9)
// ============================================================

// Start (or restart) a search. Begins with SETTLE so a wheel that is still
// spinning from alignment coasts down before the first push.
void beginSearch(unsigned long now) {
  motorRelease();
  searchPower = 0;
  searchPhase = SEARCH_SETTLE;
  searchPhaseSince = now;
}

void serviceSearch(unsigned long now) {
  unsigned long t = now - searchPhaseSince;

  if (searchPhase == SEARCH_SETTLE) {
    motorRelease();
    searchPower = 0;

    if (absFloat(wheelRPM) <= SEARCH_SETTLE_RPM ||
        t >= SEARCH_SETTLE_MAX_MS) {
      searchPhase = SEARCH_PUSH;
      searchPhaseSince = now;
    }
    return;
  }

  if (searchPhase == SEARCH_PUSH) {
    unsigned long k = (t < SEARCH_PUSH_MS) ? t : SEARCH_PUSH_MS;

    searchPower = (int)map(
      (long)k, 0L, (long)SEARCH_PUSH_MS,
      (long)SEARCH_PUSH_START_POWER, (long)SEARCH_PUSH_END_POWER
    );

    driveBodyTorqueBalanced(searchDirection, searchPower);

    if (t >= SEARCH_PUSH_MS) {
      searchPhase = SEARCH_EASE;
      searchPhaseSince = now;
    }
    return;
  }

  // SEARCH_EASE
  unsigned long k = (t < SEARCH_EASE_MS) ? t : SEARCH_EASE_MS;

  searchPower = (int)map(
    (long)k, 0L, (long)SEARCH_EASE_MS,
    (long)SEARCH_PUSH_END_POWER, 0L
  );

  if (searchPower < SEARCH_EASE_RELEASE_POWER) {
    motorRelease();
    searchPower = 0;
  } else {
    driveBodyTorqueBalanced(searchDirection, searchPower);
  }

  if (t >= SEARCH_EASE_MS) {
    searchPhase = SEARCH_SETTLE;
    searchPhaseSince = now;
  }
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

// ------------------------------------------------------------
// V7.8 APPROACH PROFILE
// ------------------------------------------------------------

ApproachBand bandForZone(Zone z) {
  if (z == LEFT_ONLY || z == RIGHT_ONLY) {
    return BAND_OUTER;
  }

  if (z == LEFT_CENTRE || z == CENTRE_RIGHT) {
    return BAND_INNER;
  }

  return BAND_NONE;
}

// Forget the approach history (beacon newly appeared, or lost).
void resetApproachBand() {
  currentBand = BAND_NONE;
  currentBandSide = 0;
  bandSince = 0;
  bandStartPower = 0;
}

// Called whenever a side zone is active. Restarts the taper only when the
// band (outer/inner) or side actually changes.
void updateApproachBand(ApproachBand band, int side, unsigned long now) {
  if (band == currentBand && side == currentBandSide) {
    return;
  }

  if (band == BAND_OUTER) {
    // Quick start only on the first approach after the beacon appears.
    // Drifting back out (or overshooting across) uses the calmer level.
    bandStartPower =
      (currentBandSide == 0) ? OUTER_START_POWER : OUTER_END_POWER;
  } else {
    bandStartPower = INNER_START_POWER;
  }

  currentBand = band;
  currentBandSide = side;
  bandSince = now;
}

// Target power for the current band right now.
int approachTargetPower(unsigned long now) {
  int startPower;
  int endPower;
  unsigned long taperMs;

  if (currentBand == BAND_OUTER) {
    startPower = bandStartPower;
    endPower = OUTER_END_POWER;
    taperMs = OUTER_TAPER_MS;
  } else if (currentBand == BAND_INNER) {
    startPower = bandStartPower;
    endPower = INNER_END_POWER;
    taperMs = INNER_TAPER_MS;
  } else {
    return 0;
  }

  if (endPower > startPower) {
    endPower = startPower;
  }

  unsigned long t = now - bandSince;

  // 1. Taper from start to end.
  if (t < taperMs) {
    return (int)map((long)t, 0L, (long)taperMs, (long)startPower, (long)endPower);
  }

  // 2. Hold the fine level.
  unsigned long afterTaper = t - taperMs;

  if (afterTaper < STALL_CREEP_DELAY_MS) {
    return endPower;
  }

  // 3. Still in the same zone: creep up slowly so it cannot get stuck.
  float creep =
    STALL_CREEP_PWM_PER_S *
    (float)(afterTaper - STALL_CREEP_DELAY_MS) / 1000.0f;

  int creepCeiling =
    (currentBand == BAND_OUTER) ? OUTER_START_POWER : INNER_START_POWER;

  return constrain(endPower + (int)creep, endPower, creepCeiling);
}

// Move the applied power toward the target at a limited rate.
int slewAppliedPower(int target, unsigned long now) {
  float dt = (float)(now - lastPowerUpdate) / 1000.0f;
  lastPowerUpdate = now;

  if (dt > 0.1f) {
    dt = 0.1f;
  }

  if (alignAppliedPower < (float)START_FLOOR_POWER &&
      target >= START_FLOOR_POWER) {
    // Coming from a released motor: start at the floor, not at 0.
    alignAppliedPower = (float)START_FLOOR_POWER;
  }

  if ((float)target > alignAppliedPower) {
    alignAppliedPower += POWER_SLEW_UP_PER_S * dt;
    if (alignAppliedPower > (float)target) {
      alignAppliedPower = (float)target;
    }
  } else {
    alignAppliedPower -= POWER_SLEW_DOWN_PER_S * dt;
    if (alignAppliedPower < (float)target) {
      alignAppliedPower = (float)target;
    }
  }

  return (int)(alignAppliedPower + 0.5f);
}

int reversalPower(float rpm) {
  float p = (float)REVERSAL_MIN_POWER +
            REVERSAL_PWM_PER_RPM * absFloat(rpm);

  return constrain((int)p, REVERSAL_MIN_POWER, ALIGN_REVERSAL_POWER);
}

int centreArrestPower(float rpm) {
  float p = (float)CENTRE_ARREST_MIN_POWER +
            CENTRE_ARREST_PWM_PER_RPM * absFloat(rpm);

  return constrain((int)p, CENTRE_ARREST_MIN_POWER, CENTRE_ARREST_POWER);
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
  alignAppliedPower = 0.0f;
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
    reversalPower(wheelRPM)
  );
}

void beginCentreArrest(unsigned long now) {
  // Nothing to arrest: no approach direction, or the wheel is already
  // (almost) still. Just release.
  if (lastSteeringDirection == 0 ||
      absFloat(wheelRPM) <= CENTRE_STOP_RPM) {
    motorRelease();
    alignDriveMode = ALIGN_DRIVE_IDLE;
    alignAppliedPower = 0.0f;
    return;
  }

  alignDriveMode = ALIGN_DRIVE_CENTRE_ARREST;
  alignModeSince = now;
  centreStableSince = 0;
  arrestStartRPM = wheelRPM;

  alignCommandDirection = -lastSteeringDirection;

  // Oppose the direction that carried the chassis into centre, with force
  // proportional to how much wheel motion is left.
  driveBodyTorqueBalanced(
    alignCommandDirection,
    centreArrestPower(wheelRPM)
  );
}

void serviceContinuousAlignment(unsigned long now) {
  // ----------------------------------------------------------
  // CENTRE / ALL THREE: actively arrest overshoot.
  // ----------------------------------------------------------
  if (zone == CENTRE_ONLY || zone == ALL_THREE) {
    // Arrest only if we were actually driving toward centre. Once the
    // arrest has finished (IDLE), do not start another one while sitting
    // at centre - V7.7b re-fired it every ~70 ms.
    if (alignDriveMode == ALIGN_DRIVE_TRACK ||
        alignDriveMode == ALIGN_DRIVE_REVERSING) {
      beginCentreArrest(now);
    }

    if (alignDriveMode == ALIGN_DRIVE_CENTRE_ARREST) {
      bool crossedZero =
        (arrestStartRPM > 0.0f && wheelRPM <= 0.0f) ||
        (arrestStartRPM < 0.0f && wheelRPM >= 0.0f);

      if (
        absFloat(wheelRPM) <= CENTRE_STOP_RPM ||
        crossedZero ||
        now - alignModeSince >= CENTRE_ARREST_TIMEOUT_MS
      ) {
        // Wheel motion gone (or reversed): stop pushing immediately.
        motorRelease();
        alignDriveMode = ALIGN_DRIVE_IDLE;
        alignCommandDirection = 0;
        alignAppliedPower = 0.0f;
        centreStableSince = now;
      } else {
        centreStableSince = 0;

        driveBodyTorqueBalanced(
          alignCommandDirection,
          centreArrestPower(wheelRPM)
        );
      }
    } else {
      motorRelease();
      alignAppliedPower = 0.0f;
    }

    return;
  }

  // ----------------------------------------------------------
  // LEFT / RIGHT tracking.
  // ----------------------------------------------------------
  int desiredDirection = trackingDirectionForZone(zone);

  if (desiredDirection == 0) {
    motorRelease();
    alignAppliedPower = 0.0f;
    return;
  }

  // V7.8: power comes from the approach profile (quick far out, finer
  // close in), slew-limited for smoothness.
  updateApproachBand(bandForZone(zone), desiredDirection, now);
  int desiredPower = approachTargetPower(now);

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
      absFloat(wheelRPM) <= REVERSAL_HANDOVER_RPM ||
      crossedZero ||
      now - alignModeSince >= REVERSAL_TIMEOUT_MS
    ) {
      alignDriveMode = ALIGN_DRIVE_TRACK;
      alignCommandDirection = alignRequestedDirection;

      // Old motion is gone: continue at the (gentle) profile level, not at
      // reversal power.
      alignAppliedPower = (float)desiredPower;
      lastPowerUpdate = now;

      driveBodyTorqueBalanced(
        alignRequestedDirection,
        desiredPower
      );
    } else {
      driveBodyTorqueBalanced(
        alignRequestedDirection,
        reversalPower(wheelRPM)
      );
    }

    return;
  }

  // Normal continuous steering (smooth, slew-limited).
  if (alignDriveMode != ALIGN_DRIVE_TRACK) {
    lastPowerUpdate = now;
  }

  alignDriveMode = ALIGN_DRIVE_TRACK;
  alignCommandDirection = desiredDirection;
  alignRequestedDirection = desiredDirection;
  alignRequestedPower = desiredPower;
  lastSteeringDirection = desiredDirection;

  driveBodyTorqueBalanced(
    desiredDirection,
    slewAppliedPower(desiredPower, now)
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
      beginSearch(now);
      changeState(SEARCH, now);
    }
  }

  // ==========================================================
  // SEARCH - PUSH AND EASE (V7.9)
  // ==========================================================
  // No IR: keep searching. As soon as any receiver sees the beacon
  // (stable for the debounce time), hand over to alignment WITHOUT
  // releasing or braking: alignment starts from the current search power
  // and direction, then the approach profile takes over smoothly.

  else if (state == SEARCH) {
    digitalWrite(LASER_PIN, LOW);

    if (
      targetVisible() &&
      now - zoneSince >= ZONE_DECISION_STABLE_MS
    ) {
      resetApproachBand();          // fresh approach: quick start allowed
      centreProbeAttempts = 0;
      repeatedEdgeLosses = 0;

      if (searchPower > 0) {
        // Smooth hand-over: same direction, same power.
        lastSteeringDirection = searchDirection;
        alignCommandDirection = searchDirection;
        alignRequestedDirection = searchDirection;
        alignDriveMode = ALIGN_DRIVE_TRACK;
        alignAppliedPower = (float)searchPower;
      } else {
        lastSteeringDirection = 0;
        alignDriveMode = ALIGN_DRIVE_IDLE;
        alignAppliedPower = 0.0f;
      }

      lastPowerUpdate = now;
      searchPower = 0;
      changeState(ALIGN, now);
    }

    else {
      serviceSearch(now);
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

    else if (targetVisible()) {
      // Zone changed less than ZONE_DECISION_STABLE_MS ago: keep the
      // current motor command (V7.7b briefly released here).
    }

    else {
      // Brief target loss: release the wheel instead of injecting pulses.
      motorRelease();
      alignAppliedPower = 0.0f;
      alignDriveMode = ALIGN_DRIVE_IDLE;

      unsigned long lostFor =
        targetLastSeen == 0
        ? TARGET_LOST_MS
        : now - targetLastSeen;

      if (lostFor >= TARGET_LOST_MS) {
        clearHold();

        centreProbeAttempts = 0;
        repeatedEdgeLosses = 0;

        // Search toward where the target was last seen.
        if (lastTargetDirection != 0) {
          searchDirection = lastTargetDirection;
        }

        lastTargetDirection = 0;
        centreSeenThisAcquisition = false;
        centreCrossingGuard = false;
        approachSide = APPROACH_NONE;
        approachDirection = 0;

        resetContinuousAlignment();
        resetApproachBand();
        lastSteeringDirection = 0;

        beginSearch(now);
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

        lastTargetDirection = 0;
        centreProbeAttempts = 0;
        repeatedEdgeLosses = 0;
        centreSeenThisAcquisition = false;
        centreCrossingGuard = false;
        approachSide = APPROACH_NONE;
        approachDirection = 0;

        resetApproachBand();
        lastSteeringDirection = 0;

        // Alternate direction after each completed target.
        searchDirection = -searchDirection;

        beginSearch(now);
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

    Serial.print(" PWR:");
    Serial.print(
      state == SEARCH
      ? searchPower
      : (int)(alignAppliedPower + 0.5f)
    );

    if (state == SEARCH) {
      Serial.print(" SRCH:");
      Serial.print(
        searchPhase == SEARCH_PUSH ? "PUSH" :
        searchPhase == SEARCH_EASE ? "EASE" : "SETTLE"
      );
    }

    Serial.print(" BAND:");
    Serial.print(
      currentBand == BAND_OUTER ? "OUT" :
      currentBand == BAND_INNER ? "IN" : "-"
    );

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
