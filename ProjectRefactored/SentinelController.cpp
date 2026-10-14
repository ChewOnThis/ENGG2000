// ============================================================
#include "SentinelController.h"
#include "MotorController.h"
#include "Encoder.h"
#include "Mpu6050Gyro.h"
#include "IrSensors.h"

// The original V8 control/state-machine implementation below remains in one
// translation unit so interrupt timing and ordering are unchanged.
MotorController motorController(6, 7, false);
Encoder encoder(9, 10, 700);
Mpu6050Gyro gyro(0x68, 2, 1.0f, 131.0f);
IrSensors irSensors(2, 3, 4, 250);
// MQ SENTINEL - MOMENTUM FUNNEL V8.0
//
// Modified from FLUID MICRO ALIGN V7.6.
//
// PRESERVED FROM V7.6 / V6
//   - Arduino Nano, IRremote.hpp, three-receiver ISR hookup
//   - D2/D3/D4 IR, D6/D7 motor driver, D8 laser, D9/D10 encoder, D13 LED
//   - motorLeftPWM(), motorRightPWM(), motorNeutral() exactly as tested
//   - 700 encoder counts per output revolution (same counting rule)
//   - ALL_THREE required before firing, ~2.1 s dwell, wait-for-clear
//
// ------------------------------------------------------------
// WHY V8 IS DIFFERENT
// ------------------------------------------------------------
// The chassis only feels a torque while the reaction wheel's speed is
// CHANGING:
//
//     chassis torque = -(wheel inertia) x (wheel acceleration)
//
// V7.6 commanded PWM levels. A PWM level sets the wheel's eventual SPEED,
// not the torque, so the torque the chassis actually received depended on
// how fast the wheel was already spinning - after a search it could be
// zero, or even the wrong sign. motorNeutral() is also an electrical BRAKE
// (IN1 = IN2 = HIGH), so every "stop" dumped whatever momentum the wheel
// was holding straight into the chassis. With the wheel at rest (beacon
// held at centre first) none of this shows up, which is why fine tracking
// looked good and side acquisition after a search did not.
//
// V8 closes a speed loop around the wheel using the encoder and steers by
// RAMPING the wheel-speed setpoint. The ramp rate IS the chassis torque,
// in either direction, whatever the wheel speed, gearbox friction or
// driver decay mode.
//
// The wheel is kept spinning in ONE direction (flywheel CW - the
// motorLeftPWM() direction) inside a fixed speed window:
//
//     speed the CW flywheel UP    -> chassis CCW torque (as motorLeftPWM)
//     slow the CW flywheel DOWN   -> chassis CW  torque (the same reaction
//                                    motorRightPWM() produces)
//
// Both torque directions come from the same driver path, so LEFT and RIGHT
// corrections are physically identical, the wheel never reverses through
// zero (no gearbox backlash clunk), and there is never a brake event that
// releases stored momentum. The wheel-speed window also caps how much
// energy can ever be put into the chassis/string.
//
// ONE steering law is used in every state:
//
//     torque = STEER_GAIN x (driftTarget - driftEstimate)
//
//   driftTarget   - chosen ONLY from the CURRENT sensor zone (or the
//                   search direction). The current sensors always win;
//                   there is no stored command to finish.
//   driftEstimate - the chassis rotation we have put in recently, fading
//                   away at the rate swivel friction removes it.
//
// Because driftTarget shrinks toward centre and is zero at ALL_THREE, the
// same law slows the approach, stops at the centre, and reverses at once
// if inertia carries the target through to the far side
// (RIGHT -> CENTRE -> LEFT). No approach memory, crossing guards, micro
// pulse recovery or capture state is needed. Three small additions handle
// what friction and the string do to a fixed push:
//   - a side integral that leans harder while the target stays on one
//     side, zeroed the instant it crosses over;
//   - a slow wheel REWIND (never a step) so the search can keep going
//     and tracking can regain headroom;
//   - one learned, bounded string-hold value used only during the shot.
//
// The MPU-6050 body gyro now measures actual chassis yaw rate. It is used to
// damp coasting/overshoot and to qualify a stable firing position. Its
// integrated yaw is boot-relative and is not an absolute compass heading.
//
// THE ENCODER IS NEVER TREATED AS A CHASSIS RATE SENSOR. It is used to
//   - run the wheel speed loop,
//   - measure how much momentum each correction actually delivered,
//   - enforce the wheel-speed window and over-speed / fault trips.
//
// The control loop is millis()/micros() based. A short blocking delay is used
// only during stationary gyro bias calibration at startup.
// ============================================================

#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES
void UserIRReceiveTimerInterruptHandler();
#include <IRremote.hpp>
#include <Wire.h>
#include <string.h>

// ============================================================
// PINS (unchanged)
// ============================================================

const byte LEFT_SENSOR   = 2;
const byte CENTRE_SENSOR = 3;
const byte RIGHT_SENSOR  = 4;

const byte MOTOR_IN1  = 6;
const byte MOTOR_IN2  = 7;
const byte LASER_PIN  = 8;
const byte ENCODER_A  = 9;    // PB1 / PCINT1 on the ATmega328P
const byte ENCODER_B  = 10;   // PB2
const byte STATUS_LED = 13;

// MPU-6050 is connected over I2C: SDA=A4, SCL=A5 on the Nano.
// Mount the gyro rigidly to the body, not to the reaction wheel.
const byte MPU6050_ADDRESS = 0x68; // Use 0x69 if AD0 is tied HIGH.
const byte GYRO_YAW_AXIS = 2;      // 0=X, 1=Y, 2=Z; select the body yaw axis.
const float GYRO_YAW_SIGN = 1.0f;  // Set to -1 if the measured direction is reversed.
const float GYRO_LSB_PER_DPS = 131.0f; // +/-250 dps full-scale configuration.
const unsigned long GYRO_CALIBRATION_SAMPLES = 300;
const unsigned long GYRO_CALIBRATION_INTERVAL_MS = 10;
const float GYRO_RATE_FILTER_ALPHA = 0.15f;
const float GYRO_CONFIRM_RATE_LIMIT_DPS = 0.65f;
const float GYRO_FIRE_RATE_LIMIT_DPS = 1.0f;
const float GYRO_RATE_DAMPING = 2.0f; // Drift units of counter-torque per deg/s.
const unsigned long GYRO_STALL_MS = 500;
const bool REQUIRE_GYRO_FOR_FIRE = true;

// ============================================================
// DIRECTION CONVENTION (unchanged)
// ============================================================

const bool REVERSE_MOTOR = false;

const int BODY_CCW = -1;
const int BODY_CW  = +1;

// Physical sensor geometry verified on V6:
//   target in LEFT cone  -> CCW chassis correction
//   target in RIGHT cone -> CW  chassis correction
// If the sensors are ever re-mounted, change ONLY these two lines.
const int LEFT_CORRECTION_DIRECTION  = BODY_CCW;
const int RIGHT_CORRECTION_DIRECTION = BODY_CW;

// ============================================================
// ====================  TUNING VALUES  =======================
// ============================================================
//
// UNITS
//   wheel RPM   Flywheel speed. Positive = flywheel CW (the direction
//               motorLeftPWM() drives it).
//   torque      Flywheel acceleration in wheel RPM per second.
//               The chassis torque is proportional to this, so
//               "100 RPM/s" means the same torque in either direction.
//   drift       The chassis rotation we believe we have created,
//               expressed as the wheel-RPM change that created it.
//               Positive = chassis turning CW. Bigger = faster turn.
//               (No gyro: this is an estimate used for smooth braking.)
//
// QUICK TUNING GUIDE (watch the Serial Monitor at 115200)
//   Chassis creeps too slowly / will not start moving in a side cone
//       -> raise STEERING_STRENGTH (e.g. 1.0 -> 1.5) or SIDE_PUSH_RATE
//   Approaches too fast / overshoots through centre
//       -> lower STEERING_STRENGTH, or raise COAST_TIME_S
//   Stops short and backs away from centre after a correction
//       -> lower COAST_TIME_S (the swivel friction is stronger than assumed)
//   Search too fast / too slow
//       -> DRIFT_SEARCH
//   Chassis turns backwards while the search is rewinding (LED 900 ms)
//       -> lower REWIND_TORQUE
//   One direction visibly stronger than the other
//       -> LEFT_TORQUE_SCALE / RIGHT_TORQUE_SCALE
// ============================================================

// ---------- IR and firing (same behaviour as V7.6) ----------

// Keeps decoded 38 kHz frames "present" between frames. This must be longer
// than TWO beacon frame periods, otherwise a single missed frame on one
// receiver (common at the edge of a cone) looks like a loss of alignment and
// aborts the 2.1 s shot. A typical NEC-style beacon repeats every ~108 ms,
// so 250 ms tolerates one missed frame. Larger values make the zones lag
// the real chassis position, so do not go much higher.
const unsigned long IR_HOLD_MS = 250;

// ALL_THREE must be held this long (with low estimated drift) before firing.
const unsigned long ALIGN_CONFIRM_MS = 200;
const float CONFIRM_MAX_DRIFT = 4.0f;

// Entering ALL_THREE from a side means we are on the EDGE of the overlap.
// Keep a very small push in the same direction for this long so the target
// settles inside the overlap instead of on its boundary, then stop.
// Cancelled at once if the opposite side's sensor appears.
const unsigned long ALL3_CARRY_MS = 250;
const float DRIFT_ALL3_CARRY = 4.0f;

// String hold for the shot. When the string is wound up it can pull the
// chassis out of ALL_THREE as soon as steering stops. Each time the target
// slips out of ALL_THREE during CONFIRM/FIRE, a small steady lean against
// that slip is learned (HOLD_STEP) for the next attempt on the SAME target.
// Bounded by HOLD_MAX, used only in CONFIRM/FIRE, reset for every new target.
// Set HOLD_STEP = 0 to disable.
const float HOLD_STEP = 2.0f;    // drift units per slip
const float HOLD_MAX  = 12.0f;   // drift units

// Uninterrupted laser dwell.
const unsigned long FIRE_TIME_MS = 2100;

// After a shot, the beacon must disappear for this long before searching.
const unsigned long CLEAR_CONFIRM_MS = 350;

// ---------- Steering targets (drift units) ----------
//
//     LEFT_ONLY   LEFT_CENTRE   ALL_THREE   CENTRE_RIGHT   RIGHT_ONLY
//      OUTER        INNER          0           INNER          OUTER
//       CCW          CCW         (stop)         CW              CW
//
// LEFT and RIGHT use the SAME numbers. Only the sign differs.

const float STEERING_STRENGTH = 1.00f;  // master scale for the four values below

const float DRIFT_OUTER     = 14.0f;    // LEFT_ONLY / RIGHT_ONLY
const float DRIFT_INNER     =  7.0f;    // LEFT_CENTRE / CENTRE_RIGHT
const float DRIFT_CREEP     =  3.0f;    // CENTRE_ONLY, after it has settled
const float DRIFT_REACQUIRE =  5.0f;    // brief IR loss near the target

const float DRIFT_SEARCH    = 24.0f;    // search sweep (not scaled by STEERING_STRENGTH)

// Hardware asymmetry calibration. Leave at 1.00 unless testing proves
// one chassis direction is physically stronger.
//   LEFT_TORQUE_SCALE  -> every CCW chassis torque (LEFT-side corrections)
//   RIGHT_TORQUE_SCALE -> every CW  chassis torque (RIGHT-side corrections)
const float LEFT_TORQUE_SCALE  = 1.00f;
const float RIGHT_TORQUE_SCALE = 1.00f;

// ---------- Steering law ----------

// How hard the controller pushes drift toward its target (1/s).
// 6 means it gets ~2/3 of the way there in about 1/6 s.
const float STEER_GAIN = 6.0f;

// Swivel friction model: how long the chassis keeps turning on its own
// after a push (seconds). Long = low friction. Short = high friction.
const float COAST_TIME_S = 0.8f;

// Torque ceilings (wheel RPM per second). These are the smoothness limits.
const float TRACK_MAX_TORQUE  = 250.0f;
const float SEARCH_MAX_TORQUE =  80.0f;

// Side integral ("keep leaning"): while the target stays on the SAME side
// (L/LC, or CR/R), the push grows slowly. This is what overcomes swivel
// friction and string torque, which a fixed small push cannot. It is reset
// to zero the moment the target crosses to the other side or reaches
// ALL_THREE, so it can never carry the chassis through an overshoot.
// CENTRE_ONLY and brief IR loss freeze it (no growth, no reset).
const float SIDE_PUSH_RATE = 8.0f;    // drift units added per second
const float SIDE_PUSH_MAX  = 30.0f;   // drift units

// ---------- CENTRE_ONLY and brief-loss behaviour ----------

// CENTRE_ONLY: hold still first, then creep very slowly, alternating sides.
const unsigned long CENTRE_ONLY_SETTLE_MS = 300;
const unsigned long CENTRE_CREEP_FLIP_MS  = 1200;

// IR lost while tracking (measured from the last decoded frame):
//   0 .. REACQUIRE_SETTLE_MS          hold still, let the string settle
//   .. TARGET_LOST_MS                 gentle creep toward the last seen side
//   >= TARGET_LOST_MS                 resume search toward that side
const unsigned long REACQUIRE_SETTLE_MS = 300;
const unsigned long TARGET_LOST_MS      = 900;

// If tracking needs torque the wheel-speed window cannot supply for this
// long, the wheel is gently REWOUND to the middle of its window (see below)
// and tracking then continues with fresh headroom.
const unsigned long TRACK_BLOCKED_MS = 1500;

// ---------- Rewind (how the search keeps going without a big reversal) ----------
//
// A reaction wheel cannot push forever: each push uses up wheel speed.
// With friction at the swivel, a symmetric back-and-forth sweep just
// oscillates over the same small arc. So SEARCH works like this:
//
//   PUSH    turn the chassis in the search direction (DRIFT_SEARCH) until
//           the wheel reaches the edge of the search window,
//   REWIND  bring the wheel speed back to the other edge at a torque low
//           enough that swivel friction holds the chassis still,
//   repeat.
//
// The old V7 5-second reversal did the same job with a STEP change of wheel
// speed - a huge torque that released the string and spun the chassis.
// Here the rewind is a slow ramp (SEARCH_REWIND_TORQUE), never a step, and
// IR detection interrupts it immediately.
//
// Tune: during a search REWIND the LED blinks slowly (900 ms) and telemetry
// shows "rw". If the chassis visibly turns backwards while rewinding, lower
// REWIND_TORQUE. If rewinding takes too long, raise it a little.
const float REWIND_TORQUE = 8.0f;    // wheel RPM per second (search)

// Rewind used while TRACKING (wheel out of headroom with the target in
// view). Gentler, because the string is usually already pulling the same
// way and the target must not be lost while the wheel is recentred.
const float TRACK_REWIND_TORQUE = 4.0f;   // wheel RPM per second

// Optional: reverse the search direction if nothing has been found after
// this long (0 = never). The change uses the normal capped search torque.
const unsigned long SEARCH_FLIP_AFTER_MS = 0;

// ---------- Wheel speed window (momentum budget), flywheel CW RPM ----------
//
// The wheel never leaves these limits, so the energy that can be put into
// the chassis and string is always bounded.

const float WHEEL_TRACK_MIN_RPM  =  20.0f;
const float WHEEL_TRACK_MAX_RPM  = 245.0f;

// The search only uses the MIDDLE of the window. A target always enters the
// cone on the side that needs MORE torque in the search direction (searching
// CCW, it appears in LEFT, which needs CCW), so tracking must start with
// spare wheel speed in that same direction. The gap between the search and
// track windows (65-70 RPM each side here) is that reserve.
const float WHEEL_SEARCH_MIN_RPM =  85.0f;
const float WHEEL_SEARCH_MAX_RPM = 175.0f;

// Safety trips -> FAULT state (laser off, wheel ramped down open-loop).
const float WHEEL_TRIP_RPM           = 290.0f;
const float WHEEL_BACKWARDS_TRIP_RPM =  20.0f;
const unsigned long ENCODER_STALL_MS = 700;

// ---------- Inner wheel-speed loop ----------

const unsigned long CONTROL_PERIOD_US = 10000;   // 100 Hz control loop
const byte SPEED_WINDOW_TICKS = 4;               // speed measured over 40 ms

// Feed-forward: PWM needed to hold a given wheel speed (rough is fine,
// the integral term corrects it).
const float WHEEL_FF_PWM_PER_RPM = 0.90f;
const float WHEEL_FF_PWM_OFFSET  = 12.0f;

const float WHEEL_KP      = 0.8f;    // PWM per RPM of speed error
const float WHEEL_KI      = 4.0f;    // PWM per (RPM x s)
const float WHEEL_I_LIMIT = 80.0f;   // PWM

// The setpoint may lead the measured speed by at most this much.
const float WHEEL_SETPOINT_LEAD_RPM = 25.0f;

const int WHEEL_PWM_MAX = 230;

// ---------- Start-up ----------
//
// At power-up the wheel is brought up to speed slowly (open-loop ramp until
// the encoder responds, then closed-loop). The chassis turns gently CCW
// during this, which is simply the first part of the CCW search.
// The encoder direction is learned automatically on the first movement.

const unsigned long STARTUP_SETTLE_MS = 500;
const int  START_PWM_MIN = 35;
const int  START_PWM_MAX = 110;
const unsigned long START_RAMP_MS = 1500;
const long START_DETECT_COUNTS = 20;

// 0 = learn automatically. +1 / -1 = force (only if auto-detect misbehaves).
const int ENCODER_SIGN_OVERRIDE = 0;

// ---------- Fault ramp-down ----------
const float FAULT_RAMP_PWM_PER_S = 60.0f;

// ---------- Telemetry ----------
const unsigned long TELEMETRY_MS = 200;   // 0 disables serial telemetry

// ============================================================
// ===================  END OF TUNING  ========================
// ============================================================

// ============================================================
// TYPES
// ============================================================

// ------------------------------------------------------------
// MAIN STATES
// ------------------------------------------------------------
// STARTUP     Learn encoder direction, bring the wheel into its window.
//             IR is ignored. Ends in SEARCH.
// SEARCH      No IR. PUSH: slow continuous turn (driftTarget = DRIFT_SEARCH
//             in searchDirection). REWIND: wheel speed slowly returned with
//             the chassis held by friction. ANY sensor -> TRACK immediately.
// TRACK       The sensor funnel. driftTarget comes only from the current
//             zone. Brief IR loss is handled here (settle, gentle creep).
//             Out of wheel headroom -> gentle rewind, then continue.
//             ALL_THREE -> CONFIRM. Lost > TARGET_LOST_MS -> SEARCH.
// CONFIRM     ALL_THREE: short carry off the edge of the overlap, then
//             steering target zero. Fires after ALL_THREE has been held for
//             ALIGN_CONFIRM_MS with low estimated drift.
//             Leaving ALL_THREE -> TRACK.
// FIRE        Laser on while ALL_THREE holds (steering target = learned
//             string hold, usually 0). Any loss -> laser off at once, dwell
//             timer discarded, back to TRACK. FIRE_TIME_MS -> WAIT_CLEAR.
// WAIT_CLEAR  Laser off, hold still. Beacon gone for CLEAR_CONFIRM_MS ->
//             SEARCH in the opposite direction.
// FAULT       Laser off, wheel ramped down open-loop. Reset to clear.
// ------------------------------------------------------------
enum MainState {
  STARTUP,
  SEARCH,
  TRACK,
  CONFIRM,
  FIRE,
  WAIT_CLEAR,
  FAULT
};

enum StartupPhase {
  START_WAIT,     // let everything power up, motor held
  START_KICK,     // open-loop PWM ramp until the encoder moves
  START_SPINUP    // closed loop, wheel rises into the search window
};

// ============================================================
// GLOBALS
// ============================================================

// ---------- IR ----------
// Correction direction for the side the target was most recently seen on.
// Used only when NO side sensor is active (CENTRE_ONLY creep, brief loss,
// search restart). It never overrides an active sensor.
int lastSideDirection = 0;

// ---------- State ----------
MainState state = STARTUP;
unsigned long stateSince = 0;
StartupPhase startupPhase = START_WAIT;
unsigned long startupPhaseSince = 0;
long startupCount = 0;

int searchDirection = BODY_CCW;

unsigned long alignedSince = 0;
unsigned long fireSince = 0;
unsigned long clearSince = 0;

// ---------- Sensor state ----------
Zone zone = NONE;
unsigned long zoneSince = 0;
unsigned long targetLastSeen = 0;

// ---------- Wheel / steering ----------
float wheelSetpoint = 0.0f;
float wheelIntegral = 0.0f;
int   wheelPWM = 0;

bool wheelAtCeiling = false;      // PWM saturated, wheel cannot go faster
unsigned long ceilingSince = 0;

float driftEstimate = 0.0f;       // + = wheel-model estimate of body turning CW
unsigned long gyroStableSince = 0;
float sidePush = 0.0f;            // integral push, drift units (>= 0)
int   sidePushSide = 0;           // side it was built on (BODY_CW / BODY_CCW)
int   carrySide = 0;              // side ALL_THREE was entered from
float holdBias = 0.0f;            // learned lean during CONFIRM/FIRE (+ = CW)
float driftTarget = 0.0f;
float prevWheelForDrift = 0.0f;

bool steeringBlocked = false;     // torque wanted but wheel window exhausted
unsigned long blockedSince = 0;

bool  rewinding = false;          // slow wheel-speed return in progress
float rewindTo = 0.0f;            // wheel RPM the rewind is heading for
unsigned long searchStarted = 0;

unsigned long backwardsSince = 0;
unsigned long lastTickUs = 0;

// ---------- Fault ----------
const __FlashStringHelper *faultReason = nullptr;
float faultPWM = 0.0f;

// ---------- LED ----------
unsigned long lastLedToggle = 0;
bool ledState = false;

// Optional, passive IR/gyro recenter diagnostic.  It never changes the
// autonomous state machine or motor command; it only uses its live sensor data.
bool recenterDiagnostic = false;
enum RecenterCaptureState { RECENTER_IDLE, RECENTER_TRACKING, RECENTER_DONE };
RecenterCaptureState recenterState = RECENTER_IDLE;
float recenterFirstAngle = 0.0f, recenterLastAngle = 0.0f, recenterTarget = 0.0f;
unsigned long recenterLastSeen = 0, lastDiagnosticPrint = 0;
const unsigned long RECENTER_GAP_MS = 150;
const float RECENTER_TOL_ON = 2.0f, RECENTER_TOL_OFF = 3.0f;
bool recenterLed = false;

// ============================================================
// IR (unchanged reading logic)
// ============================================================

// ============================================================
// STATE HELPERS
// ============================================================

void changeState(MainState nextState, unsigned long now) {
  if (state == nextState) {
    return;
  }

  state = nextState;
  stateSince = now;

  // A new state always starts from the sensors, never from an old rewind.
  rewinding = false;
  blockedSince = 0;
  if (nextState != CONFIRM) {
    gyroStableSince = 0;
  }

  if (nextState == SEARCH) {
    searchStarted = now;
  }
}

void enterFault(const __FlashStringHelper *reason) {
  digitalWrite(LASER_PIN, LOW);
  faultReason = reason;
  faultPWM = (float)wheelPWM;
  changeState(FAULT, millis());

  Serial.print(F("FAULT: "));
  Serial.println(faultReason);
}

// ============================================================
// WHEEL SPEED MEASUREMENT
// ============================================================

// Returns true if a fault was raised.
bool checkWheelSafety(unsigned long nowMs) {
  if (encoder.rpm() > WHEEL_TRIP_RPM) {
    enterFault(F("Wheel over-speed"));
    return true;
  }

  if (encoder.rpm() < -WHEEL_BACKWARDS_TRIP_RPM) {
    if (backwardsSince == 0) {
      backwardsSince = nowMs;
    } else if (nowMs - backwardsSince > 150) {
      enterFault(F("Wheel turning backwards - set ENCODER_SIGN_OVERRIDE"));
      return true;
    }
  } else {
    backwardsSince = 0;
  }

  if (wheelSetpoint > 25.0f && nowMs - encoder.lastCountChangeMs() > ENCODER_STALL_MS) {
    enterFault(F("Encoder stopped counting - check D9/D10 and motor power"));
    return true;
  }

  if (REQUIRE_GYRO_FOR_FIRE &&
      (state == CONFIRM || state == FIRE) &&
      (!gyro.ready() || nowMs - gyro.lastReadMs() > GYRO_STALL_MS)) {
    enterFault(F("Gyro data unavailable during alignment"));
    return true;
  }

  return false;
}

// ============================================================
// DRIFT ESTIMATE
// ============================================================
//
// Every change in measured wheel speed is an equal and opposite push on the
// chassis. We add those pushes up and let the total fade with COAST_TIME_S
// to represent swivel friction. This is the ONLY place the wheel measurement
// feeds into chassis reasoning, and it is used purely to brake smoothly.
//
// String torque is not modelled; the IR sensors correct for it.

void updateDriftEstimate(float dt) {
  float dw = encoder.rpm() - prevWheelForDrift;
  prevWheelForDrift = encoder.rpm();

  // Flywheel CW speeding up pushes the chassis CCW (negative drift).
  // A rewind is deliberately too gentle to move the chassis against swivel
  // friction, so it is not counted as chassis motion.
  if (!rewinding) {
    driftEstimate -= dw;
  }

  driftEstimate -= driftEstimate * (dt / COAST_TIME_S);
  driftEstimate = constrain(driftEstimate, -250.0f, 250.0f);
}

// ============================================================
// STEERING TARGET - the sensor funnel
// ============================================================

// Which way the CURRENT zone says to turn (0 = no side information).
int zoneSide(Zone z) {
  if (z == LEFT_ONLY  || z == LEFT_CENTRE)  return LEFT_CORRECTION_DIRECTION;
  if (z == RIGHT_ONLY || z == CENTRE_RIGHT) return RIGHT_CORRECTION_DIRECTION;
  return 0;
}

void updateSidePush(float dt) {
  if (state != TRACK) {
    sidePush = 0.0f;
    sidePushSide = 0;
    return;
  }

  int side = zoneSide(zone);

  if (side == 0) {
    return;                        // CENTRE_ONLY / LEFT_RIGHT / NONE: freeze
  }

  if (side != sidePushSide) {
    sidePush = 0.0f;               // crossed over (or first side seen)
    sidePushSide = side;
    return;
  }

  sidePush += SIDE_PUSH_RATE * dt;
  if (sidePush > SIDE_PUSH_MAX) {
    sidePush = SIDE_PUSH_MAX;
  }
}

int fallbackDirection() {
  return lastSideDirection != 0 ? lastSideDirection : searchDirection;
}

// Drift target while tracking. Depends ONLY on the zone active right now
// (plus the side integral, which is zeroed on any crossing). There is
// nothing to "finish" from a previous zone: if LEFT appears while we were
// correcting for RIGHT, the very next 10 ms tick targets CCW and the
// steering law reverses smoothly.
float trackDriftTarget(unsigned long now) {
  unsigned long zoneAge = now - zoneSince;

  switch (zone) {
    case RIGHT_ONLY:
      return RIGHT_CORRECTION_DIRECTION * (DRIFT_OUTER + sidePush);

    case CENTRE_RIGHT:
      return RIGHT_CORRECTION_DIRECTION * (DRIFT_INNER + sidePush);

    case LEFT_ONLY:
      return LEFT_CORRECTION_DIRECTION * (DRIFT_OUTER + sidePush);

    case LEFT_CENTRE:
      return LEFT_CORRECTION_DIRECTION * (DRIFT_INNER + sidePush);

    case ALL_THREE:
    case LEFT_RIGHT:
      // Aligned, or an ambiguous L+R pattern: stop and let it settle.
      return 0.0f;

    case CENTRE_ONLY: {
      // Very close but not ALL_THREE (often a side receiver missing a
      // frame). Hold still first; if it persists, creep very slowly -
      // first continuing the way we were going, then alternating.
      if (zoneAge < CENTRE_ONLY_SETTLE_MS) {
        return 0.0f;
      }

      unsigned long phase =
        (zoneAge - CENTRE_ONLY_SETTLE_MS) / CENTRE_CREEP_FLIP_MS;

      // phase 0: dir, 1-2: -dir, 3-4: dir, ... (sweeps both sides evenly)
      int dir = fallbackDirection();
      if (((phase + 1) / 2) % 2 == 1) {
        dir = -dir;
      }

      return dir * DRIFT_CREEP;
    }

    case NONE:
    default: {
      // Brief loss near the target. Hold still, then a gentle creep back
      // toward where it was last seen. No escalation.
      unsigned long lostAge = now - targetLastSeen;

      if (targetLastSeen == 0 || lostAge < REACQUIRE_SETTLE_MS) {
        return 0.0f;
      }

      return fallbackDirection() * DRIFT_REACQUIRE;
    }
  }
}

float chooseDriftTarget(unsigned long now) {
  switch (state) {
    case STARTUP:
      return BODY_CCW * DRIFT_SEARCH;           // spin-up = start of CCW sweep

    case SEARCH:
      return searchDirection * DRIFT_SEARCH;

    case TRACK:
      return STEERING_STRENGTH * trackDriftTarget(now);

    case CONFIRM:
      // Short carry into the overlap, then hold still (plus any learned
      // string hold).
      if (carrySide != 0 && now - stateSince < ALL3_CARRY_MS) {
        return carrySide * DRIFT_ALL3_CARRY + holdBias;
      }
      return holdBias;

    case FIRE:
      return holdBias;

    case WAIT_CLEAR:
    default:
      return 0.0f;                              // hold still
  }
}

// ============================================================
// STEERING LAW
// ============================================================
//
// Returns the requested chassis torque (+ = CW) in wheel RPM per second.

float steeringTorque(float target) {
  float maxTorque =
    (state == SEARCH || state == STARTUP)
    ? SEARCH_MAX_TORQUE
    : TRACK_MAX_TORQUE;

  float torque = STEER_GAIN * (target - driftEstimate);

  // The encoder predicts wheel-induced body motion. The gyro measures the
  // actual body rate, so use it as a damping term for coasting and overshoot.
  // This does not turn gyro yaw into an absolute heading.
  if (gyro.ready()) {
    torque -= GYRO_RATE_DAMPING * gyro.yawRateDps();
  }
  torque = constrain(torque, -maxTorque, maxTorque);

  if (torque > 0.0f) {
    torque *= RIGHT_TORQUE_SCALE;   // CW
  } else {
    torque *= LEFT_TORQUE_SCALE;    // CCW
  }

  return torque;
}

// Turns a chassis torque request into a wheel-speed setpoint ramp, inside
// the wheel-speed window. CW chassis torque = slow the CW flywheel down.
void advanceWheelSetpoint(float torque, float dt) {
  float lo;
  float hi;

  if (state == SEARCH || state == STARTUP) {
    lo = WHEEL_SEARCH_MIN_RPM;
    hi = WHEEL_SEARCH_MAX_RPM;
  } else {
    lo = WHEEL_TRACK_MIN_RPM;
    hi = WHEEL_TRACK_MAX_RPM;
  }

  float next = wheelSetpoint - torque * dt;
  bool blocked = false;

  if (next < wheelSetpoint) {
    // Slowing down: never below the floor (or below where we already are
    // if we start outside the window - the limits never FORCE a move).
    float floorRPM = min(wheelSetpoint, lo);
    if (next < floorRPM) {
      next = floorRPM;
      blocked = true;
    }
  } else if (next > wheelSetpoint) {
    float ceilRPM = max(wheelSetpoint, hi);
    if (next > ceilRPM) {
      next = ceilRPM;
      blocked = true;
    }
    if (wheelAtCeiling) {
      next = wheelSetpoint;
      blocked = true;
    }
  }

  // Anti-windup: the setpoint may not run away from a wheel that lags.
  next = constrain(
    next,
    encoder.rpm() - WHEEL_SETPOINT_LEAD_RPM,
    encoder.rpm() + WHEEL_SETPOINT_LEAD_RPM
  );

  wheelSetpoint = next;
  steeringBlocked = blocked && fabs(torque) > 1.0f;
}

// ============================================================
// INNER WHEEL-SPEED LOOP (feed-forward + PI)
// ============================================================

void runWheelSpeedLoop(float dt, unsigned long nowMs) {
  float err = wheelSetpoint - encoder.rpm();

  float ff = WHEEL_FF_PWM_OFFSET + WHEEL_FF_PWM_PER_RPM * wheelSetpoint;
  float u = ff + WHEEL_KP * err + wheelIntegral;

  bool satHigh = u >= (float)WHEEL_PWM_MAX;
  bool satLow  = u <= 0.0f;

  // Conditional integration (anti-windup).
  if (!(satHigh && err > 0.0f) && !(satLow && err < 0.0f)) {
    wheelIntegral += WHEEL_KI * err * dt;
    wheelIntegral = constrain(wheelIntegral, -WHEEL_I_LIMIT, WHEEL_I_LIMIT);
  }

  u = constrain(u, 0.0f, (float)WHEEL_PWM_MAX);
  wheelPWM = (int)(u + 0.5f);

  // The ONLY motor command in normal operation.
  motorController.leftPwm(wheelPWM);

  // Wheel cannot go any faster (supply / motor limit reached).
  if (satHigh && err > 3.0f) {
    if (ceilingSince == 0) {
      ceilingSince = nowMs;
    }
    wheelAtCeiling = (nowMs - ceilingSince) >= 200;
  } else {
    ceilingSince = 0;
    wheelAtCeiling = false;
  }
}

// ============================================================
// START-UP (open-loop part)
// ============================================================

void serviceStartupOpenLoop(unsigned long nowMs) {
  if (startupPhase == START_WAIT) {
    motorController.neutral();   // wheel is at rest at power-up, so this is harmless

    if (nowMs - stateSince >= STARTUP_SETTLE_MS) {
      startupPhase = START_KICK;
      startupPhaseSince = nowMs;
      startupCount = encoder.count();
    }
    return;
  }

  // START_KICK: gentle open-loop ramp of the flywheel CW.
  unsigned long k = nowMs - startupPhaseSince;
  long rampMs = (long)min(k, START_RAMP_MS);
  int pwm = (int)map(rampMs, 0L, (long)START_RAMP_MS,
                     (long)START_PWM_MIN, (long)START_PWM_MAX);

  wheelPWM = pwm;
  motorController.leftPwm(pwm);

  long displacement = encoder.count() - startupCount;

  if (labs(displacement) >= START_DETECT_COUNTS) {
    if (ENCODER_SIGN_OVERRIDE != 0) {
      encoder.setSign(ENCODER_SIGN_OVERRIDE);
    } else {
      encoder.setSign(displacement > 0 ? +1 : -1);
    }

    // Bumpless hand-over to the closed loop.
    wheelSetpoint = encoder.rpm();
    wheelIntegral = constrain(
      (float)pwm - (WHEEL_FF_PWM_OFFSET + WHEEL_FF_PWM_PER_RPM * wheelSetpoint),
      -WHEEL_I_LIMIT, WHEEL_I_LIMIT
    );

    // The kick so far has pushed the chassis CCW by this much.
    driftEstimate = -encoder.rpm();
    prevWheelForDrift = encoder.rpm();

    startupPhase = START_SPINUP;

    Serial.print(F("Encoder sign learned: "));
    Serial.println(encoder.sign());
    return;
  }

  if (k >= START_RAMP_MS + 800) {
    enterFault(F("No encoder counts at start-up - check motor power and D9/D10"));
  }
}

// ============================================================
// FAULT (open-loop gentle ramp-down)
// ============================================================

void serviceFault(float dt) {
  faultPWM -= FAULT_RAMP_PWM_PER_S * dt;

  if (faultPWM <= 0.0f) {
    faultPWM = 0.0f;
    wheelPWM = 0;
    motorController.neutral();
  } else {
    wheelPWM = (int)faultPWM;
    motorController.leftPwm(wheelPWM);
  }
}

// ============================================================
// SEARCH: PUSH / REWIND
// ============================================================
//
// CW search slows the flywheel; CCW search speeds it up. When the wheel
// reaches the edge of the search window in the search direction there is no
// push left, so the wheel is rewound slowly to the opposite edge and the
// push starts again. The search direction itself does not change.

void updateSearchSweep(unsigned long now) {
  if (!rewinding) {
    bool atEdge =
      (searchDirection == BODY_CW)
      ? (wheelSetpoint <= WHEEL_SEARCH_MIN_RPM + 0.5f)
      : (wheelSetpoint >= WHEEL_SEARCH_MAX_RPM - 0.5f || wheelAtCeiling);

    if (atEdge) {
      rewinding = true;
      rewindTo =
        (searchDirection == BODY_CW)
        ? WHEEL_SEARCH_MAX_RPM
        : WHEEL_SEARCH_MIN_RPM;
    }
  }

  if (SEARCH_FLIP_AFTER_MS > 0 && now - searchStarted >= SEARCH_FLIP_AFTER_MS) {
    searchDirection = -searchDirection;
    searchStarted = now;
    rewinding = false;
  }
}

// Replaces the steering torque with a slow, constant rewind torque while a
// rewind is in progress. Current sensors still win: if the zone already
// wants torque in the rewind direction, the rewind is simply dropped.
float applyRewind(float torque) {
  if (!rewinding) {
    return torque;
  }

  // + = CW chassis torque = slowing the CW flywheel down.
  float toward = (rewindTo < wheelSetpoint) ? 1.0f : -1.0f;

  if (fabs(wheelSetpoint - rewindTo) <= 1.0f) {
    rewinding = false;
    return torque;
  }

  if (state == TRACK && torque * toward > 0.0f) {
    rewinding = false;
    return torque;
  }

  return toward * ((state == TRACK) ? TRACK_REWIND_TORQUE : REWIND_TORQUE);
}

// ============================================================
// 100 Hz CONTROL TICK
// ============================================================

void controlTick(unsigned long nowMs) {
  unsigned long nowUs = micros();
  float dt = (float)(nowUs - lastTickUs) * 1.0e-6f;
  lastTickUs = nowUs;

  if (dt <= 0.0f || dt > 0.05f) {
    dt = (float)CONTROL_PERIOD_US * 1.0e-6f;
  }

  encoder.measure(nowUs, nowMs, SPEED_WINDOW_TICKS);
  if (gyro.ready()) {
    gyro.update(nowMs);
  }

  if (state == FAULT) {
    serviceFault(dt);
    return;
  }

  if (state == STARTUP && startupPhase != START_SPINUP) {
    serviceStartupOpenLoop(nowMs);
    return;
  }

  if (checkWheelSafety(nowMs)) {
    return;
  }

  if (state == SEARCH) {
    updateSearchSweep(nowMs);
  }

  updateDriftEstimate(dt);
  updateSidePush(dt);

  driftTarget = chooseDriftTarget(nowMs);

  float torque = applyRewind(steeringTorque(driftTarget));
  advanceWheelSetpoint(torque, dt);
  runWheelSpeedLoop(dt, nowMs);

  // Start-up finishes once the wheel is inside the search window.
  if (state == STARTUP &&
      (wheelSetpoint >= WHEEL_SEARCH_MIN_RPM || wheelAtCeiling)) {
    changeState(SEARCH, nowMs);
  }
}

// ============================================================
// STRING HOLD LEARNING (one rule)
// ============================================================
// Called when the target slips out of ALL_THREE during CONFIRM/FIRE.
// The zone it slipped into says which way to lean next time.

void learnHold() {
  int side = zoneSide(zone);

  if (side == 0) {
    return;   // slipped into CENTRE_ONLY / LEFT_RIGHT / NONE: no direction info
  }

  holdBias += side * HOLD_STEP;
  holdBias = constrain(holdBias, -HOLD_MAX, HOLD_MAX);
}

// ============================================================
// MAIN STATE MACHINE (runs every loop, so the laser reacts instantly)
// ============================================================

void updateStateMachine(unsigned long now) {
  if (steeringBlocked) {
    if (blockedSince == 0) {
      blockedSince = now;
    }
  } else {
    blockedSince = 0;
  }

  switch (state) {

    case STARTUP:
      // Handled in controlTick(). IR ignored until the wheel is ready.
      break;

    // --------------------------------------------------------
    // SEARCH: no IR. Any sensor -> TRACK immediately.
    // The search torque stops being requested on the very next tick
    // because driftTarget now comes from the zone instead.
    // --------------------------------------------------------
    case SEARCH:
      if (zone != NONE) {
        changeState(TRACK, now);
      }
      break;

    // --------------------------------------------------------
    // TRACK: the sensor funnel (see trackDriftTarget()).
    // --------------------------------------------------------
    case TRACK:
      if (zone == ALL_THREE) {
        // The side we were being pushed from is the edge we just crossed.
        carrySide = sidePushSide;
        alignedSince = now + ALL3_CARRY_MS;
        gyroStableSince = 0;
        changeState(CONFIRM, now);
      } else if (zone == NONE &&
                 targetLastSeen != 0 &&
                 now - targetLastSeen >= TARGET_LOST_MS) {
        searchDirection = fallbackDirection();
        holdBias = 0.0f;
        changeState(SEARCH, now);
      } else if (!rewinding &&
                 blockedSince != 0 &&
                 now - blockedSince >= TRACK_BLOCKED_MS) {
        // Wheel window exhausted in the needed direction: rewind gently to
        // the middle of the window, then carry on tracking.
        rewinding = true;
        rewindTo = 0.5f * (WHEEL_TRACK_MIN_RPM + WHEEL_TRACK_MAX_RPM);
        blockedSince = 0;
      }
      break;

    // --------------------------------------------------------
    // CONFIRM: ALL_THREE must hold, with the chassis believed settled.
    // --------------------------------------------------------
    case CONFIRM:
      if (zone != ALL_THREE) {
        if (now - stateSince >= ALL3_CARRY_MS) {
          learnHold();
        }
        carrySide = 0;
        changeState(TRACK, now);
      } else if (now - stateSince >= ALL3_CARRY_MS) {
        carrySide = 0;

        if (fabs(driftEstimate - driftTarget) > CONFIRM_MAX_DRIFT ||
            (long)(now - alignedSince) < 0) {
          alignedSince = now;   // still coasting: restart the confirm timer
        }

        if (fabs(gyro.yawRateDps()) <= GYRO_CONFIRM_RATE_LIMIT_DPS) {
          if (gyroStableSince == 0) {
            gyroStableSince = now;
          }
        } else {
          gyroStableSince = 0;
        }

        bool gyroSettled =
          gyroStableSince != 0 &&
          now - gyroStableSince >= ALIGN_CONFIRM_MS;

        if (gyroSettled && now - alignedSince >= ALIGN_CONFIRM_MS) {
          fireSince = now;
          changeState(FIRE, now);
        }
      }
      break;

    // --------------------------------------------------------
    // FIRE: laser on only while ALL_THREE holds (enforced in loop()).
    // --------------------------------------------------------
    case FIRE:
      if (zone != ALL_THREE ||
          !gyro.ready() ||
          now - gyro.lastReadMs() > GYRO_STALL_MS ||
          fabs(gyro.yawRateDps()) > GYRO_FIRE_RATE_LIMIT_DPS) {
        // Any loss: laser off this same loop pass, dwell discarded.
        learnHold();
        fireSince = 0;
        alignedSince = 0;
        changeState(TRACK, now);
      } else if (now - fireSince >= FIRE_TIME_MS) {
        clearSince = 0;
        changeState(WAIT_CLEAR, now);
        Serial.println(F("SHOT COMPLETE"));
      }
      break;

    // --------------------------------------------------------
    // WAIT_CLEAR: hold still until the beacon has gone.
    // --------------------------------------------------------
    case WAIT_CLEAR:
      if (zone == NONE) {
        if (clearSince == 0) {
          clearSince = now;
        }

        if (now - clearSince >= CLEAR_CONFIRM_MS) {
          searchDirection = -searchDirection;   // alternate each target
          lastSideDirection = 0;
          holdBias = 0.0f;                      // new target, new string angle
          changeState(SEARCH, now);
        }
      } else {
        clearSince = 0;
      }
      break;

    case FAULT:
    default:
      break;
  }
}

// ============================================================
// STATUS LED
// ============================================================
//   STARTUP     250 ms blink
//   SEARCH      450 ms blink (900 ms while rewinding)
//   TRACK        90 ms blink
//   CONFIRM/FIRE solid
//   WAIT_CLEAR 1000 ms blink
//   FAULT        40 ms blink

void updateStatusLED(unsigned long now) {
  if (recenterDiagnostic) {
    digitalWrite(STATUS_LED, recenterLed ? HIGH : LOW);
    return;
  }
  if (state == CONFIRM || state == FIRE) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }

  unsigned long interval = 450;

  switch (state) {
    case STARTUP:    interval = 250;  break;
    case SEARCH:     interval = rewinding ? 900 : 450; break;
    case TRACK:      interval = 90;   break;
    case WAIT_CLEAR: interval = 1000; break;
    case FAULT:      interval = 40;   break;
    default:         interval = 450;  break;
  }

  if (now - lastLedToggle >= interval) {
    lastLedToggle = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
  }
}

void printGyroDiagnostics() {
  Serial.print(F("GYRO who=0x")); Serial.print(gyro.whoAmI(), HEX);
  Serial.print(F(" init=")); Serial.print(gyro.initialized() ? 1 : 0);
  Serial.print(F(" ready=")); Serial.print(gyro.ready() ? 1 : 0);
  Serial.print(F(" rate="));
  for (byte a = 0; a < 3; ++a) { if (a) Serial.print(','); Serial.print(gyro.rateDps(a), 2); }
  Serial.print(F(" angle="));
  for (byte a = 0; a < 3; ++a) { if (a) Serial.print(','); Serial.print(gyro.angleDegrees(a), 1); }
  Serial.print(F(" failures=")); Serial.println(gyro.readFailures());
}

void startRecenterDiagnostic() {
  recenterDiagnostic = true; recenterState = RECENTER_IDLE; recenterLed = false;
  digitalWrite(STATUS_LED, LOW);
  Serial.println(F("RECENTER started (passive; autonomous control remains active)"));
}

void serviceRecenterDiagnostic(unsigned long now) {
  bool detected = zone != NONE;
  if (recenterState == RECENTER_IDLE && detected) {
    recenterFirstAngle = recenterLastAngle = gyro.yawDegrees();
    recenterLastSeen = now; recenterState = RECENTER_TRACKING;
    Serial.print(F("IR FIRST angle=")); Serial.println(recenterFirstAngle, 2);
  } else if (recenterState == RECENTER_TRACKING) {
    if (detected) {
      recenterLastAngle = gyro.yawDegrees(); recenterLastSeen = now;
    } else if (now - recenterLastSeen > RECENTER_GAP_MS) {
      recenterTarget = (recenterFirstAngle + recenterLastAngle) * 0.5f;
      recenterState = RECENTER_DONE;
      Serial.print(F("IR LAST angle=")); Serial.println(recenterLastAngle, 2);
      Serial.print(F("RECENTER target=")); Serial.println(recenterTarget, 2);
    }
  }

  if (recenterState == RECENTER_DONE) {
    float error = recenterTarget - gyro.yawDegrees();
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    float absError = fabs(error);
    if (!recenterLed && absError <= RECENTER_TOL_ON) recenterLed = true;
    if (recenterLed && absError > RECENTER_TOL_OFF) recenterLed = false;
    if (now - lastDiagnosticPrint >= 200) {
      lastDiagnosticPrint = now;
      Serial.print(F("RECENTER angle=")); Serial.print(gyro.yawDegrees(), 2);
      Serial.print(F(" target=")); Serial.print(recenterTarget, 2);
      Serial.print(F(" error=")); Serial.print(error, 2);
      Serial.println(recenterLed ? F(" CENTERED") : F(" seeking"));
    }
  }
}

void serviceSerialCommands() {
  static char command[20]; static byte length = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      command[length] = '\0';
      if (length == 1 && (command[0] == 'R' || command[0] == 'r')) startRecenterDiagnostic();
      else if (strcmp(command, "RECENTER") == 0 || strcmp(command, "recenter") == 0) startRecenterDiagnostic();
      else if ((length == 1 && (command[0] == 'G' || command[0] == 'g')) ||
               strcmp(command, "GYRO") == 0 || strcmp(command, "gyro") == 0) printGyroDiagnostics();
      else if (strcmp(command, "NORMAL") == 0 || strcmp(command, "normal") == 0) {
        recenterDiagnostic = false; recenterState = RECENTER_IDLE; recenterLed = false;
        Serial.println(F("NORMAL autonomous mode"));
      } else if (length) {
        Serial.println(F("Commands: RECENTER (R), GYRO (G), NORMAL"));
      }
      length = 0;
    } else if (length < sizeof(command) - 1) {
      command[length++] = c;
    }
  }
}

// ============================================================
// SERIAL TELEMETRY
// ============================================================
//
//   t      ms since power-up
//   state  STARTUP / SEARCH / TRACK / CONFIRM / FIRE / WAIT / FAULT
//   z      - L LC C LCR CR R LR
//   w      measured wheel RPM (flywheel CW positive)
//   sp     wheel setpoint RPM
//   d      estimated chassis drift (+ = CW)
//   tg     drift target from the current zone
//   pwm    motorLeftPWM() power
//   las    laser 1/0
//   rw     shown while the wheel is being rewound
//   hold   learned string hold for the current target (when non-zero)

const __FlashStringHelper *stateName(MainState s) {
  switch (s) {
    case STARTUP:    return F("STARTUP");
    case SEARCH:     return F("SEARCH");
    case TRACK:      return F("TRACK");
    case CONFIRM:    return F("CONFIRM");
    case FIRE:       return F("FIRE");
    case WAIT_CLEAR: return F("WAIT");
    case FAULT:      return F("FAULT");
    default:         return F("?");
  }
}

const __FlashStringHelper *zoneName(Zone z) {
  switch (z) {
    case NONE:         return F("-");
    case LEFT_ONLY:    return F("L");
    case LEFT_CENTRE:  return F("LC");
    case CENTRE_ONLY:  return F("C");
    case ALL_THREE:    return F("LCR");
    case CENTRE_RIGHT: return F("CR");
    case RIGHT_ONLY:   return F("R");
    case LEFT_RIGHT:   return F("LR");
    default:           return F("?");
  }
}

void printTelemetry(unsigned long now) {
  static unsigned long lastPrint = 0;

  if (TELEMETRY_MS == 0 || now - lastPrint < TELEMETRY_MS) {
    return;
  }
  lastPrint = now;

  Serial.print(F("t="));      Serial.print(now);
  Serial.print(' ');          Serial.print(stateName(state));
  Serial.print(F(" z="));     Serial.print(zoneName(zone));
  Serial.print(F(" w="));     Serial.print(encoder.rpm(), 1);
  Serial.print(F(" sp="));    Serial.print(wheelSetpoint, 1);
  Serial.print(F(" d="));     Serial.print(driftEstimate, 1);
  Serial.print(F(" tg="));    Serial.print(driftTarget, 1);
  Serial.print(F(" gr="));     Serial.print(gyro.yawRateDps(), 2);
  Serial.print(F(" gy="));     Serial.print(gyro.yawDegrees(), 1);
  Serial.print(F(" pwm="));    Serial.print(wheelPWM);
  Serial.print(F(" las="));   Serial.print(digitalRead(LASER_PIN) ? 1 : 0);

  if (rewinding) {
    Serial.print(F(" rw"));
  }

  if (holdBias != 0.0f) {
    Serial.print(F(" hold="));
    Serial.print(holdBias, 0);
  }

  if (state == FAULT && faultReason != nullptr) {
    Serial.print(F(" ! "));
    Serial.print(faultReason);
  }

  Serial.println();
}

// ============================================================
// SETUP
// ============================================================

void sentinelSetup() {
  Serial.begin(115200);

  motorController.begin();
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(LASER_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);

  motorController.neutral();

  encoder.begin();

  if (gyro.begin() && gyro.calibrate(GYRO_CALIBRATION_SAMPLES, GYRO_CALIBRATION_INTERVAL_MS)) {
        Serial.print(F("MPU-6050 gyro ready WHO_AM_I=0x"));
        Serial.println(gyro.whoAmI(), HEX);
  } else {
        Serial.println(F("FAULT: MPU-6050 unavailable or calibration failed"));
    if (REQUIRE_GYRO_FOR_FIRE) {
      enterFault(F("MPU-6050 unavailable - laser disabled"));
    }
  }

  irSensors.begin();

  unsigned long now = millis();
  stateSince = now;
  zoneSince = now;
  lastTickUs = micros();

  Serial.println(F("MQ Sentinel V8.0 - momentum funnel"));
  Serial.println(F("Commands: RECENTER (R), GYRO (G), NORMAL"));
}

// ============================================================
// LOOP
// ============================================================

void sentinelLoop() {
  unsigned long now = millis();

  serviceSerialCommands();
  encoder.poll();          // no-op on the Nano (interrupt driven)
  irSensors.read(now);
  irSensors.updateZone(now);

  zone = irSensors.zone();
  zoneSince = irSensors.zoneSince();
  targetLastSeen = irSensors.targetLastSeen();
  if (recenterDiagnostic) {
    serviceRecenterDiagnostic(now);
  }

  if (micros() - lastTickUs >= CONTROL_PERIOD_US) {
    controlTick(now);
  }

  updateStateMachine(now);

  // The ONLY place the laser can be switched on. It is on strictly while
  // in FIRE with all three sensors active, so it goes off on the same loop
  // pass that ALL_THREE is lost.
  bool gyroSafeForLaser =
    gyro.ready() &&
    now - gyro.lastReadMs() <= GYRO_STALL_MS &&
    fabs(gyro.yawRateDps()) <= GYRO_FIRE_RATE_LIMIT_DPS;
  bool laserOn = (state == FIRE) && (zone == ALL_THREE) && gyroSafeForLaser;
  digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);

  updateStatusLED(now);
  printTelemetry(now);
}

// ============================================================
// MULTIPLE IR RECEIVER TIMER HANDLER (unchanged)
// ============================================================
// IRremote's Timer2 ISR services IrReceiver (LEFT) itself, then calls this
// for the other two instances.

void UserIRReceiveTimerInterruptHandler() {
  IrSensors::timerInterrupt();
}

void SentinelController::begin() {
  sentinelSetup();
}

void SentinelController::update() {
  sentinelLoop();
}
