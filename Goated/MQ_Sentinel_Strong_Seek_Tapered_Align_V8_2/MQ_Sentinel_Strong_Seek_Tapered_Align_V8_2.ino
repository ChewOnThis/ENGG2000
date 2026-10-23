// ============================================================
// MQ SENTINEL - STRONG SEEK + TAPERED ALIGNMENT V8.2
//
// PURPOSE
// - If NO IR is detected, the Sentinel performs a slow continuous seek.
// - Seek is gentle and has NO timed direction reversal.
// - LEFT/RIGHT sensors continuously funnel the beacon toward the centre.
// - Outer correction starts strong for fast acquisition.
// - While the same side remains active, correction power continuously tapers.
// - LEFT+CENTRE / CENTRE+RIGHT immediately switch to a much finer taper.
// - CENTRE_ONLY / ALL_THREE actively arrest overshoot, then release.
// - Opposite-side detection immediately changes the commanded direction.
// - LEFT and RIGHT powers are numerically identical.
//
// PHYSICALLY VERIFIED MOTOR MAPPING
// motorLeftPWM()  -> flywheel CLOCKWISE
//                 -> chassis COUNTER-CLOCKWISE torque
//
// motorRightPWM() -> flywheel COUNTER-CLOCKWISE
//                 -> chassis CLOCKWISE torque
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
// D13 Built-in LED
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
// VERIFIED DIRECTION CONVENTION
// ============================================================

const bool REVERSE_MOTOR = false;

const int BODY_CCW = -1;
const int BODY_CW  = +1;

// LEFT sensor means move chassis CCW toward centre.
// RIGHT sensor means move chassis CW toward centre.
const int LEFT_DIRECTION  = BODY_CCW;
const int RIGHT_DIRECTION = BODY_CW;

// Equal calibration by default.
const float LEFT_TORQUE_SCALE  = 1.00f;
const float RIGHT_TORQUE_SCALE = 1.00f;

// ============================================================
// IR
// ============================================================

IRrecv centreReceiver(CENTRE_SENSOR);
IRrecv rightReceiver(RIGHT_SENSOR);

unsigned long lastLeftIR = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR = 0;

// Long enough to bridge repeated IRremote frames.
const unsigned long IR_HOLD_MS = 140;

// Ignore a sensor transition until it has existed briefly.
const unsigned long ZONE_STABLE_MS = 22;

// ============================================================
// ENCODER
// ============================================================

const long COUNTS_PER_REV = 700;

long encoderCount = 0;
byte previousEncoderA = LOW;

long previousSpeedCount = 0;
unsigned long previousSpeedTime = 0;
float wheelRPM = 0.0f;

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
  SEEK,
  TRACKING,
  CONFIRM,
  FIRING,
  WAIT_CLEAR
};

MainState state = SEEK;

unsigned long alignedSince = 0;
unsigned long fireSince = 0;
unsigned long clearSince = 0;

const unsigned long ALIGN_CONFIRM_MS = 220;
const unsigned long FIRE_TIME_MS = 2100;
const unsigned long CLEAR_CONFIRM_MS = 350;

// ============================================================
// PROGRESSIVE ALIGNMENT TUNING
// ============================================================
//
// OUTER sensor:
//   quick initial acquisition, then ramps down while still on that sensor.
//
// INNER overlap:
//   much gentler continuous steering as centre is approached.
//
// CENTRE / ALL THREE:
//   small opposite torque to remove the most recent steering momentum,
//   then release.
//
// BOTH SIDES USE THE SAME VALUES.
// ============================================================

// LEFT_ONLY / RIGHT_ONLY
const int OUTER_START_POWER = 112;
const int OUTER_END_POWER   = 58;
const unsigned long OUTER_SLOWDOWN_MS = 520;

// LEFT+CENTRE / CENTRE+RIGHT
const int INNER_START_POWER = 56;
const int INNER_END_POWER   = 34;
const unsigned long INNER_SLOWDOWN_MS = 320;

// If opposite side becomes active, use this until old flywheel rotation
// has reached/crossed approximately zero.
const int REVERSAL_START_POWER = 108;
const int REVERSAL_END_POWER   = 62;
const unsigned long REVERSAL_RAMP_MS = 140;
const float REVERSAL_ZERO_RPM = 6.0f;
const unsigned long REVERSAL_TIMEOUT_MS = 240;

// When centre is reached, gently oppose the last steering direction.
// This is a single continuous arrest phase, not repeated pulses.
const int CENTRE_ARREST_START_POWER = 62;
const int CENTRE_ARREST_END_POWER   = 36;
const unsigned long CENTRE_ARREST_RAMP_MS = 150;
const float CENTRE_STOP_RPM = 6.0f;
const unsigned long CENTRE_ARREST_TIMEOUT_MS = 180;

// ============================================================
// SEEK TUNING
// ============================================================
//
// When NO IR is detected, rotate slowly and continuously until a beacon
// enters any receiver cone.
//
// There is NO automatic timed reversal. That was previously responsible
// for large stored momentum/string wind-up and sudden multi-revolution
// behaviour.
//
// Seek direction alternates only AFTER a completed target has cleared.
// ============================================================

int seekDirection = BODY_CCW;

const int SEEK_START_POWER = 105;
const int SEEK_RUN_POWER   = 132;
const int SEEK_LOW_POWER   = 102;

const unsigned long SEEK_RAMP_MS = 650;

// As wheel speed builds, back off the seek torque so the reaction wheel
// does not keep accelerating aggressively.
const float SEEK_RPM_LIMIT = 120.0f;
const float SEEK_RPM_SOFT  = 80.0f;

unsigned long seekStarted = 0;

// Brief ONE-TIME release when seek first finds IR.
// This prevents the stronger search torque carrying straight through the
// target before the progressive alignment controller takes over.
const unsigned long SEEK_CAPTURE_RELEASE_MS = 45;
unsigned long seekCaptureStarted = 0;
bool seekCaptureActive = false;

// ============================================================
// CONTROLLER MEMORY
// ============================================================

enum DriveMode {
  DRIVE_IDLE,
  DRIVE_TRACKING,
  DRIVE_REVERSING,
  DRIVE_CENTRE_ARREST
};

DriveMode driveMode = DRIVE_IDLE;

int currentDirection = 0;
int desiredDirection = 0;
int desiredPower = 0;

int lastSteeringDirection = 0;

float reversalStartRPM = 0.0f;
unsigned long driveModeSince = 0;

// ============================================================
// MOTOR FUNCTIONS - KEEP VERIFIED MAPPING
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
  power = constrain(power, 0, 255);

  if (direction == BODY_CCW) {
    int scaled = (int)(power * LEFT_TORQUE_SCALE);
    motorLeftPWM(constrain(scaled, 0, 255));
  }

  else if (direction == BODY_CW) {
    int scaled = (int)(power * RIGHT_TORQUE_SCALE);
    motorRightPWM(constrain(scaled, 0, 255));
  }
}

// True LOW/LOW release for the IN/IN behaviour used by the latest controller.
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

float absFloat(float x) {
  return x < 0.0f ? -x : x;
}

// ============================================================
// IR
// ============================================================

void readIR(unsigned long now) {
  if (IrReceiver.decode()) {
    lastLeftIR = now;
    IrReceiver.resume();
  }

  if (centreReceiver.decode()) {
    lastCentreIR = now;
    centreReceiver.resume();
  }

  if (rightReceiver.decode()) {
    lastRightIR = now;
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
}

bool targetVisible() {
  return zone != NONE;
}

bool aligned() {
  return zone == ALL_THREE;
}

// ============================================================
// POWER CURVES
// ============================================================

int rampDownPower(
  int startPower,
  int endPower,
  unsigned long elapsed,
  unsigned long duration
) {
  if (elapsed >= duration) {
    return endPower;
  }

  long p = map(
    (long)elapsed,
    0L,
    (long)duration,
    (long)startPower,
    (long)endPower
  );

  return constrain(
    (int)p,
    min(startPower, endPower),
    max(startPower, endPower)
  );
}

int trackingDirectionForZone(Zone z) {
  if (z == LEFT_ONLY || z == LEFT_CENTRE) {
    return LEFT_DIRECTION;
  }

  if (z == RIGHT_ONLY || z == CENTRE_RIGHT) {
    return RIGHT_DIRECTION;
  }

  // LEFT+RIGHT without centre can occur because IR_HOLD_MS keeps recent
  // frames alive. Whichever side was detected most recently wins.
  if (z == LEFT_RIGHT) {
    if (lastLeftIR > lastRightIR) {
      return LEFT_DIRECTION;
    }

    if (lastRightIR > lastLeftIR) {
      return RIGHT_DIRECTION;
    }
  }

  return 0;
}

int trackingPowerForZone(Zone z, unsigned long now) {
  unsigned long elapsed = now - zoneSince;

  if (z == LEFT_ONLY || z == RIGHT_ONLY) {
    return rampDownPower(
      OUTER_START_POWER,
      OUTER_END_POWER,
      elapsed,
      OUTER_SLOWDOWN_MS
    );
  }

  if (z == LEFT_CENTRE || z == CENTRE_RIGHT) {
    return rampDownPower(
      INNER_START_POWER,
      INNER_END_POWER,
      elapsed,
      INNER_SLOWDOWN_MS
    );
  }

  // Ambiguous two-side/no-centre condition gets inner-level power only.
  if (z == LEFT_RIGHT) {
    return INNER_END_POWER;
  }

  return 0;
}

// ============================================================
// SEEK CONTROL
// ============================================================

int getSeekPower(unsigned long now) {
  int targetPower = SEEK_RUN_POWER;

  float speed = absFloat(wheelRPM);

  if (speed >= SEEK_RPM_LIMIT) {
    targetPower = SEEK_LOW_POWER;
  }

  else if (speed >= SEEK_RPM_SOFT) {
    targetPower = (SEEK_RUN_POWER + SEEK_LOW_POWER) / 2;
  }

  unsigned long elapsed = now - seekStarted;

  if (elapsed < SEEK_RAMP_MS) {
    targetPower = map(
      (long)elapsed,
      0L,
      (long)SEEK_RAMP_MS,
      (long)SEEK_START_POWER,
      (long)targetPower
    );
  }

  return constrain(targetPower, 0, 255);
}

void beginSeek(unsigned long now) {
  seekStarted = now;

  // Clear old alignment/reversal state so a previous target cannot influence
  // the next search.
  driveMode = DRIVE_IDLE;
  currentDirection = 0;
  desiredDirection = 0;
  desiredPower = 0;
  lastSteeringDirection = 0;
}

void serviceSeek(unsigned long now) {
  if (targetVisible()) {
    return;
  }

  driveBodyTorque(
    seekDirection,
    getSeekPower(now)
  );
}

// ============================================================
// DRIVE CONTROL
// ============================================================

void stopDrive() {
  motorRelease();

  driveMode = DRIVE_IDLE;
  currentDirection = 0;
  desiredDirection = 0;
  desiredPower = 0;
}

void beginReversal(
  int newDirection,
  int newPower,
  unsigned long now
) {
  reversalStartRPM = wheelRPM;

  desiredDirection = newDirection;
  desiredPower = newPower;

  currentDirection = newDirection;
  driveMode = DRIVE_REVERSING;
  driveModeSince = now;

  driveBodyTorque(
    newDirection,
    REVERSAL_START_POWER
  );
}

void beginCentreArrest(unsigned long now) {
  if (lastSteeringDirection == 0) {
    stopDrive();
    return;
  }

  currentDirection = -lastSteeringDirection;
  driveMode = DRIVE_CENTRE_ARREST;
  driveModeSince = now;

  driveBodyTorque(
    currentDirection,
    CENTRE_ARREST_START_POWER
  );
}

void serviceTracking(unsigned long now) {
  // ----------------------------------------------------------
  // NO IR: STATIONARY
  // ----------------------------------------------------------
  if (zone == NONE) {
    stopDrive();
    return;
  }

  // ----------------------------------------------------------
  // CENTRE / ALL THREE
  // Gradually arrest the last approach, then release.
  // ----------------------------------------------------------
  if (zone == CENTRE_ONLY || zone == ALL_THREE) {
    if (driveMode != DRIVE_CENTRE_ARREST) {
      beginCentreArrest(now);
    }

    if (driveMode == DRIVE_CENTRE_ARREST) {
      bool stopped =
        absFloat(wheelRPM) <= CENTRE_STOP_RPM;

      bool timeout =
        now - driveModeSince >=
        CENTRE_ARREST_TIMEOUT_MS;

      if (stopped || timeout) {
        stopDrive();
        return;
      }

      int arrestPower = rampDownPower(
        CENTRE_ARREST_START_POWER,
        CENTRE_ARREST_END_POWER,
        now - driveModeSince,
        CENTRE_ARREST_RAMP_MS
      );

      driveBodyTorque(
        -lastSteeringDirection,
        arrestPower
      );
    }

    return;
  }

  // ----------------------------------------------------------
  // SIDE ERROR
  // ----------------------------------------------------------

  int newDirection =
    trackingDirectionForZone(zone);

  int newPower =
    trackingPowerForZone(zone, now);

  if (newDirection == 0 || newPower == 0) {
    stopDrive();
    return;
  }

  // Opposite sensor has become active:
  // the target crossed the middle. Reverse the flywheel course now.
  if (
    lastSteeringDirection != 0 &&
    newDirection != lastSteeringDirection &&
    driveMode != DRIVE_REVERSING
  ) {
    beginReversal(
      newDirection,
      newPower,
      now
    );

    lastSteeringDirection =
      newDirection;

    return;
  }

  // ----------------------------------------------------------
  // REVERSAL PHASE
  // ----------------------------------------------------------
  if (driveMode == DRIVE_REVERSING) {
    bool crossedZero = false;

    if (absFloat(reversalStartRPM) > REVERSAL_ZERO_RPM) {
      crossedZero =
        (
          reversalStartRPM > 0.0f &&
          wheelRPM <= 0.0f
        ) ||
        (
          reversalStartRPM < 0.0f &&
          wheelRPM >= 0.0f
        );
    }

    bool nearZero =
      absFloat(wheelRPM) <=
      REVERSAL_ZERO_RPM;

    bool timeout =
      now - driveModeSince >=
      REVERSAL_TIMEOUT_MS;

    if (
      nearZero ||
      crossedZero ||
      timeout
    ) {
      driveMode = DRIVE_TRACKING;
      currentDirection = desiredDirection;

      driveBodyTorque(
        desiredDirection,
        desiredPower
      );

      return;
    }

    int reversalPower = rampDownPower(
      REVERSAL_START_POWER,
      REVERSAL_END_POWER,
      now - driveModeSince,
      REVERSAL_RAMP_MS
    );

    driveBodyTorque(
      desiredDirection,
      reversalPower
    );

    return;
  }

  // ----------------------------------------------------------
  // NORMAL PROGRESSIVE CONTINUOUS TRACKING
  // ----------------------------------------------------------
  driveMode = DRIVE_TRACKING;

  currentDirection = newDirection;
  desiredDirection = newDirection;
  desiredPower = newPower;

  lastSteeringDirection =
    newDirection;

  driveBodyTorque(
    newDirection,
    newPower
  );
}

// ============================================================
// LED
// ============================================================

void updateLED(unsigned long now) {
  static unsigned long lastToggle = 0;
  static bool ledState = false;

  if (state == CONFIRM || state == FIRING) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }

  if (state == WAIT_CLEAR) {
    digitalWrite(STATUS_LED, LOW);
    return;
  }

  if (now - lastToggle >= 100) {
    lastToggle = now;
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

  previousEncoderA =
    digitalRead(ENCODER_A);

  previousSpeedTime = millis();

  motorRelease();

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

  zoneSince = millis();
  beginSeek(millis());
}

// ============================================================
// MAIN LOOP
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

  updateLED(now);

  // ==========================================================
  // SEEK
  // NO IR = SLOW CONTINUOUS SEARCH ROTATION
  // ==========================================================

  if (state == SEEK) {
    digitalWrite(LASER_PIN, LOW);

    if (targetVisible()) {
      // Remove the stronger seek torque immediately.
      motorRelease();

      driveMode = DRIVE_IDLE;
      currentDirection = 0;
      desiredDirection = 0;
      desiredPower = 0;
      lastSteeringDirection = 0;

      seekCaptureStarted = now;
      seekCaptureActive = true;

      state = TRACKING;
    }

    else {
      serviceSeek(now);
    }
  }

  // ==========================================================
  // TRACKING
  // ==========================================================

  else if (state == TRACKING) {
    digitalWrite(LASER_PIN, LOW);

    if (!targetVisible()) {
      seekCaptureActive = false;

      stopDrive();
      lastSteeringDirection = 0;

      beginSeek(now);
      state = SEEK;
    }

    else if (seekCaptureActive) {
      // One-time smooth handover from strong seek to alignment.
      motorRelease();

      if (
        now - seekCaptureStarted >=
        SEEK_CAPTURE_RELEASE_MS
      ) {
        seekCaptureActive = false;

        // Start the correction taper from this stable handover point.
        zoneSince = now;
      }
    }

    else if (
      aligned() &&
      now - zoneSince >=
      ZONE_STABLE_MS
    ) {
      // Arrest residual approach momentum before confirmation.
      serviceTracking(now);

      if (driveMode == DRIVE_IDLE) {
        alignedSince = now;
        state = CONFIRM;
      }
    }

    else if (
      now - zoneSince >=
      ZONE_STABLE_MS
    ) {
      serviceTracking(now);
    }
  }

  // ==========================================================
  // CONFIRM
  // ==========================================================

  else if (state == CONFIRM) {
    digitalWrite(LASER_PIN, LOW);

    if (!aligned()) {
      alignedSince = 0;
      state = TRACKING;
    }

    else {
      stopDrive();

      if (
        now - alignedSince >=
        ALIGN_CONFIRM_MS
      ) {
        fireSince = now;
        digitalWrite(LASER_PIN, HIGH);
        state = FIRING;
      }
    }
  }

  // ==========================================================
  // FIRING
  // ==========================================================

  else if (state == FIRING) {
    if (!aligned()) {
      digitalWrite(LASER_PIN, LOW);

      fireSince = 0;
      alignedSince = 0;

      state = TRACKING;
    }

    else {
      stopDrive();
      digitalWrite(LASER_PIN, HIGH);

      if (
        now - fireSince >=
        FIRE_TIME_MS
      ) {
        digitalWrite(LASER_PIN, LOW);

        clearSince = 0;
        state = WAIT_CLEAR;
      }
    }
  }

  // ==========================================================
  // WAIT CLEAR
  // ==========================================================

  else if (state == WAIT_CLEAR) {
    digitalWrite(LASER_PIN, LOW);
    stopDrive();

    if (zone == NONE) {
      if (clearSince == 0) {
        clearSince = now;
      }

      if (
        now - clearSince >=
        CLEAR_CONFIRM_MS
      ) {
        clearSince = 0;
        lastSteeringDirection = 0;

        // Alternate direction only between completed targets.
        // Never reverse automatically while continuously searching.
        seekDirection = -seekDirection;
        seekCaptureActive = false;

        beginSeek(now);
        state = SEEK;
      }
    }

    else {
      clearSince = 0;
    }
  }

  // ==========================================================
  // OPTIONAL SERIAL
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

    Serial.print(" DRIVE:");
    Serial.print((int)driveMode);

    Serial.print(" DIR:");
    Serial.print(currentDirection);

    Serial.print(" POWER:");
    Serial.print(desiredPower);

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
