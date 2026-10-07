// ============================================================

// MQ SENTINEL - PRECISION REACTION-WHEEL IR TRACKER V2

// Arduino Nano
//
// ALIGNMENT RULE V3:
// LEFT + CENTRE + RIGHT must ALL be active at the same time.
// Centre sensor alone is NOT considered aligned.
// Laser firing is impossible unless all 3 sensors are active.
//
//
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

// Main improvements:

//  1. Side detection starts braking BEFORE centre is reached.

//  2. Centre is treated as a capture region, not an instant "stop".

//  3. High-speed capture uses wheel RPM + side->centre crossing time.

//  4. After every correction, the new flywheel RPM is actively held.

//     This prevents flywheel/gearbox drag slowly giving momentum back

//     to the chassis after it looked stationary.

//  5. Fine alignment uses much smaller impulses and much longer

//     observation periods to reduce L-C-R hunting.

//  6. Laser only fires after LEFT + CENTRE + RIGHT are ALL active and stable.

//

// IMPORTANT:

// The reaction wheel is the actuator. Braking/locking the wheel does

// not directly lock the chassis. This controller therefore controls

// chassis motion by changing wheel momentum, then holds the resulting

// wheel speed while the satellite is aligned.

// ============================================================



#define SUPPORT_MULTIPLE_RECEIVER_INSTANCES

void UserIRReceiveTimerInterruptHandler();

#include <IRremote.hpp>



// ============================================================

// PINS

// ============================================================



const byte LEFT_SENSOR = 2;

const byte CENTRE_SENSOR = 3;

const byte RIGHT_SENSOR = 4;



const byte MOTOR_IN1 = 6;  // PWM capable on Nano

const byte MOTOR_IN2 = 7;



const byte LASER_PIN = 8;



const byte ENCODER_A = 9;

const byte ENCODER_B = 10;



// ============================================================

// IR RECEIVERS

// ============================================================



IRrecv centreReceiver(CENTRE_SENSOR);

IRrecv rightReceiver(RIGHT_SENSOR);



// ============================================================

// GENERAL SETTINGS

// ============================================================



const bool REVERSE_MOTOR = false;



// Search direction, using the same convention as the original code:

// -1 = motorLeftPWM(), +1 = motorRightPWM()

const int SEARCH_DIRECTION = -1;



// Shorter than the old 60 ms so a stale centre signal cannot survive

// for long while the chassis is moving quickly.

const unsigned long IR_HOLD_MS = 45;



// LEFT + CENTRE + RIGHT must ALL remain active for this long before lock.

const unsigned long LOCK_CONFIRM_MS = 320;



// While locked, losing all target information for this long returns

// to fine alignment.

const unsigned long TARGET_LOST_MS = 130;



// Sensor zone must remain unchanged briefly before a precision

// correction is allowed.

const unsigned long FINE_ZONE_CONFIRM_MS = 28;

const unsigned long HOLD_ZONE_CONFIRM_MS = 22;



// ============================================================

// ENCODER / WHEEL SPEED

// ============================================================



const long COUNTS_PER_REV = 700;



long encoderCount = 0;

byte previousEncoderA = LOW;



long previousSpeedCount = 0;

unsigned long previousSpeedTime = 0;



float wheelRPM = 0.0;



// Sign of encoder RPM produced by motorLeftPWM().

// Learned automatically during the initial search.

int leftDriveEncoderSign = 0;



// ============================================================

// IR TIMING

// ============================================================



unsigned long lastLeftIR = 0;

unsigned long lastCentreIR = 0;

unsigned long lastRightIR = 0;

unsigned long targetLastSeen = 0;



// ============================================================

// SENSOR ZONE

//

// -2 = LEFT only

// -1 = LEFT + CENTRE

//  0 = CENTRE only (or all three)

// +1 = CENTRE + RIGHT

// +2 = RIGHT only

// 99 = no useful/ambiguous target

// ============================================================



int currentZone = 99;

int previousZone = 99;

unsigned long zoneSince = 0;



// ============================================================

// STATE MACHINE

// ============================================================



enum ControlState {

  SEARCHING,

  CAPTURE_APPROACH,

  CAPTURE_DAMPING,

  FINE_ALIGN,

  TARGET_LOCKED

};



ControlState state = SEARCHING;



unsigned long stateStartTime = 0;

unsigned long centreStableSince = 0;



// Direction in which the target was approached:

// -1 = from LEFT, +1 = from RIGHT

int approachSide = 0;

int lastKnownSide = 0;



unsigned long firstSideTime = 0;

unsigned long firstCentreTime = 0;



// ============================================================

// SEARCH TUNING

// ============================================================

//

// The old program allowed a large amount of wheel momentum to build.

// The satellite can still scan quickly, but the wheel is now ramped

// into a bounded search band rather than continuously driven hard.

// ============================================================



const float SEARCH_RPM_FAST_END = 95.0;

const float SEARCH_RPM_MID_END = 125.0;

const float SEARCH_RPM_LIMIT = 155.0;



const int SEARCH_POWER_FAST = 255;

const int SEARCH_POWER_MID = 175;

const int SEARCH_POWER_LOW = 95;



// ============================================================

// HIGH-SPEED CAPTURE TUNING

// ============================================================



// Pre-brake begins as soon as a side sensor sees the beacon.

// Exact values should be tuned from Serial telemetry.

int capturePreBrakePower = 0;

unsigned long capturePreBrakeDuration = 0;



int centreDampPower = 0;

unsigned long centreDampDuration = 0;



int counterDirection = 0;



bool centreSeenDuringCapture = false;

unsigned long captureObservationStart = 0;



// Maximum time to spend trying to damp one high-speed pass before

// handing over to the fine controller.

const unsigned long CAPTURE_TIMEOUT_MS = 850;



// Stable centre required during capture before handing to fine align.

const unsigned long CAPTURE_CENTRE_STABLE_MS = 140;



// ============================================================

// FINE ALIGNMENT TUNING

// ============================================================

//

// Near-centre corrections are intentionally tiny.

// Long settle times allow the suspended chassis to respond before

// another impulse is added.

// ============================================================



// LEFT/RIGHT only

const int FINE_OUTER_POWER = 62;

const unsigned long FINE_OUTER_PULSE_MS = 9;

const unsigned long FINE_OUTER_SETTLE_MS = 125;



// LEFT+CENTRE / CENTRE+RIGHT

const int FINE_INNER_POWER = 48;

const unsigned long FINE_INNER_PULSE_MS = 5;

const unsigned long FINE_INNER_SETTLE_MS = 145;



// ============================================================

// LOCKED HOLD TUNING

// ============================================================



const int HOLD_OUTER_POWER = 54;

const unsigned long HOLD_OUTER_PULSE_MS = 7;

const unsigned long HOLD_OUTER_SETTLE_MS = 145;



const int HOLD_INNER_POWER = 42;

const unsigned long HOLD_INNER_PULSE_MS = 4;

const unsigned long HOLD_INNER_SETTLE_MS = 165;



// ============================================================

// WHEEL-SPEED HOLD

//

// Once chassis motion has been arrested, the wheel may still be

// spinning quickly. If it is simply allowed to coast, gearbox/motor

// friction can slowly transfer that stored momentum back into the

// chassis. We therefore hold the wheel near the RPM produced by the

// last attitude correction.

// ============================================================



float holdWheelRPM = 0.0;

bool holdWheelSpeedValid = false;



const float MIN_WHEEL_HOLD_RPM = 18.0;

const float WHEEL_HOLD_DEADBAND_RPM = 4.0;



const int WHEEL_HOLD_MIN_POWER = 38;

const int WHEEL_HOLD_MAX_POWER = 82;



const unsigned long WHEEL_HOLD_PULSE_MS = 5;

const unsigned long WHEEL_HOLD_CHECK_MS = 18;



bool wheelHoldPulseActive = false;

unsigned long wheelHoldPulseEnd = 0;

unsigned long lastWheelHoldCheck = 0;



// ============================================================

// POINTING TORQUE PULSE MANAGER

// ============================================================



bool torquePulseActive = false;

unsigned long torquePulseEnd = 0;

unsigned long nextTorqueAllowed = 0;



// When true, capture the new wheel RPM when the pulse finishes and

// maintain that speed afterwards.

bool captureWheelSpeedAtPulseEnd = true;



// ============================================================

// SERIAL

// ============================================================



unsigned long lastPrint = 0;

const unsigned long PRINT_INTERVAL_MS = 120;



// ============================================================

// SMALL HELPERS

// ============================================================



float absFloat(float x) {

  return (x < 0.0f) ? -x : x;

}



int signOfFloat(float x) {

  if (x > 0.0f) return 1;

  if (x < 0.0f) return -1;

  return 0;

}



int signOfInt(int x) {

  if (x > 0) return 1;

  if (x < 0) return -1;

  return 0;

}



// ============================================================

// MOTOR CONTROL

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



void motorCoast() {

  analogWrite(MOTOR_IN1, 0);

  digitalWrite(MOTOR_IN1, LOW);

  digitalWrite(MOTOR_IN2, LOW);

}



void driveDirection(int direction, int power) {

  if (direction < 0) {

    motorLeftPWM(power);

  } else if (direction > 0) {

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

    if (digitalRead(ENCODER_B) != currentA) {

      encoderCount++;

    } else {

      encoderCount--;

    }

  }



  previousEncoderA = currentA;

}



void updateWheelSpeed(unsigned long now) {

  if (now - previousSpeedTime < 40) {

    return;

  }



  unsigned long dt = now - previousSpeedTime;

  long delta = encoderCount - previousSpeedCount;



  wheelRPM =

    ((float)delta / (float)COUNTS_PER_REV) * (60000.0f / (float)dt);



  previousSpeedCount = encoderCount;

  previousSpeedTime = now;

}



float getWheelAngle() {

  return ((float)encoderCount * 360.0f) / (float)COUNTS_PER_REV;

}



// ============================================================

// WHEEL DIRECTION LEARNING

// ============================================================



void learnLeftDriveEncoderSign() {

  // Search uses motorLeftPWM(). Once the encoder clearly reports motion,

  // remember which encoder sign corresponds to that motor direction.

  if (leftDriveEncoderSign == 0 && absFloat(wheelRPM) > 25.0f) {

    leftDriveEncoderSign = signOfFloat(wheelRPM);



    Serial.print(">>> Learned motorLeft encoder sign: ");

    Serial.println(leftDriveEncoderSign);

  }

}



int motorDirectionForWheelSign(int wheelSign) {

  // If mapping has not yet been learned, do not guess aggressively.

  if (leftDriveEncoderSign == 0 || wheelSign == 0) {

    return 0;

  }



  if (wheelSign == leftDriveEncoderSign) {

    return -1;  // motorLeftPWM()

  }



  return +1;  // motorRightPWM()

}



// ============================================================

// WHEEL-SPEED HOLD

// ============================================================



void disableWheelSpeedHold() {

  holdWheelSpeedValid = false;

  wheelHoldPulseActive = false;

}



void captureCurrentWheelSpeedForHold() {

  holdWheelRPM = wheelRPM;



  holdWheelSpeedValid =

    (leftDriveEncoderSign != 0) && (absFloat(holdWheelRPM) >= MIN_WHEEL_HOLD_RPM);



  if (holdWheelSpeedValid) {

    Serial.print(">>> Holding wheel at ");

    Serial.print(holdWheelRPM, 1);

    Serial.println(" RPM");

  }

}



void stopWheelHoldPulse() {

  wheelHoldPulseActive = false;

  motorCoast();

}



void maintainWheelSpeed(unsigned long now) {

  if (!holdWheelSpeedValid) {

    motorCoast();

    return;

  }



  // Finish a small maintenance pulse.

  if (wheelHoldPulseActive) {

    if (now - wheelHoldPulseEnd < 0x80000000UL && now >= wheelHoldPulseEnd) {

      stopWheelHoldPulse();

    }

    return;

  }



  if (now - lastWheelHoldCheck < WHEEL_HOLD_CHECK_MS) {

    return;

  }



  lastWheelHoldCheck = now;



  int targetSign = signOfFloat(holdWheelRPM);



  if (targetSign == 0) {

    motorCoast();

    return;

  }



  float targetMagnitude = absFloat(holdWheelRPM);



  // Signed current speed in the target wheel direction.

  float currentAlongTarget = wheelRPM * (float)targetSign;



  float speedError = targetMagnitude - currentAlongTarget;



  // If wheel is at/above target speed, do not add torque.

  // Let the tiny excess decay naturally.

  if (speedError <= WHEEL_HOLD_DEADBAND_RPM) {

    motorCoast();

    return;

  }



  int motorDirection = motorDirectionForWheelSign(targetSign);



  if (motorDirection == 0) {

    motorCoast();

    return;

  }



  int power =

    WHEEL_HOLD_MIN_POWER + (int)(speedError * 2.0f);



  power = constrain(

    power,

    WHEEL_HOLD_MIN_POWER,

    WHEEL_HOLD_MAX_POWER);



  driveDirection(motorDirection, power);



  wheelHoldPulseActive = true;

  wheelHoldPulseEnd = now + WHEEL_HOLD_PULSE_MS;

}



// ============================================================

// POINTING TORQUE PULSES

// ============================================================



void cancelWheelHoldPulse() {

  wheelHoldPulseActive = false;

}



void startTorquePulse(

  int direction,

  int power,

  unsigned long duration,

  unsigned long settleTime,

  unsigned long now,

  bool captureSpeedAfter = true) {



  if (torquePulseActive) {

    return;

  }



  cancelWheelHoldPulse();



  torquePulseActive = true;

  torquePulseEnd = now + duration;

  nextTorqueAllowed = torquePulseEnd + settleTime;

  captureWheelSpeedAtPulseEnd = captureSpeedAfter;



  driveDirection(direction, power);

}



void serviceTorquePulse(unsigned long now) {

  if (!torquePulseActive) {

    return;

  }



  if (now >= torquePulseEnd) {

    torquePulseActive = false;

    motorCoast();



    // The new wheel speed is the momentum state that corresponds to

    // the attitude correction we just made. Preserve it rather than

    // allowing it to decay back into chassis motion.

    if (captureWheelSpeedAtPulseEnd) {

      captureCurrentWheelSpeedForHold();

    }

  }

}



bool torqueReady(unsigned long now) {

  return !torquePulseActive && now >= nextTorqueAllowed;

}



// ============================================================

// SENSOR ZONE

// ============================================================



int calculateZone(bool L, bool C, bool R) {

  // STRICT alignment: all three receivers must see the beacon.
  if (L && C && R) return 0;

  // Directional overlap zones.
  if (L && C && !R) return -1;
  if (R && C && !L) return +1;

  // Outer zones.
  if (L && !C && !R) return -2;
  if (R && !C && !L) return +2;

  // Centre alone is near the target, but NOT aligned.
  if (C && !L && !R) return 98;

  // Ambiguous unusual overlap.
  if (L && R && !C) return 97;

  return 99;
}



void updateZone(int newZone, unsigned long now) {

  if (newZone != currentZone) {

    previousZone = currentZone;

    currentZone = newZone;

    zoneSince = now;

  }

}



bool zoneStableFor(unsigned long duration, unsigned long now) {

  return currentZone != 99 && (now - zoneSince >= duration);

}



// ============================================================

// STATE HELPERS

// ============================================================



const char* stateName(ControlState s) {

  switch (s) {

    case SEARCHING: return "SEARCH";

    case CAPTURE_APPROACH: return "CAPTURE_APPROACH";

    case CAPTURE_DAMPING: return "CAPTURE_DAMP";

    case FINE_ALIGN: return "FINE";

    case TARGET_LOCKED: return "LOCKED";

  }



  return "?";

}



void changeState(ControlState next, unsigned long now) {

  if (state == next) {

    return;

  }



  state = next;

  stateStartTime = now;



  Serial.print("***** STATE -> ");

  Serial.println(stateName(state));

}



// ============================================================

// SEARCH DRIVE

// ============================================================



void runSearchDrive() {

  disableWheelSpeedHold();



  float rpm = absFloat(wheelRPM);



  // If the wheel is still rotating strongly in the opposite encoder

  // direction from the normal search direction, use full power to

  // recover the intended search direction.

  if (leftDriveEncoderSign != 0 && absFloat(wheelRPM) > 20.0f && signOfFloat(wheelRPM) != leftDriveEncoderSign) {



    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_FAST);

    return;

  }



  if (rpm < SEARCH_RPM_FAST_END) {

    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_FAST);

  } else if (rpm < SEARCH_RPM_MID_END) {

    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_MID);

  } else if (rpm < SEARCH_RPM_LIMIT) {

    driveDirection(SEARCH_DIRECTION, SEARCH_POWER_LOW);

  } else {

    // At search speed, stop accelerating the wheel further.

    motorCoast();

  }

}



// ============================================================

// CAPTURE STRENGTH CALCULATION

// ============================================================



void choosePreBrakeFromWheelRPM() {

  float rpm = absFloat(wheelRPM);



  if (rpm >= 145.0f) {

    capturePreBrakePower = 255;

    capturePreBrakeDuration = 72;

  } else if (rpm >= 115.0f) {

    capturePreBrakePower = 225;

    capturePreBrakeDuration = 54;

  } else if (rpm >= 80.0f) {

    capturePreBrakePower = 180;

    capturePreBrakeDuration = 38;

  } else {

    capturePreBrakePower = 125;

    capturePreBrakeDuration = 22;

  }

}



void chooseCentreDamping(

  unsigned long crossingTimeMs,

  float rpmMagnitude) {



  // Very quick side->centre movement means the chassis still has

  // significant angular velocity, regardless of where the wheel RPM

  // happens to be after the pre-brake pulse.

  if (crossingTimeMs <= 70 || rpmMagnitude >= 145.0f) {

    centreDampPower = 255;

    centreDampDuration = 105;

  } else if (crossingTimeMs <= 125 || rpmMagnitude >= 115.0f) {

    centreDampPower = 225;

    centreDampDuration = 78;

  } else if (crossingTimeMs <= 210 || rpmMagnitude >= 80.0f) {

    centreDampPower = 180;

    centreDampDuration = 55;

  } else {

    centreDampPower = 130;

    centreDampDuration = 32;

  }

}



// ============================================================

// START TARGET CAPTURE

// ============================================================



void beginCaptureFromSide(

  int side,

  unsigned long now) {



  approachSide = side;

  lastKnownSide = side;



  firstSideTime = now;

  firstCentreTime = 0;



  centreSeenDuringCapture = false;

  centreStableSince = 0;



  // Opposite attitude command removes the existing chassis motion.

  counterDirection = -approachSide;



  choosePreBrakeFromWheelRPM();



  Serial.print(">>> SIDE ACQUIRE ");

  Serial.print(side < 0 ? "LEFT" : "RIGHT");

  Serial.print(" | wheel=");

  Serial.print(wheelRPM, 1);

  Serial.print(" RPM | pre-brake ");

  Serial.print(capturePreBrakePower);

  Serial.print(" / ");

  Serial.print(capturePreBrakeDuration);

  Serial.println(" ms");



  disableWheelSpeedHold();



  startTorquePulse(

    counterDirection,

    capturePreBrakePower,

    capturePreBrakeDuration,

    25,

    now,

    true);



  changeState(CAPTURE_APPROACH, now);

}



void beginCaptureFromCentre(unsigned long now) {

  // If centre is hit directly during the normal search, the body is

  // almost certainly travelling in the search direction.

  approachSide = SEARCH_DIRECTION;

  lastKnownSide = approachSide;



  firstSideTime = now;

  firstCentreTime = now;



  centreSeenDuringCapture = true;

  centreStableSince = 0;



  counterDirection = -approachSide;



  chooseCentreDamping(

    0,

    absFloat(wheelRPM));



  disableWheelSpeedHold();



  startTorquePulse(

    counterDirection,

    centreDampPower,

    centreDampDuration,

    45,

    now,

    true);



  captureObservationStart = now;



  changeState(CAPTURE_DAMPING, now);

}



// ============================================================

// SETUP

// ============================================================



void setup() {

  Serial.begin(115200);



  pinMode(MOTOR_IN1, OUTPUT);

  pinMode(MOTOR_IN2, OUTPUT);



  // Safe initial state.

  motorBrake();



  pinMode(LASER_PIN, OUTPUT);

  digitalWrite(LASER_PIN, LOW);



  pinMode(ENCODER_A, INPUT);

  pinMode(ENCODER_B, INPUT);



  previousEncoderA = digitalRead(ENCODER_A);

  previousSpeedTime = millis();



  IrReceiver.begin(

    LEFT_SENSOR,

    false);



  centreReceiver.begin(

    CENTRE_SENSOR,

    false);



  rightReceiver.begin(

    RIGHT_SENSOR,

    false);



  stateStartTime = millis();

  zoneSince = millis();



  Serial.println();

  Serial.println("================================================");

  Serial.println(" MQ SENTINEL PRECISION TRACKER V2");

  Serial.println("================================================");

  Serial.println("Early braking + centre damping + wheel-speed hold");

  Serial.println("Fine corrections use small impulse / long settle.");

  Serial.println();

}



// ============================================================

// MAIN LOOP

// ============================================================



void loop() {

  unsigned long now = millis();



  updateEncoder();

  updateWheelSpeed(now);



  // Finish any attitude torque pulse before state logic decides

  // whether another command is needed.

  serviceTorquePulse(now);



  // ==========================================================

  // READ IR RECEIVERS

  // ==========================================================



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



  bool L =

    lastLeftIR != 0 && now - lastLeftIR <= IR_HOLD_MS;



  bool C =

    lastCentreIR != 0 && now - lastCentreIR <= IR_HOLD_MS;



  bool R =

    lastRightIR != 0 && now - lastRightIR <= IR_HOLD_MS;




  // TRUE alignment exists only when all three sensors are active.
  bool aligned = L && C && R;

  updateZone(

    calculateZone(L, C, R),

    now);



  if (currentZone == -2 || currentZone == -1) {
    lastKnownSide = -1;
  } else if (currentZone == +1 || currentZone == +2) {
    lastKnownSide = +1;
  }



  // ==========================================================

  // SEARCHING

  // ==========================================================



  if (state == SEARCHING) {

    digitalWrite(LASER_PIN, LOW);



    centreStableSince = 0;

    firstSideTime = 0;

    firstCentreTime = 0;



    // Learn encoder direction while executing the known search command.

    learnLeftDriveEncoderSign();



    // Centre detection gets priority.

    if (C) {

      beginCaptureFromCentre(now);

    } else if (L && !R) {

      beginCaptureFromSide(-1, now);

    } else if (R && !L) {

      beginCaptureFromSide(+1, now);

    } else {

      runSearchDrive();

    }

  }



  // ==========================================================

  // CAPTURE APPROACH

  //

  // A side sensor has appeared. We are already applying the

  // opposite reaction-wheel torque BEFORE centre is reached.

  // We do NOT add more torque toward the target at this stage.

  // ==========================================================



  else if (state == CAPTURE_APPROACH) {

    digitalWrite(LASER_PIN, LOW);



    if (C && !centreSeenDuringCapture) {

      centreSeenDuringCapture = true;

      firstCentreTime = now;



      Serial.print(">>> CENTRE ENTERED after ");

      Serial.print(now - firstSideTime);

      Serial.println(" ms");

    }



    // Let the pre-brake pulse finish.

    if (torquePulseActive) {

      // Motor command is managed by the pulse manager.

    } else {

      // Preserve the new wheel momentum while observing the chassis.

      maintainWheelSpeed(now);



      if (centreSeenDuringCapture) {

        unsigned long crossingTime =

          firstCentreTime - firstSideTime;



        chooseCentreDamping(

          crossingTime,

          absFloat(wheelRPM));



        Serial.print(">>> CENTRE DAMP power=");

        Serial.print(centreDampPower);

        Serial.print(" duration=");

        Serial.print(centreDampDuration);

        Serial.println(" ms");



        startTorquePulse(

          counterDirection,

          centreDampPower,

          centreDampDuration,

          55,

          now,

          true);



        centreStableSince = 0;

        captureObservationStart = now;



        changeState(CAPTURE_DAMPING, now);

      } else {

        // If the early pre-brake stopped the chassis just before the

        // centre sensor, that is preferable to blasting through.

        // Fine alignment will now walk the final small angle.

        if (now - stateStartTime >= 230) {

          centreStableSince = 0;

          changeState(FINE_ALIGN, now);

        }

      }

    }

  }



  // ==========================================================

  // CAPTURE DAMPING

  //

  // Centre has been reached. Continue removing chassis angular

  // velocity, then preserve the resulting wheel speed.

  //

  // Sensor sequence is used as a coarse velocity sign:

  //

  // approach LEFT  -> centre -> RIGHT = overshoot

  // approach RIGHT -> centre -> LEFT  = overshoot

  //

  // If it returns to the original side, braking was too strong.

  // ==========================================================



  else if (state == CAPTURE_DAMPING) {

    digitalWrite(LASER_PIN, LOW);



    if (C && firstCentreTime == 0) {

      firstCentreTime = now;

    }



    if (torquePulseActive) {

      // Pulse manager owns motor.

    } else {

      maintainWheelSpeed(now);



      int zoneSign = 0;



      if (currentZone == -2 || currentZone == -1) zoneSign = -1;
      else if (currentZone == +1 || currentZone == +2) zoneSign = +1;



      // --------------------------------------------------------

      // ALL THREE active: true centre capture. Do not immediately "lock".

      // --------------------------------------------------------

      if (currentZone == 0) {

        if (centreStableSince == 0) {

          centreStableSince = now;

        }



        if (now - centreStableSince >= CAPTURE_CENTRE_STABLE_MS) {

          Serial.println(">>> ALL 3 SENSORS captured - entering fine alignment");

          changeState(FINE_ALIGN, now);

        }

      }



      // --------------------------------------------------------

      // Opposite side after centre = still travelling through.

      // Add another short counter-torque impulse.

      // --------------------------------------------------------

      else if (

        zoneSign != 0 && approachSide != 0 && zoneSign == -approachSide && zoneStableFor(20, now) && torqueReady(now)) {

        centreStableSince = 0;



        Serial.println(">>> OVERSHOOT still moving through centre");



        startTorquePulse(

          counterDirection,

          185,

          18,

          85,

          now,

          true);

      }



      // --------------------------------------------------------

      // Back on original approach side after centre = the last

      // counter-torque reversed the chassis too far.

      // Use a much smaller impulse back toward centre.

      // --------------------------------------------------------

      else if (

        centreSeenDuringCapture && zoneSign != 0 && approachSide != 0 && zoneSign == approachSide && zoneStableFor(28, now) && torqueReady(now)) {

        centreStableSince = 0;



        Serial.println(">>> Capture reversed slightly - gentle recovery");



        startTorquePulse(

          approachSide,

          95,

          7,

          120,

          now,

          true);

      }



      else {

        centreStableSince = 0;

      }



      if (now - stateStartTime >= CAPTURE_TIMEOUT_MS) {

        Serial.println(">>> Capture timeout -> fine alignment");

        changeState(FINE_ALIGN, now);

      }

    }

  }



  // ==========================================================

  // FINE ALIGN

  //

  // Small impulse + long observation. The old controller used

  // corrections that were too frequent, producing visible hunting.

  // ==========================================================



  else if (state == FINE_ALIGN) {

    digitalWrite(LASER_PIN, LOW);



    if (torquePulseActive) {

      // Pulse manager owns motor.

    } else {

      // Between pointing corrections preserve wheel speed so stored

      // flywheel momentum does not leak back into chassis motion.

      maintainWheelSpeed(now);



      // --------------------------------------------------------

      // ALL THREE SENSORS ACTIVE - TRUE ALIGNMENT

      // --------------------------------------------------------

      if (currentZone == 0) {

        if (centreStableSince == 0) {

          centreStableSince = now;

        }



        if (

          now - centreStableSince >= LOCK_CONFIRM_MS && !wheelHoldPulseActive) {

          changeState(TARGET_LOCKED, now);

        }

      }



      // --------------------------------------------------------
      // CENTRE ONLY (NOT ALIGNED)
      //
      // Never lock or fire here. If centre-only persists, make a
      // very small move toward the last known side to find the
      // overlap where LEFT + CENTRE + RIGHT are all active.
      // --------------------------------------------------------

      else if (
        currentZone == 98 &&
        zoneStableFor(90, now)) {

        centreStableSince = 0;

        if (lastKnownSide != 0 && torqueReady(now)) {
          startTorquePulse(
            lastKnownSide,
            38,
            4,
            180,
            now,
            true);
        }
      }


      // --------------------------------------------------------

      // LEFT + CENTRE / CENTRE + RIGHT

      // Very small correction.

      // --------------------------------------------------------

      else if (

        (currentZone == -1 || currentZone == +1) && zoneStableFor(FINE_ZONE_CONFIRM_MS, now)) {

        centreStableSince = 0;



        if (torqueReady(now)) {

          int direction = signOfInt(currentZone);



          startTorquePulse(

            direction,

            FINE_INNER_POWER,

            FINE_INNER_PULSE_MS,

            FINE_INNER_SETTLE_MS,

            now,

            true);

        }

      }



      // --------------------------------------------------------

      // LEFT only / RIGHT only

      // Still small, but slightly larger than near-centre pulse.

      // --------------------------------------------------------

      else if (

        (currentZone == -2 || currentZone == +2) && zoneStableFor(FINE_ZONE_CONFIRM_MS, now)) {

        centreStableSince = 0;



        if (torqueReady(now)) {

          int direction = signOfInt(currentZone);



          startTorquePulse(

            direction,

            FINE_OUTER_POWER,

            FINE_OUTER_PULSE_MS,

            FINE_OUTER_SETTLE_MS,

            now,

            true);

        }

      }



      else {

        centreStableSince = 0;



        // If signal flickers out briefly, maintain the wheel momentum

        // and WAIT rather than immediately kicking the chassis.

        //

        // Only after a longer loss do we gently move toward the last

        // known target side.

        if (

          currentZone == 99 && targetLastSeen != 0 && now - targetLastSeen > 180 && now - targetLastSeen <= 950 && lastKnownSide != 0 && torqueReady(now)) {

          startTorquePulse(

            lastKnownSide,

            48,

            5,

            160,

            now,

            true);

        }



        // Fully lost target -> return to bounded search.

        if (

          targetLastSeen != 0 && now - targetLastSeen > 1200) {

          disableWheelSpeedHold();

          approachSide = 0;

          firstSideTime = 0;

          firstCentreTime = 0;

          lastKnownSide = 0;



          motorCoast();



          changeState(SEARCHING, now);

        }

      }

    }

  }



  // ==========================================================

  // TARGET LOCKED

  //

  // Centre-only does NOT mean "switch motor off".

  // The flywheel speed achieved at capture is maintained so motor/

  // gearbox drag cannot slowly hand its momentum back to the chassis.

  // ==========================================================



  else if (state == TARGET_LOCKED) {



    if (torquePulseActive) {

      digitalWrite(LASER_PIN, LOW);

    } else {

      maintainWheelSpeed(now);



      // --------------------------------------------------------

      // PERFECT CENTRE = LEFT + CENTRE + RIGHT ALL ACTIVE

      // --------------------------------------------------------

      if (currentZone == 0) {

        if (centreStableSince == 0) {

          centreStableSince = now;

        }



        // Hard alignment gate: laser cannot fire unless L, C and R
        // are simultaneously active.
        digitalWrite(LASER_PIN, aligned ? HIGH : LOW);

      }



      // --------------------------------------------------------
      // CENTRE-ONLY / AMBIGUOUS IS NOT ALIGNED
      // --------------------------------------------------------

      else if (
        (currentZone == 98 || currentZone == 97) &&
        zoneStableFor(HOLD_ZONE_CONFIRM_MS, now)) {

        digitalWrite(LASER_PIN, LOW);
        centreStableSince = 0;
        changeState(FINE_ALIGN, now);
      }


      // --------------------------------------------------------

      // SMALL LEFT/RIGHT DRIFT NEAR CENTRE

      // --------------------------------------------------------

      else if (

        (currentZone == -1 || currentZone == +1) && zoneStableFor(HOLD_ZONE_CONFIRM_MS, now)) {

        digitalWrite(LASER_PIN, LOW);

        centreStableSince = 0;



        if (torqueReady(now)) {

          int direction = signOfInt(currentZone);



          startTorquePulse(

            direction,

            HOLD_INNER_POWER,

            HOLD_INNER_PULSE_MS,

            HOLD_INNER_SETTLE_MS,

            now,

            true);

        }

      }



      // --------------------------------------------------------

      // LARGER DRIFT

      // --------------------------------------------------------

      else if (

        (currentZone == -2 || currentZone == +2) && zoneStableFor(HOLD_ZONE_CONFIRM_MS, now)) {

        digitalWrite(LASER_PIN, LOW);

        centreStableSince = 0;



        if (torqueReady(now)) {

          int direction = signOfInt(currentZone);



          startTorquePulse(

            direction,

            HOLD_OUTER_POWER,

            HOLD_OUTER_PULSE_MS,

            HOLD_OUTER_SETTLE_MS,

            now,

            true);

        }

      }



      else {

        digitalWrite(LASER_PIN, LOW);

        centreStableSince = 0;



        if (

          targetLastSeen != 0 && now - targetLastSeen > TARGET_LOST_MS) {

          changeState(FINE_ALIGN, now);

        }

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



    Serial.print(" | ALIGN:");

    Serial.print(aligned ? "YES-ALL3" : "NO");



    Serial.print(" | ZONE:");

    if (currentZone == 99) {
      Serial.print("NONE");
    } else if (currentZone == 98) {
      Serial.print("CENTRE_ONLY");
    } else if (currentZone == 97) {
      Serial.print("LEFT_RIGHT_NO_CENTRE");
    } else if (currentZone == 0) {
      Serial.print("ALL3_ALIGNED");
    } else {
      Serial.print(currentZone);
    }



    Serial.print(" | RPM:");

    Serial.print(wheelRPM, 1);



    Serial.print(" | HOLD_RPM:");

    if (holdWheelSpeedValid) {

      Serial.print(holdWheelRPM, 1);

    } else {

      Serial.print("---");

    }



    Serial.print(" | ENC:");

    Serial.print(encoderCount);



    Serial.print(" | WHEEL:");

    Serial.print(getWheelAngle(), 1);

    Serial.print("deg");



    Serial.print(" | APPROACH:");

    if (approachSide < 0) {

      Serial.print("LEFT");

    } else if (approachSide > 0) {

      Serial.print("RIGHT");

    } else {

      Serial.print("NONE");

    }



    Serial.print(" | PULSE:");

    Serial.print(torquePulseActive ? "ON" : "OFF");



    Serial.print(" | LASER:");

    Serial.println(

      digitalRead(LASER_PIN) ? "ON" : "OFF");

  }

}



// ============================================================

// IRREMOTE MULTIPLE RECEIVER INTERRUPT HANDLER

// ============================================================



void UserIRReceiveTimerInterruptHandler() {

  centreReceiver.ReceiveInterruptHandler();

  rightReceiver.ReceiveInterruptHandler();

}
