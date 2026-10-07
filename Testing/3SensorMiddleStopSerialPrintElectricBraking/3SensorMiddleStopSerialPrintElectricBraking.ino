// ============================================================
// MQ SENTINEL - PRECISION 3-SENSOR IR TRACKING CONTROLLER
// Arduino Nano
//
// D2  = Left IR
// D3  = Centre IR
// D4  = Right IR
//
// D6  = Motor IN1 (PWM capable)
// D7  = Motor IN2
//
// D8  = Laser
//
// D9  = Encoder A
// D10 = Encoder B
//
// CONTROL:
//   Fast search
//       -> side sensor = early warning
//       -> aggressive deceleration
//       -> centre capture
//       -> brake
//       -> fine correction
//       -> stable centre
//       -> laser ON
//
// IMPORTANT:
// D6 is PWM capable on the Nano.
// D7 is not PWM capable.
// The motor functions below account for this.
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

const byte MOTOR_IN1 = 6;     // PWM
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
// IR SETTINGS
// ============================================================

// IMPORTANT:
// Your old value was 250 ms.
//
// That is far too long for accurate centre tracking because the
// satellite can rotate substantially during 250 ms.
//
// Start testing around 60 ms.
// If detection flickers, increase toward 80-100 ms.

const unsigned long IR_HOLD_MS = 60;


// ============================================================
// MOTOR DIRECTION
// ============================================================

const bool REVERSE_MOTOR = false;


// ============================================================
// ENCODER
// ============================================================

const long COUNTS_PER_REV = 700;

long encoderCount = 0;

byte previousEncoderA = LOW;


// ============================================================
// WHEEL SPEED ESTIMATION
// ============================================================

long previousSpeedCount = 0;

unsigned long previousSpeedTime = 0;

float wheelRPM = 0.0;


// ============================================================
// IR DETECTION TIMES
// ============================================================

unsigned long lastLeftIR   = 0;
unsigned long lastCentreIR = 0;
unsigned long lastRightIR  = 0;


// ============================================================
// CONTROLLER STATES
// ============================================================

enum ControlState {

    SEARCHING,

    APPROACHING,

    CENTRE_BRAKE,

    FINE_ALIGN,

    TARGET_LOCKED

};

ControlState state = SEARCHING;


// ============================================================
// MOTION MEMORY
// ============================================================

// Remembers which side of the target we came from.
//
// -1 = target was LEFT
//  0 = unknown
// +1 = target was RIGHT

int lastTargetSide = 0;


// Used to detect if we shot through the target.

int previousTargetSide = 0;


// ============================================================
// TIMERS
// ============================================================

unsigned long stateStartTime = 0;

unsigned long centreStableSince = 0;

unsigned long lastCorrectionTime = 0;

unsigned long lastPrint = 0;


// ============================================================
// TUNING PARAMETERS
// ============================================================

// ------------------------------------------------------------
// SEARCH
// ------------------------------------------------------------

// Maximum reaction-wheel RPM during acquisition.
//
// This prevents the flywheel from accelerating indefinitely.
//
// Start around 170.
// Increase after accuracy is working.

const float MAX_SEARCH_RPM = 170.0;


// Full search power.

const int SEARCH_POWER = 255;


// Reduced search power once wheel is moving quickly.

const int SEARCH_CRUISE_POWER = 150;


// ------------------------------------------------------------
// APPROACH
// ------------------------------------------------------------

// Side sensor has detected target.
// We no longer want maximum acceleration.

const int APPROACH_POWER = 110;


// ------------------------------------------------------------
// FINE ALIGNMENT
// ------------------------------------------------------------

// Low-power correction pulses.

const int FINE_POWER = 75;


// Duration of one correction pulse.

const unsigned long FINE_PULSE_MS = 35;


// Wait between correction pulses.
//
// This is VERY important.
//
// The satellite needs time to physically respond before another
// correction is commanded.

const unsigned long FINE_SETTLE_MS = 45;


// ------------------------------------------------------------
// CENTRE BRAKING
// ------------------------------------------------------------

// Initial hard electrical braking when centre first appears.

const unsigned long CENTRE_BRAKE_MS = 100;


// ------------------------------------------------------------
// TARGET LOCK
// ------------------------------------------------------------

// Centre must remain valid this long before laser fires.

const unsigned long LOCK_CONFIRM_MS = 200;


// If centre disappears while locked, immediately disable laser.

const unsigned long LOCK_LOST_MS = 70;


// ============================================================
// MOTOR CONTROL
// ============================================================
//
// D6 supports PWM.
// D7 does not.
//
// speed:
// 0   = no drive
// 255 = maximum drive
//
// ============================================================


void motorLeftPWM(int speed) {

    speed = constrain(speed, 0, 255);

    if (!REVERSE_MOTOR) {

        // IN1 LOW
        // IN2 HIGH
        //
        // Because IN2 is permanently HIGH in this direction,
        // PWM must be inverted on IN1.
        //
        // 0 PWM command -> IN1 HIGH -> brake
        // full command  -> IN1 LOW  -> full drive

        digitalWrite(MOTOR_IN2, HIGH);

        analogWrite(
            MOTOR_IN1,
            255 - speed
        );

    }

    else {

        digitalWrite(MOTOR_IN2, LOW);

        analogWrite(
            MOTOR_IN1,
            speed
        );
    }
}


void motorRightPWM(int speed) {

    speed = constrain(speed, 0, 255);

    if (!REVERSE_MOTOR) {

        digitalWrite(MOTOR_IN2, LOW);

        analogWrite(
            MOTOR_IN1,
            speed
        );

    }

    else {

        digitalWrite(MOTOR_IN2, HIGH);

        analogWrite(
            MOTOR_IN1,
            255 - speed
        );
    }
}


// ============================================================
// ELECTRICAL BRAKE
// ============================================================

void motorBrake() {

    // Disable PWM first.

    analogWrite(MOTOR_IN1, 255);

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, HIGH);
}


// ============================================================
// ENCODER
// ============================================================

void updateEncoder() {

    byte currentA = digitalRead(ENCODER_A);

    // Rising edge

    if (currentA == HIGH &&
        previousEncoderA == LOW) {

        if (digitalRead(ENCODER_B) != currentA) {

            encoderCount++;

        }

        else {

            encoderCount--;

        }
    }

    previousEncoderA = currentA;
}


// ============================================================
// WHEEL SPEED
// ============================================================

void updateWheelSpeed(unsigned long now) {

    // Calculate speed every 50 ms.

    if (now - previousSpeedTime < 50)
        return;


    unsigned long dt =
        now - previousSpeedTime;


    long difference =
        encoderCount - previousSpeedCount;


    wheelRPM =
        (
            (float)difference /
            (float)COUNTS_PER_REV
        )
        *
        (
            60000.0 /
            (float)dt
        );


    previousSpeedCount = encoderCount;

    previousSpeedTime = now;
}


// ============================================================
// ANGLE
// ============================================================

float getWheelAngle() {

    return
        (encoderCount * 360.0)
        /
        COUNTS_PER_REV;
}


// ============================================================
// STATE CHANGE
// ============================================================

void changeState(
    ControlState newState,
    unsigned long now
) {

    if (state == newState)
        return;


    state = newState;

    stateStartTime = now;


    Serial.print("***** STATE -> ");


    switch (state) {

        case SEARCHING:
            Serial.println("SEARCHING");
            break;

        case APPROACHING:
            Serial.println("APPROACHING");
            break;

        case CENTRE_BRAKE:
            Serial.println("CENTRE BRAKE");
            break;

        case FINE_ALIGN:
            Serial.println("FINE ALIGN");
            break;

        case TARGET_LOCKED:
            Serial.println("TARGET LOCKED");
            break;
    }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(115200);


    // --------------------------------------------------------
    // Motor
    // --------------------------------------------------------

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    motorBrake();


    // --------------------------------------------------------
    // Laser
    // --------------------------------------------------------

    pinMode(LASER_PIN, OUTPUT);

    digitalWrite(
        LASER_PIN,
        LOW
    );


    // --------------------------------------------------------
    // Encoder
    // --------------------------------------------------------

    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);

    previousEncoderA =
        digitalRead(ENCODER_A);


    previousSpeedTime =
        millis();


    // --------------------------------------------------------
    // IR receivers
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // Start
    // --------------------------------------------------------

    stateStartTime = millis();


    Serial.println();
    Serial.println("========================================");
    Serial.println(" MQ SENTINEL PRECISION TRACKER");
    Serial.println("========================================");

    Serial.println();

    Serial.println("D2  LEFT IR");
    Serial.println("D3  CENTRE IR");
    Serial.println("D4  RIGHT IR");

    Serial.println();

    Serial.println("Precision tracking enabled.");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

    unsigned long now = millis();


    // ========================================================
    // ENCODER
    // ========================================================

    updateEncoder();

    updateWheelSpeed(now);


    // ========================================================
    // IR RECEIVERS
    // ========================================================

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


    // ========================================================
    // DETECTION STATES
    // ========================================================

    bool leftDetected =

        lastLeftIR != 0 &&

        now - lastLeftIR <= IR_HOLD_MS;


    bool centreDetected =

        lastCentreIR != 0 &&

        now - lastCentreIR <= IR_HOLD_MS;


    bool rightDetected =

        lastRightIR != 0 &&

        now - lastRightIR <= IR_HOLD_MS;


    // ========================================================
    // ESTIMATE TARGET DIRECTION
    // ========================================================

    int targetSide = 0;


    // Strong left error

    if (
        leftDetected &&
        !centreDetected &&
        !rightDetected
    ) {

        targetSide = -1;
    }


    // Slight left error

    else if (
        leftDetected &&
        centreDetected &&
        !rightDetected
    ) {

        targetSide = -1;
    }


    // Strong right error

    else if (
        rightDetected &&
        !centreDetected &&
        !leftDetected
    ) {

        targetSide = 1;
    }


    // Slight right error

    else if (
        rightDetected &&
        centreDetected &&
        !leftDetected
    ) {

        targetSide = 1;
    }


    if (targetSide != 0) {

        lastTargetSide =
            targetSide;
    }


    // ========================================================
    // OVERSHOOT DETECTION
    // ========================================================

    bool crossedTarget = false;


    if (
        previousTargetSide != 0 &&
        targetSide != 0 &&
        previousTargetSide != targetSide
    ) {

        crossedTarget = true;


        Serial.println(
            ">>> TARGET CROSSING / OVERSHOOT DETECTED"
        );
    }


    if (targetSide != 0) {

        previousTargetSide =
            targetSide;
    }


    // ========================================================
    // STATE MACHINE
    // ========================================================


    // --------------------------------------------------------
    // SEARCHING
    // --------------------------------------------------------

    if (state == SEARCHING) {

        digitalWrite(
            LASER_PIN,
            LOW
        );


        // Centre appeared unexpectedly.
        //
        // Immediately stop accelerating.

        if (centreDetected) {

            motorBrake();

            centreStableSince = now;

            changeState(
                CENTRE_BRAKE,
                now
            );
        }


        // SIDE SENSOR = EARLY WARNING
        //
        // We are approaching the target.
        //
        // Immediately leave maximum-speed search.

        else if (
            leftDetected ||
            rightDetected
        ) {

            motorBrake();

            changeState(
                APPROACHING,
                now
            );
        }


        else {

            // Do NOT accelerate wheel forever.

            if (
                abs(wheelRPM) <
                MAX_SEARCH_RPM
            ) {

                motorLeftPWM(
                    SEARCH_POWER
                );
            }

            else {

                // Reduce drive substantially once
                // wheel reaches search speed.

                motorLeftPWM(
                    SEARCH_CRUISE_POWER
                );
            }
        }
    }


    // --------------------------------------------------------
    // APPROACHING
    // --------------------------------------------------------

    else if (state == APPROACHING) {

        digitalWrite(
            LASER_PIN,
            LOW
        );


        // Centre reached.

        if (centreDetected) {

            motorBrake();

            centreStableSince = now;

            changeState(
                CENTRE_BRAKE,
                now
            );
        }


        // Target left.

        else if (
            leftDetected &&
            !rightDetected
        ) {

            // MUCH less power than search.

            motorLeftPWM(
                APPROACH_POWER
            );
        }


        // Target right.

        else if (
            rightDetected &&
            !leftDetected
        ) {

            motorRightPWM(
                APPROACH_POWER
            );
        }


        else {

            // Signal disappeared.
            //
            // Do NOT immediately return to full search.
            //
            // First stop the wheel and enter fine alignment.

            motorBrake();

            changeState(
                FINE_ALIGN,
                now
            );
        }
    }


    // --------------------------------------------------------
    // CENTRE BRAKE
    // --------------------------------------------------------

    else if (state == CENTRE_BRAKE) {

        digitalWrite(
            LASER_PIN,
            LOW
        );


        // HARD electrical brake.

        motorBrake();


        // Give the mechanical system time to settle.

        if (
            now - stateStartTime >=
            CENTRE_BRAKE_MS
        ) {

            changeState(
                FINE_ALIGN,
                now
            );
        }
    }


    // --------------------------------------------------------
    // FINE ALIGN
    // --------------------------------------------------------

    else if (state == FINE_ALIGN) {

        digitalWrite(
            LASER_PIN,
            LOW
        );


        // ====================================================
        // PERFECT CENTRE
        // ====================================================

        if (
            centreDetected &&
            !leftDetected &&
            !rightDetected
        ) {

            motorBrake();


            if (centreStableSince == 0) {

                centreStableSince = now;
            }


            // Centre has remained stable long enough.

            if (
                now - centreStableSince >=
                LOCK_CONFIRM_MS
            ) {

                changeState(
                    TARGET_LOCKED,
                    now
                );
            }
        }


        // ====================================================
        // CENTRE + LEFT
        //
        // Very small error.
        // ====================================================

        else if (
            centreDetected &&
            leftDetected &&
            !rightDetected
        ) {

            centreStableSince = 0;


            if (
                now - lastCorrectionTime >=
                FINE_SETTLE_MS
            ) {

                motorLeftPWM(
                    FINE_POWER
                );


                delay(
                    FINE_PULSE_MS
                );


                motorBrake();


                lastCorrectionTime =
                    millis();
            }
        }


        // ====================================================
        // CENTRE + RIGHT
        // ====================================================

        else if (
            centreDetected &&
            rightDetected &&
            !leftDetected
        ) {

            centreStableSince = 0;


            if (
                now - lastCorrectionTime >=
                FINE_SETTLE_MS
            ) {

                motorRightPWM(
                    FINE_POWER
                );


                delay(
                    FINE_PULSE_MS
                );


                motorBrake();


                lastCorrectionTime =
                    millis();
            }
        }


        // ====================================================
        // LEFT ONLY
        // ====================================================

        else if (
            leftDetected &&
            !rightDetected
        ) {

            centreStableSince = 0;


            if (
                now - lastCorrectionTime >=
                FINE_SETTLE_MS
            ) {

                motorLeftPWM(
                    FINE_POWER
                );


                delay(
                    FINE_PULSE_MS
                );


                motorBrake();


                lastCorrectionTime =
                    millis();
            }
        }


        // ====================================================
        // RIGHT ONLY
        // ====================================================

        else if (
            rightDetected &&
            !leftDetected
        ) {

            centreStableSince = 0;


            if (
                now - lastCorrectionTime >=
                FINE_SETTLE_MS
            ) {

                motorRightPWM(
                    FINE_POWER
                );


                delay(
                    FINE_PULSE_MS
                );


                motorBrake();


                lastCorrectionTime =
                    millis();
            }
        }


        // ====================================================
        // NOTHING DETECTED
        //
        // We probably overshot.
        //
        // Use the LAST KNOWN target side rather than immediately
        // spinning at full speed.
        // ====================================================

        else {

            centreStableSince = 0;


            if (
                now - lastCorrectionTime >=
                FINE_SETTLE_MS
            ) {

                if (lastTargetSide < 0) {

                    motorLeftPWM(
                        FINE_POWER
                    );
                }

                else if (lastTargetSide > 0) {

                    motorRightPWM(
                        FINE_POWER
                    );
                }

                else {

                    motorLeftPWM(
                        FINE_POWER
                    );
                }


                delay(
                    FINE_PULSE_MS
                );


                motorBrake();


                lastCorrectionTime =
                    millis();
            }


            // If we've been unable to reacquire for a while,
            // return to fast search.

            if (
                now - stateStartTime >
                1500
            ) {

                previousTargetSide = 0;
                lastTargetSide = 0;

                changeState(
                    SEARCHING,
                    now
                );
            }
        }
    }


    // --------------------------------------------------------
    // TARGET LOCKED
    // --------------------------------------------------------

    else if (state == TARGET_LOCKED) {

        // Perfect centre.

        if (
            centreDetected &&
            !leftDetected &&
            !rightDetected
        ) {

            motorBrake();

            digitalWrite(
                LASER_PIN,
                HIGH
            );
        }


        // Drifting left.

        else if (
            leftDetected
        ) {

            digitalWrite(
                LASER_PIN,
                LOW
            );

            motorLeftPWM(
                FINE_POWER
            );

            delay(
                FINE_PULSE_MS
            );

            motorBrake();

            centreStableSince = 0;

            changeState(
                FINE_ALIGN,
                millis()
            );
        }


        // Drifting right.

        else if (
            rightDetected
        ) {

            digitalWrite(
                LASER_PIN,
                LOW
            );

            motorRightPWM(
                FINE_POWER
            );

            delay(
                FINE_PULSE_MS
            );

            motorBrake();

            centreStableSince = 0;

            changeState(
                FINE_ALIGN,
                millis()
            );
        }


        // Lost centre completely.

        else if (
            now - lastCentreIR >
            LOCK_LOST_MS
        ) {

            digitalWrite(
                LASER_PIN,
                LOW
            );

            motorBrake();

            centreStableSince = 0;

            changeState(
                FINE_ALIGN,
                now
            );
        }
    }


    // ========================================================
    // SERIAL MONITOR
    // ========================================================

    if (
        now - lastPrint >= 200
    ) {

        lastPrint = now;


        Serial.print("STATE: ");


        switch (state) {

            case SEARCHING:
                Serial.print("SEARCH");
                break;

            case APPROACHING:
                Serial.print("APPROACH");
                break;

            case CENTRE_BRAKE:
                Serial.print("BRAKE");
                break;

            case FINE_ALIGN:
                Serial.print("FINE");
                break;

            case TARGET_LOCKED:
                Serial.print("LOCKED");
                break;
        }


        Serial.print(" | IR: ");


        Serial.print(
            leftDetected ? "L" : "-"
        );

        Serial.print(
            centreDetected ? "C" : "-"
        );

        Serial.print(
            rightDetected ? "R" : "-"
        );


        Serial.print(" | RPM: ");

        Serial.print(
            wheelRPM,
            1
        );


        Serial.print(" | ENC: ");

        Serial.print(
            encoderCount
        );


        Serial.print(" | ANGLE: ");

        Serial.print(
            getWheelAngle(),
            1
        );


        Serial.print(" | LAST SIDE: ");

        if (lastTargetSide < 0)
            Serial.print("LEFT");

        else if (lastTargetSide > 0)
            Serial.print("RIGHT");

        else
            Serial.print("NONE");


        Serial.print(" | LASER: ");

        Serial.println(
            digitalRead(LASER_PIN)
            ? "ON"
            : "OFF"
        );
    }
}


// ============================================================
// MULTIPLE IR RECEIVER INTERRUPT HANDLER
// ============================================================

void UserIRReceiveTimerInterruptHandler() {

    centreReceiver.ReceiveInterruptHandler();

    rightReceiver.ReceiveInterruptHandler();
}