
/*
  MQ SENTINEL - TEST 6
  COAST VS BRAKE VS COUNTER-PULSE

  Board: Arduino Nano (ATmega328P)

  Runs without USB or Serial Monitor.

  D6  = Motor IN1
  D7  = Motor IN2
  D8  = Laser (disabled)
  D9  = Encoder A
  D10 = Encoder B
  D13 = Built-in LED

  One trial per power cycle.
  35 trials total.

  LED ON  = Forward acceleration
  LED OFF = Stopping method begins

  3 short flashes = Result saved.
  5 long flashes  = All trials complete.

  WARNING:
  Verify your motor driver's coast and brake
  truth table before operating the motor.
*/

#include <EEPROM.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// ============================================================
// PINS
// ============================================================

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte ENCODER_A = 9;
const byte ENCODER_B = 10;

const byte STATUS_LED = LED_BUILTIN;

// ============================================================
// MOTOR SETTINGS
// ============================================================

// Reverse both motor directions if necessary.

const bool REVERSE_MOTOR = false;

// Initial acceleration duration.

const unsigned int ACCELERATION_MS = 500;

// Estimate wheel speed over the final 100 ms
// of the acceleration period.

const unsigned int SPEED_SAMPLE_MS = 100;

// Counter-pulse durations.

const unsigned int REVERSE_TIMES[5] = {
  10, 20, 30, 40, 50
};

// ============================================================
// TEST SETTINGS
// ============================================================

const byte REPEATS = 5;

const byte TOTAL_TRIALS = 35;

// Preparation time for phone recording.

const unsigned long START_DELAY_MS = 5000;

// Maximum time observing the wheel after cutoff.

const unsigned long MAX_OBSERVATION_MS = 12000;

// No encoder pulses for this interval means
// the wheel is classified as stationary.

const unsigned long SETTLE_MS = 1500;

// ============================================================
// EEPROM
// ============================================================

// Test 6 uses its own identifier.
// Previous test results should be retrieved first.

const int HEADER_ADDRESS = 0;
const int COUNTER_ADDRESS = 2;
const int DATA_ADDRESS = 16;

const byte MAGIC_1 = 0xB6;
const byte MAGIC_2 = 0x6B;

const byte RECORD_MAGIC = 0xA6;

// ============================================================
// STOPPING METHODS
// ============================================================

enum StopMethod : byte {
  COAST = 0,
  BRAKE = 1,
  COUNTER_PULSE = 2
};

// ============================================================
// RESULT RECORD
// ============================================================

struct TestRecord {

  byte magic;

  byte method;

  unsigned int requestedReverseMs;

  unsigned int actualForwardMs;

  unsigned int actualReverseMs;

  // Speed measurement immediately before cutoff.

  long speedSampleCounts;

  unsigned int speedSampleDurationMs;

  // Encoder positions.

  long countAtCutoff;

  long countAfterStopping;

  long finalCount;

  // Encoder settling measurement.

  unsigned long observationMs;

  byte settled;
};

// ============================================================
// GLOBAL VARIABLES
// ============================================================

volatile long encoderCount = 0;

volatile byte previousEncoderA = LOW;

byte currentTrial = 0;

// ============================================================
// ENCODER INTERRUPT
// ============================================================

// Arduino Nano:
//
// D9  = PB1 / PCINT1
// D10 = PB2
//
// Count encoder A rising edges.
// Read encoder B to determine direction.

ISR(PCINT0_vect) {

  byte currentA =
      (PINB & _BV(PB1)) ? HIGH : LOW;

  if (
    currentA == HIGH &&
    previousEncoderA == LOW
  ) {

    byte currentB =
        (PINB & _BV(PB2)) ? HIGH : LOW;

    if (currentB != currentA) {

      encoderCount++;

    } else {

      encoderCount--;
    }
  }

  previousEncoderA = currentA;
}

// ============================================================
// ENCODER FUNCTIONS
// ============================================================

void initialiseEncoder() {

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  previousEncoderA =
      digitalRead(ENCODER_A);

  // Enable pin-change interrupt group 0.

  PCICR |= _BV(PCIE0);

  // Enable D9 / PCINT1.

  PCMSK0 |= _BV(PCINT1);
}

long readEncoder() {

  long count;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    count = encoderCount;
  }

  return count;
}

void resetEncoder() {

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    encoderCount = 0;

    previousEncoderA =
        digitalRead(ENCODER_A);
  }
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void motorForward() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

  } else {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }
}

void motorReverse() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

  } else {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  }
}

void motorCoast() {

  // Verify LOW/LOW = coast on your driver.

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void motorBrake() {

  // Verify HIGH/HIGH = electrical braking.

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}

// ============================================================
// LED
// ============================================================

void flashLED(
  byte flashes,
  unsigned int onTime,
  unsigned int offTime
) {

  for (byte i = 0; i < flashes; i++) {

    digitalWrite(STATUS_LED, HIGH);
    delay(onTime);

    digitalWrite(STATUS_LED, LOW);
    delay(offTime);
  }
}

// ============================================================
// EEPROM INITIALISATION
// ============================================================

void initialiseStorage() {

  bool valid =
      EEPROM.read(HEADER_ADDRESS) == MAGIC_1 &&
      EEPROM.read(HEADER_ADDRESS + 1) == MAGIC_2;

  if (!valid) {

    EEPROM.update(HEADER_ADDRESS, MAGIC_1);

    EEPROM.update(HEADER_ADDRESS + 1, MAGIC_2);

    EEPROM.update(COUNTER_ADDRESS, 0);
  }
}

// ============================================================
// SELECT TRIAL
// ============================================================

StopMethod getMethod(byte trial) {

  if (trial < 5) {
    return COAST;
  }

  if (trial < 10) {
    return BRAKE;
  }

  return COUNTER_PULSE;
}

unsigned int getReverseDuration(byte trial) {

  if (trial < 10) {
    return 0;
  }

  byte index = (trial - 10) / REPEATS;

  return REVERSE_TIMES[index];
}

// ============================================================
// SAVE RESULTS
// ============================================================

void saveResult(const TestRecord &result) {

  int address =
      DATA_ADDRESS +
      currentTrial * sizeof(TestRecord);

  EEPROM.put(address, result);

  // Advance only after saving.

  EEPROM.update(
    COUNTER_ADDRESS,
    currentTrial + 1
  );
}

// ============================================================
// RUN MOTOR TEST
// ============================================================

void runTrial() {

  StopMethod method =
      getMethod(currentTrial);

  unsigned int reverseDuration =
      getReverseDuration(currentTrial);

  // ----------------------------------------------------------
  // INITIAL CONDITIONS
  // ----------------------------------------------------------

  motorCoast();

  resetEncoder();

  TestRecord result;

  result.magic = RECORD_MAGIC;

  result.method = method;

  result.requestedReverseMs = reverseDuration;

  result.actualReverseMs = 0;

  result.speedSampleCounts = 0;

  result.speedSampleDurationMs = 0;

  result.countAfterStopping = 0;

  result.finalCount = 0;

  result.observationMs = 0;

  result.settled = 0;

  // ----------------------------------------------------------
  // START ACCELERATION
  // ----------------------------------------------------------

  digitalWrite(STATUS_LED, HIGH);

  unsigned long startUs = micros();

  motorForward();

  // ----------------------------------------------------------
  // WAIT UNTIL SPEED SAMPLE
  // ----------------------------------------------------------

  unsigned long sampleStartTargetUs =
      (unsigned long)(
        ACCELERATION_MS - SPEED_SAMPLE_MS
      ) * 1000UL;

  while (
    (unsigned long)(micros() - startUs)
    < sampleStartTargetUs
  ) {
    // Encoder counted by interrupt.
  }

  unsigned long sampleStartUs = micros();

  long sampleStartCount = readEncoder();

  // ----------------------------------------------------------
  // CONTINUE UNTIL MOTOR CUTOFF
  // ----------------------------------------------------------

  unsigned long accelerationUs =
      (unsigned long)ACCELERATION_MS * 1000UL;

  while (
    (unsigned long)(micros() - startUs)
    < accelerationUs
  ) {
    // Encoder counted by interrupt.
  }

  // ----------------------------------------------------------
  // APPLY SELECTED STOPPING METHOD
  // ----------------------------------------------------------

  // Switch motor outputs before logging data.

  if (method == COAST) {

    motorCoast();

  } else if (method == BRAKE) {

    motorBrake();

  } else {

    // Immediately begin reverse drive.

    motorReverse();
  }

  unsigned long cutoffUs = micros();

  // LED OFF marks beginning of stopping method.

  digitalWrite(STATUS_LED, LOW);

  // Capture encoder count at cutoff.

  long cutoffCount = readEncoder();

  result.countAtCutoff = cutoffCount;

  result.actualForwardMs =
      (unsigned int)(
        (cutoffUs - startUs) / 1000UL
      );

  result.speedSampleCounts =
      cutoffCount - sampleStartCount;

  result.speedSampleDurationMs =
      (unsigned int)(
        (cutoffUs - sampleStartUs) / 1000UL
      );

  // ----------------------------------------------------------
  // COUNTER-PULSE
  // ----------------------------------------------------------

  if (method == COUNTER_PULSE) {

    unsigned long reverseStartUs = cutoffUs;

    unsigned long requestedReverseUs =
        (unsigned long)reverseDuration * 1000UL;

    while (
      (unsigned long)(
        micros() - reverseStartUs
      ) < requestedReverseUs
    ) {
      // Encoder counted by interrupt.
    }

    // Remove motor drive after the reverse pulse.

    motorCoast();

    unsigned long reverseEndUs = micros();

    result.actualReverseMs =
        (unsigned int)(
          (reverseEndUs - reverseStartUs)
          / 1000UL
        );
  }

  // ----------------------------------------------------------
  // RECORD POSITION AFTER STOPPING ACTION
  // ----------------------------------------------------------

  // For coast and brake, this is close to cutoff.
  // For counter-pulse, it is the end of reverse drive.

  result.countAfterStopping = readEncoder();

  // ----------------------------------------------------------
  // OBSERVE RESIDUAL WHEEL MOVEMENT
  // ----------------------------------------------------------

  unsigned long observationStart = millis();

  unsigned long lastMovementTime =
      observationStart;

  long previousCount = readEncoder();

  while (
    millis() - observationStart <
    MAX_OBSERVATION_MS
  ) {

    unsigned long now = millis();

    long currentCount = readEncoder();

    if (currentCount != previousCount) {

      previousCount = currentCount;

      lastMovementTime = now;
    }

    if (
      now - lastMovementTime >= SETTLE_MS
    ) {

      result.settled = 1;

      break;
    }
  }

  result.observationMs =
      millis() - observationStart;

  result.finalCount = readEncoder();

  // ----------------------------------------------------------
  // SAVE RESULTS
  // ----------------------------------------------------------

  saveResult(result);

  // Three short flashes indicate completion.

  flashLED(3, 120, 120);

  // Coast after data collection.
  //
  // For the electrical-braking trial, braking is
  // maintained throughout the encoder observation.

  motorCoast();

  // Do not run another trial until power cycle.

  while (true) {
    delay(1000);
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);

  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(LASER_PIN, LOW);

  digitalWrite(STATUS_LED, LOW);

  motorCoast();

  initialiseEncoder();

  initialiseStorage();

  currentTrial =
      EEPROM.read(COUNTER_ADDRESS);

  // ----------------------------------------------------------
  // ALL TRIALS FINISHED
  // ----------------------------------------------------------

  if (currentTrial >= TOTAL_TRIALS) {

    flashLED(5, 500, 500);

    while (true) {

      motorCoast();

      delay(1000);
    }
  }

  // ----------------------------------------------------------
  // CAMERA PREPARATION
  // ----------------------------------------------------------

  delay(START_DELAY_MS);

  runTrial();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // One trial per power cycle.
}