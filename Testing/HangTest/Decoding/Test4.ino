
/*
  ============================================================
  MQ SENTINEL - TEST 4
  COAST / INERTIA TEST

  Arduino Nano - standalone experiment

  VERSION A: Raw IR trigger
  VERSION B: Decoded IR trigger

  No Serial Monitor required during testing.

  The motor runs for a fixed acceleration period,
  then coasts while the encoder records residual motion.

  Five trials can be saved to EEPROM.

  LED ON  = Motor powered
  LED OFF = Motor drive removed
  ============================================================
*/

// ============================================================
// SELECT TEST VERSION
// ============================================================

// 0 = Raw HIGH/LOW IR detection
// 1 = IRremote decoded detection

#define USE_DECODED_IR 0

#if USE_DECODED_IR
#include <IRremote.hpp>
#endif

#include <EEPROM.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

const byte CENTRE_SENSOR = 3;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;

const byte ENCODER_A = 9;
const byte ENCODER_B = 10;

const byte STATUS_LED = LED_BUILTIN;

// ============================================================
// EXPERIMENT SETTINGS
// ============================================================

const bool REVERSE_MOTOR = false;

// Motor acceleration period.
const unsigned long ACCELERATION_MS = 1500;

// Speed estimation interval before cutoff.
const unsigned long SPEED_SAMPLE_MS = 200;

// Camera preparation time.
const unsigned long START_DELAY_MS = 5000;

// Maximum coasting observation duration.
const unsigned long MAX_COAST_MS = 20000;

// No encoder pulses for this interval:
// wheel considered stationary for this test.
const unsigned long SETTLE_MS = 1500;

// Verify this against the actual encoder.
const float COUNTS_PER_REV = 700.0;

// Number of saved trials.
const byte TOTAL_TRIALS = 5;

// ============================================================
// EEPROM STORAGE
// ============================================================

const int TRIAL_ADDRESS = 0;
const int RECORD_ADDRESS = 16;

const byte RECORD_MAGIC = 0xA5;

// Result codes.
const byte RESULT_SETTLED = 1;
const byte RESULT_TIMEOUT = 2;

// ============================================================
// ENCODER VARIABLES
// ============================================================

volatile long encoderCount = 0;

// Last observed state of encoder channel A.
volatile byte previousEncoderA = LOW;

// ============================================================
// TEST VARIABLES
// ============================================================

byte currentTrial = 0;

bool testStarted = false;
bool testComplete = false;

// ============================================================
// RESULT STRUCTURE
// ============================================================

struct TestRecord {

  byte magic;

  unsigned long requestedTimeMs;

  unsigned long actualTimeMs;

  long cutoffCount;

  long speedSampleCount;

  long finalCount;

  unsigned long coastTimeMs;

  unsigned long settlingTimeMs;

  byte resultCode;
};

// ============================================================
// ENCODER INTERRUPT
// ============================================================

// Arduino Nano / ATmega328P:
//
// D9  = PB1 / PCINT1
// D10 = PB2 / PCINT2
//
// Interrupt is enabled for channel A only.
// Direction is determined using channel B.

ISR(PCINT0_vect) {

  byte currentA = (PINB & _BV(PB1))
                  ? HIGH : LOW;

  if (currentA == HIGH &&
      previousEncoderA == LOW) {

    byte currentB = (PINB & _BV(PB2))
                    ? HIGH : LOW;

    if (currentB != currentA) {

      encoderCount++;

    } else {

      encoderCount--;
    }
  }

  previousEncoderA = currentA;
}

// ============================================================
// READ ENCODER SAFELY
// ============================================================

long readEncoder() {

  long count;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count = encoderCount;
  }

  return count;
}

// ============================================================
// RESET ENCODER
// ============================================================

void resetEncoder() {

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

    encoderCount = 0;

    previousEncoderA =
        digitalRead(ENCODER_A);
  }
}

// ============================================================
// INITIALISE ENCODER INTERRUPT
// ============================================================

void initialiseEncoder() {

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  previousEncoderA =
      digitalRead(ENCODER_A);

  // Enable pin-change interrupts for port B.
  PCICR |= _BV(PCIE0);

  // Enable interrupts for D9 only.
  PCMSK0 |= _BV(PCINT1);
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void motorLeft() {

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

  } else {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

  }
}

void motorCoast() {

  // LOW / LOW requests coast mode
  // for the intended IN1/IN2 driver configuration.
  //
  // Verify against your motor driver's truth table.

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

// ============================================================
// LED FEEDBACK
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
// IR TRIGGER
// ============================================================

bool beaconDetected() {

#if USE_DECODED_IR

  // Version B:
  // A successful IRremote decode triggers the experiment.

  if (IrReceiver.decode()) {

    IrReceiver.resume();

    return true;
  }

  return false;

#else

  // Version A:
  // Raw active-LOW detection.

  return digitalRead(CENTRE_SENSOR) == LOW;

#endif
}

// ============================================================
// SAVE RESULTS
// ============================================================

void saveResults(const TestRecord &result) {

  int address =
      RECORD_ADDRESS +
      currentTrial * sizeof(TestRecord);

  // Save completed measurement.
  EEPROM.put(address, result);

  // Advance trial counter only after saving.
  EEPROM.update(
    TRIAL_ADDRESS,
    currentTrial + 1
  );
}

// ============================================================
// RUN COAST / INERTIA TEST
// ============================================================

void runTest() {

  if (testStarted || testComplete) {
    return;
  }

  testStarted = true;

  // Motor is initially coasting and stationary.
  motorCoast();

  resetEncoder();

  // ----------------------------------------------------------
  // START MOTOR
  // ----------------------------------------------------------

  unsigned long startTime = millis();

  // LED indicates motor power.
  digitalWrite(STATUS_LED, HIGH);

  motorLeft();

  // ----------------------------------------------------------
  // ACCELERATE FLYWHEEL
  // ----------------------------------------------------------

  unsigned long sampleTime =
      ACCELERATION_MS - SPEED_SAMPLE_MS;

  while (millis() - startTime < sampleTime) {
    // Encoder continues counting via interrupt.
  }

  // Count 200 ms before intended cutoff.
  long speedSampleCount = readEncoder();

  // Continue acceleration until cutoff time.
  while (
    millis() - startTime < ACCELERATION_MS
  ) {
    // Encoder continues counting via interrupt.
  }

  // ----------------------------------------------------------
  // MOTOR CUTOFF
  // ----------------------------------------------------------

  motorCoast();

  unsigned long cutoffTime = millis();

  // LED OFF marks the cutoff in overhead video.
  digitalWrite(STATUS_LED, LOW);

  long cutoffCount = readEncoder();

  unsigned long actualTime =
      cutoffTime - startTime;

  // ----------------------------------------------------------
  // COASTING OBSERVATION
  // ----------------------------------------------------------

  long previousCount = cutoffCount;

  unsigned long lastMovementTime = cutoffTime;

  unsigned long settlingTime = 0;

  byte resultCode = RESULT_TIMEOUT;

  // Wait until the wheel stops moving or
  // the maximum observation period expires.

  while (
    millis() - cutoffTime < MAX_COAST_MS
  ) {

    unsigned long now = millis();

    long currentCount = readEncoder();

    // New encoder movement observed.
    if (currentCount != previousCount) {

      previousCount = currentCount;

      lastMovementTime = now;
    }

    // No encoder movement for SETTLE_MS.
    if (
      now - lastMovementTime >= SETTLE_MS
    ) {

      settlingTime =
          lastMovementTime - cutoffTime;

      resultCode = RESULT_SETTLED;

      break;
    }
  }

  // ----------------------------------------------------------
  // FINAL ENCODER MEASUREMENT
  // ----------------------------------------------------------

  long finalCount = readEncoder();

  unsigned long coastTime =
      millis() - cutoffTime;

  // ----------------------------------------------------------
  // CREATE RESULT RECORD
  // ----------------------------------------------------------

  TestRecord result;

  result.magic = RECORD_MAGIC;

  result.requestedTimeMs = ACCELERATION_MS;

  result.actualTimeMs = actualTime;

  result.cutoffCount = cutoffCount;

  result.speedSampleCount = speedSampleCount;

  result.finalCount = finalCount;

  result.coastTimeMs = coastTime;

  result.settlingTimeMs = settlingTime;

  result.resultCode = resultCode;

  // ----------------------------------------------------------
  // SAVE TO EEPROM
  // ----------------------------------------------------------

  saveResults(result);

  testComplete = true;

  // Three short flashes indicate completion.
  flashLED(3, 120, 120);

  // Motor remains in coast mode.
  motorCoast();

  // No further motor movement is commanded.
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);

  pinMode(STATUS_LED, OUTPUT);

  pinMode(CENTRE_SENSOR, INPUT);

  // Laser always disabled.
  digitalWrite(LASER_PIN, LOW);

  // Indicator initially off.
  digitalWrite(STATUS_LED, LOW);

  // Motor initially coasting.
  motorCoast();

  // Initialise encoder.
  initialiseEncoder();

#if USE_DECODED_IR

  IrReceiver.begin(
    CENTRE_SENSOR,
    false
  );

#endif

  // ----------------------------------------------------------
  // LOAD TRIAL NUMBER
  // ----------------------------------------------------------

  currentTrial =
      EEPROM.read(TRIAL_ADDRESS);

  // Initialise blank EEPROM.
  if (currentTrial == 0xFF) {

    currentTrial = 0;

    EEPROM.update(TRIAL_ADDRESS, 0);
  }

  // ----------------------------------------------------------
  // ALL TRIALS COMPLETE
  // ----------------------------------------------------------

  if (currentTrial >= TOTAL_TRIALS) {

    testComplete = true;

    // Five slow flashes = all trials completed.
    flashLED(5, 500, 500);

    return;
  }

  // ----------------------------------------------------------
  // INDICATE CURRENT TRIAL
  // ----------------------------------------------------------

  // 1 flash = Trial 1
  // 2 flashes = Trial 2
  // ...
  // 5 flashes = Trial 5

  flashLED(
    currentTrial + 1,
    350,
    350
  );

  // Allow time to prepare the overhead camera.
  delay(START_DELAY_MS);
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Do not repeat the experiment automatically.
  if (testComplete) {
    return;
  }

  // Wait for beacon trigger.
  if (!testStarted && beaconDetected()) {

    runTest();
  }
}