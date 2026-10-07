
/*
  MQ SENTINEL - TEST 5
  MINIMUM EFFECTIVE MOTOR PULSE

  Arduino Nano / ATmega328P

  No Serial Monitor required during testing.

  D6  = Motor IN1
  D7  = Motor IN2
  D8  = Laser (disabled)
  D9  = Encoder A
  D10 = Encoder B
  D13 = Built-in LED

  7 durations x 2 directions x 5 repetitions
  = 70 trials.

  One trial per power cycle.

  Encoder results are saved to EEPROM.

  LED:
  Five-second preparation delay.
  ON  = Motor powered.
  OFF = Motor drive removed.
  Three flashes = Result saved.
  Five flashes = All trials finished.
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
// SETTINGS
// ============================================================

const bool REVERSE_MOTOR = false;

const unsigned int PULSE_TIMES[7] = {
  10, 20, 30, 40, 50, 75, 100
};

const byte NUM_DURATIONS = 7;
const byte REPEATS = 5;

const byte TOTAL_TRIALS =
    NUM_DURATIONS * REPEATS * 2;

const unsigned long START_DELAY_MS = 5000;

const unsigned long POST_RUN_MS = 2000;

// ============================================================
// EEPROM
// ============================================================

const int MAGIC_ADDRESS = 0;
const int COUNTER_ADDRESS = 2;
const int DATA_ADDRESS = 16;

const byte MAGIC_1 = 0xA5;
const byte MAGIC_2 = 0x5A;

// Record stores 10 bytes on ATmega328P.
// Trial number identifies duration, direction
// and repetition.

struct TestRecord {

  unsigned int actualTimeMs;

  long encoderAtCutoff;

  long encoderAfterObservation;
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

// D9  = PB1 / PCINT1
// D10 = PB2
//
// Count A rising edges and read B for direction.

ISR(PCINT0_vect) {

  byte currentA =
      (PINB & _BV(PB1)) ? HIGH : LOW;

  if (currentA == HIGH &&
      previousEncoderA == LOW) {

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

  PCICR |= _BV(PCIE0);

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

void motorCoast() {

  // Verify LOW/LOW = coast for your driver.

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

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
      EEPROM.read(MAGIC_ADDRESS) == MAGIC_1 &&
      EEPROM.read(MAGIC_ADDRESS + 1) == MAGIC_2;

  if (!valid) {

    EEPROM.update(MAGIC_ADDRESS, MAGIC_1);

    EEPROM.update(MAGIC_ADDRESS + 1, MAGIC_2);

    EEPROM.update(COUNTER_ADDRESS, 0);
  }
}

// ============================================================
// SAVE RESULT
// ============================================================

void saveResult(const TestRecord &result) {

  int address =
      DATA_ADDRESS +
      currentTrial * sizeof(TestRecord);

  EEPROM.put(address, result);

  // Advance only after saving the result.

  EEPROM.update(
    COUNTER_ADDRESS,
    currentTrial + 1
  );
}

// ============================================================
// RUN ONE TRIAL
// ============================================================

void runTrial() {

  // Trial order:
  //
  // 0-4   = 10 ms forward
  // 5-9   = 20 ms forward
  // ...
  // 30-34 = 100 ms forward
  //
  // 35-39 = 10 ms reverse
  // ...
  // 65-69 = 100 ms reverse

  bool reverseDirection =
      currentTrial >= 35;

  byte directionTrial =
      reverseDirection
      ? currentTrial - 35
      : currentTrial;

  byte durationIndex =
      directionTrial / REPEATS;

  unsigned int requestedTime =
      PULSE_TIMES[durationIndex];

  // ----------------------------------------------------------
  // RESET ENCODER
  // ----------------------------------------------------------

  motorCoast();

  resetEncoder();

  // ----------------------------------------------------------
  // MOTOR START
  // ----------------------------------------------------------

  // LED ON marks the start of the pulse.

  digitalWrite(STATUS_LED, HIGH);

  unsigned long startTime = micros();

  if (reverseDirection) {
    motorReverse();
  } else {
    motorForward();
  }

  // ----------------------------------------------------------
  // TIMED MOTOR PULSE
  // ----------------------------------------------------------

  unsigned long durationUs =
      (unsigned long)requestedTime * 1000UL;

  while (
    (unsigned long)(micros() - startTime)
    < durationUs
  ) {
    // Encoder is counted by interrupt.
  }

  // ----------------------------------------------------------
  // MOTOR CUTOFF
  // ----------------------------------------------------------

  motorCoast();

  unsigned long cutoffTime = micros();

  digitalWrite(STATUS_LED, LOW);

  long countAtCutoff = readEncoder();

  unsigned int actualTime =
      (unsigned int)(
        (cutoffTime - startTime) / 1000UL
      );

  // ----------------------------------------------------------
  // POST-RUN OBSERVATION
  // ----------------------------------------------------------

  // Continue monitoring the encoder for 2 seconds.
  // The motor remains in coast mode.

  delay(POST_RUN_MS);

  long countAfterObservation =
      readEncoder();

  // ----------------------------------------------------------
  // SAVE DATA
  // ----------------------------------------------------------

  TestRecord result;

  result.actualTimeMs = actualTime;

  result.encoderAtCutoff = countAtCutoff;

  result.encoderAfterObservation =
      countAfterObservation;

  saveResult(result);

  // Three flashes = EEPROM write completed.

  flashLED(3, 120, 120);

  // Remain stopped until next power cycle.

  motorCoast();

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
  // ALL TRIALS COMPLETE
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

  // ----------------------------------------------------------
  // EXECUTE TRIAL
  // ----------------------------------------------------------

  runTrial();
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // All test activity occurs in setup().
}