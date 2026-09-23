
/*
  MQ SENTINEL - TEST 5 RESULTS READER

  Upload AFTER the physical experiment.

  Serial Monitor: 115200 baud

  P = Print results again
  R = Reset trial counter to zero

  Resetting the counter does not erase records,
  but subsequent trials overwrite old results.
*/

#include <EEPROM.h>

const int MAGIC_ADDRESS = 0;
const int COUNTER_ADDRESS = 2;
const int DATA_ADDRESS = 16;

const byte MAGIC_1 = 0xA5;
const byte MAGIC_2 = 0x5A;

const byte TOTAL_TRIALS = 70;
const byte REPEATS = 5;

const unsigned int PULSE_TIMES[7] = {
  10, 20, 30, 40, 50, 75, 100
};

struct TestRecord {

  unsigned int actualTimeMs;

  long encoderAtCutoff;

  long encoderAfterObservation;
};

// ============================================================
// PRINT RESULTS
// ============================================================

void printResults() {

  bool valid =
      EEPROM.read(MAGIC_ADDRESS) == MAGIC_1 &&
      EEPROM.read(MAGIC_ADDRESS + 1) == MAGIC_2;

  if (!valid) {

    Serial.println("No Test 5 records found.");
    return;
  }

  byte completed =
      EEPROM.read(COUNTER_ADDRESS);

  if (completed > TOTAL_TRIALS) {

    Serial.println("Invalid trial counter.");
    return;
  }

  Serial.println();

  Serial.println("================================");
  Serial.println(" MQ SENTINEL - TEST 5 RESULTS");
  Serial.println("================================");

  Serial.print("Completed trials: ");
  Serial.println(completed);

  Serial.println();

  Serial.println(
    "Trial,Direction,Requested_ms,"
    "Actual_ms,Encoder_Cutoff,"
    "Encoder_After_2s,Residual_Counts"
  );

  for (byte i = 0; i < completed; i++) {

    bool reverseDirection = i >= 35;

    byte directionTrial =
        reverseDirection ? i - 35 : i;

    byte durationIndex =
        directionTrial / REPEATS;

    int address =
        DATA_ADDRESS +
        i * sizeof(TestRecord);

    TestRecord result;

    EEPROM.get(address, result);

    long residualCounts =
        result.encoderAfterObservation -
        result.encoderAtCutoff;

    Serial.print(i + 1);
    Serial.print(",");

    Serial.print(
      reverseDirection
      ? "REVERSE"
      : "FORWARD"
    );

    Serial.print(",");

    Serial.print(
      PULSE_TIMES[durationIndex]
    );

    Serial.print(",");

    Serial.print(result.actualTimeMs);
    Serial.print(",");

    Serial.print(result.encoderAtCutoff);
    Serial.print(",");

    Serial.print(
      result.encoderAfterObservation
    );

    Serial.print(",");

    Serial.println(residualCounts);
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  printResults();

  Serial.println();
  Serial.println("P = Print results");
  Serial.println("R = Reset trial counter");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  if (command == 'P' || command == 'p') {

    printResults();
  }

  if (command == 'R' || command == 'r') {

    EEPROM.update(COUNTER_ADDRESS, 0);

    Serial.println(
      "Trial counter reset to zero."
    );

    Serial.println(
      "Next run will be Trial 1."
    );
  }
}