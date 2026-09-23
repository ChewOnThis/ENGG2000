
/*
  MQ SENTINEL - TEST 3
  EEPROM RESULTS READER

  Upload AFTER completing the motor tests.

  Open Serial Monitor at 115200 baud.

  Sending R resets the trial counter.
  Saved results remain in EEPROM until overwritten.
*/

#include <EEPROM.h>

const byte TOTAL_TRIALS = 5;

const byte RECORD_MAGIC = 0xA5;

const int TRIAL_ADDRESS = 0;

const int RECORD_ADDRESS = 16;

const float COUNTS_PER_REV = 700.0;

// Must match the data structure
// in the motor test program.

struct TestRecord {

  byte magic;

  unsigned int requestedTime;

  unsigned int actualTime;

  long encoderAtStop;

  long encoderAfterStop;
};

// ============================================================
// PRINT RESULTS
// ============================================================

void printResults() {

  byte nextTrial =
      EEPROM.read(TRIAL_ADDRESS);

  if (nextTrial == 0xFF) {
    nextTrial = 0;
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println(" MQ SENTINEL - MOTOR TEST RESULTS");
  Serial.println("========================================");

  Serial.print("Completed trials: ");
  Serial.println(nextTrial);

  Serial.println();

  for (byte i = 0; i < TOTAL_TRIALS; i++) {

    TestRecord result;

    int address =
        RECORD_ADDRESS +
        i * sizeof(TestRecord);

    EEPROM.get(address, result);

    Serial.print("TRIAL ");
    Serial.println(i + 1);

    if (i >= nextTrial ||
        result.magic != RECORD_MAGIC) {

      Serial.println("No completed result.");
      Serial.println();

      continue;
    }

    Serial.print("Requested time (ms): ");
    Serial.println(result.requestedTime);

    Serial.print("Actual time (ms): ");
    Serial.println(result.actualTime);

    Serial.print("Encoder at power-off: ");
    Serial.println(result.encoderAtStop);

    Serial.print("Encoder after 2 seconds: ");
    Serial.println(result.encoderAfterStop);

    float wheelAngle =
        result.encoderAtStop *
        360.0 / COUNTS_PER_REV;

    Serial.print("Motor shaft angle at power-off: ");
    Serial.print(wheelAngle, 2);
    Serial.println(" degrees");

    Serial.println();
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  printResults();

  Serial.println(
    "Send R to reset trial counter."
  );
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  if (Serial.available()) {

    char command = Serial.read();

    if (command == 'R' || command == 'r') {

      EEPROM.update(TRIAL_ADDRESS, 0);

      Serial.println();
      Serial.println("Trial counter reset.");
      Serial.println(
        "Next motor test will start at 100 ms."
      );
    }
  }
}