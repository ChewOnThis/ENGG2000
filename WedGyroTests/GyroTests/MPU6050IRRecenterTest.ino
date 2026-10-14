#include <Wire.h>

// ================= SETTINGS =================
const uint8_t  MPU_ADDR   = 0x68;
const uint8_t  IR_PIN     = 2;       // TSSP OUT -> D2
const uint8_t  LED_PIN    = 6;       // LED (with resistor) -> D6
const float    GYRO_SENS  = 131.0;   // LSB per deg/s (+-250 dps range)
const uint16_t GAP_MS     = 150;     // IR gone this long = sweep finished
const float    TOL_ON     = 2.0;     // LED turns ON within +-2 deg of target
const float    TOL_OFF    = 3.0;     // LED turns OFF beyond +-3 deg (prevents flicker)
// ============================================

enum State { SEARCHING, TRACKING, RECENTERING };
State state = SEARCHING;

float angleZ = 0.0;
float bias = 0.0;
float firstAngle = 0.0;     // angle where IR was first detected
float lastAngle = 0.0;      // angle where IR was last detected
float targetAngle = 0.0;    // middle of the two = sensor direction
bool ledOn = false;
unsigned long lastMicros;
unsigned long lastPrint = 0;
unsigned long lastSeenMs = 0;

bool readGyroZ(int16_t &raw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                          // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 2, (int)true) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = (int16_t)((hi << 8) | lo);
  return true;
}

void line() {
  Serial.println(F("=========================================="));
}

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);          // wake the gyro
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("ERROR: Gyro not found at 0x68"));
    while (true) {}
  }

  Serial.println(F("Keep still, calibrating..."));
  long sum = 0;
  int good = 0;
  for (int i = 0; i < 300; i++) {
    int16_t raw;
    if (readGyroZ(raw)) { sum += raw; good++; }
    delay(3);
  }
  if (good > 0) bias = ((float)sum / good) / GYRO_SENS;

  line();
  Serial.println(F("Ready. Rotate slowly past the IR emitter."));
  Serial.println(F("Send 'r' to restart the search."));
  line();
  lastMicros = micros();
}

void loop() {
  // ---- 1. Update Z angle ----
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  int16_t raw;
  if (readGyroZ(raw)) {
    angleZ += (((float)raw / GYRO_SENS) - bias) * dt;
  }

  // ---- 2. Serial command: 'r' = restart ----
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      state = SEARCHING;
      ledOn = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println(F(">>> Restarted. Searching for IR..."));
    }
  }

  // ---- 3. IR logic ----
  bool detected = (digitalRead(IR_PIN) == LOW);   // TSSP is active LOW
  unsigned long ms = millis();

  if (state == SEARCHING) {
    if (detected) {
      firstAngle = angleZ;
      lastAngle = angleZ;
      lastSeenMs = ms;
      state = TRACKING;
      Serial.print(F(">> IR FIRST SEEN at: "));
      Serial.print(firstAngle, 2);
      Serial.println(F(" deg"));
    }
  }
  else if (state == TRACKING) {
    if (detected) {
      lastAngle = angleZ;       // keeps updating while IR is seen
      lastSeenMs = ms;
    }
    else if (ms - lastSeenMs > GAP_MS) {
      targetAngle = (firstAngle + lastAngle) / 2.0;
      state = RECENTERING;
      Serial.print(F(">> IR LAST SEEN at:  "));
      Serial.print(lastAngle, 2);
      Serial.println(F(" deg"));
      line();
      Serial.print(F("  Sensor found. TARGET = "));
      Serial.print(targetAngle, 2);
      Serial.println(F(" deg"));
      Serial.println(F("  Rotate back until the LED turns on."));
      line();
    }
  }

  // ---- 4. LED logic (only while recentering) ----
  float error = 0.0;
  if (state == RECENTERING) {
    error = targetAngle - angleZ;
    while (error > 180.0)  error -= 360.0;   // shortest way around
    while (error < -180.0) error += 360.0;
    float absErr = fabs(error);

    if (!ledOn && absErr <= TOL_ON)  ledOn = true;
    if (ledOn  && absErr > TOL_OFF)  ledOn = false;
  } else {
    ledOn = false;
  }
  digitalWrite(LED_PIN, ledOn ? HIGH : LOW);

  // ---- 5. Serial display ----
  if (ms - lastPrint >= 200) {
    lastPrint = ms;

    if (state == RECENTERING) {
      Serial.print(F("RECENTERING | Current: "));
      Serial.print(angleZ, 2);
      Serial.print(F("  Target: "));
      Serial.print(targetAngle, 2);
      Serial.print(F("  Error: "));
      Serial.print(error, 2);
      Serial.println(ledOn ? F("  [LED ON - CENTERED]") : F("  [LED off]"));
    } else {
      Serial.print(state == SEARCHING ? F("SEARCHING   | ") : F("TRACKING    | "));
      Serial.print(F("Z angle: "));
      Serial.print(angleZ, 2);
      Serial.print(F(" deg   "));
      Serial.println(detected ? F("DETECTED") : F("not detected"));
    }
  }
}
