// MPU-6050 gyro test for Arduino Nano
// Wiring (GY-521 board): VCC->5V, GND->GND, SDA->A4, SCL->A5, AD0->GND (addr 0x68)
// Serial Monitor: 115200 baud

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;        // 0x69 if AD0 is tied high
const float   GYRO_SCALE = 131.0;     // LSB per deg/s at +/-250 dps (default range)
const int     CAL_SAMPLES = 500;

float offX = 0, offY = 0, offZ = 0;   // gyro bias in deg/s
float angX = 0, angY = 0, angZ = 0;   // integrated angles in degrees
unsigned long lastMicros;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);                   // GYRO_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) return false;
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // WHO_AM_I check
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  uint8_t who = Wire.available() ? Wire.read() : 0xFF;
  Serial.print("WHO_AM_I: 0x");
  Serial.println(who, HEX);
  if (who != 0x68) {
    Serial.println("MPU not found (expected 0x68). Check wiring/address.");
    while (1);
  }

  writeReg(0x6B, 0x00);               // wake up (clear sleep bit)
  writeReg(0x1B, 0x00);               // gyro range +/-250 dps
  delay(100);

  Serial.println("Calibrating - keep sensor still...");
  long sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    int16_t x, y, z;
    if (readGyroRaw(x, y, z)) { sx += x; sy += y; sz += z; n++; }
    delay(3);
  }
  if (n == 0) { Serial.println("Read failed during calibration."); while (1); }
  offX = sx / (float)n / GYRO_SCALE;
  offY = sy / (float)n / GYRO_SCALE;
  offZ = sz / (float)n / GYRO_SCALE;
  Serial.println("Done. Columns: dps X, Y, Z | angle X, Y, Z (deg)");
  lastMicros = micros();
}

void loop() {
  int16_t rx, ry, rz;
  if (!readGyroRaw(rx, ry, rz)) {
    Serial.println("I2C read error");
    delay(100);
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;

  float gx = rx / GYRO_SCALE - offX;
  float gy = ry / GYRO_SCALE - offY;
  float gz = rz / GYRO_SCALE - offZ;

  angX += gx * dt;
  angY += gy * dt;
  angZ += gz * dt;

  Serial.print("dps: ");
  Serial.print(gx, 2); Serial.print('\t');
  Serial.print(gy, 2); Serial.print('\t');
  Serial.print(gz, 2);
  Serial.print("  |  angle: ");
  Serial.print(angX, 1); Serial.print('\t');
  Serial.print(angY, 1); Serial.print('\t');
  Serial.println(angZ, 1);

  delay(20);                          // ~50 Hz print rate
}
