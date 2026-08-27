// ============================================================
// MQ SENTINEL - MVP CONTROL
// Arduino Nano
//
// D2 = Left IR sensor
// D3 = Centre IR sensor
// D4 = Right IR sensor
// D6 = DRV8874 IN1
// D7 = DRV8874 IN2
// D8 = Laser
// ============================================================

const byte IR_LEFT   = 2;
const byte IR_CENTRE = 3;
const byte IR_RIGHT  = 4;

const byte MOTOR_IN1 = 6;
const byte MOTOR_IN2 = 7;

const byte LASER_PIN = 8;


// Change to true if motor turns the wrong direction
const bool REVERSE_MOTOR = false;


// Sensor filtering
const unsigned long IR_HOLD_MS = 60;

// Centre must remain detected this long before laser fires
const unsigned long CENTRE_STABLE_MS = 150;

// Required target dwell
const unsigned long LASER_TIME_MS = 2000;

// Serial update interval
const unsigned long PRINT_MS = 250;


// Last time each IR sensor detected activity
unsigned long lastLeft = 0;
unsigned long lastCentre = 0;
unsigned long lastRight = 0;

unsigned long centreStart = 0;
unsigned long laserStart = 0;
unsigned long lastPrint = 0;

bool firing = false;
bool targetComplete = false;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTRE, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(LASER_PIN, OUTPUT);

  stopMotor();
  laserOff();

  Serial.println("================================");
  Serial.println("     MQ SENTINEL MVP READY");
  Serial.println("================================");
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  unsigned long now = millis();


  // ----------------------------------------------------------
  // READ IR RECEIVERS
  // 38 kHz receiver modules are normally ACTIVE LOW
  // ----------------------------------------------------------

  if (digitalRead(IR_LEFT) == LOW)
    lastLeft = now;

  if (digitalRead(IR_CENTRE) == LOW)
    lastCentre = now;

  if (digitalRead(IR_RIGHT) == LOW)
    lastRight = now;


  // Hold detections briefly so short IR pulses are not missed

  bool left =
      lastLeft != 0 &&
      now - lastLeft <= IR_HOLD_MS;

  bool centre =
      lastCentre != 0 &&
      now - lastCentre <= IR_HOLD_MS;

  bool right =
      lastRight != 0 &&
      now - lastRight <= IR_HOLD_MS;


  // ==========================================================
  // CENTRE SENSOR
  // Highest priority
  // ==========================================================

  if (centre) {

    stopMotor();


    // Start alignment timer
    if (centreStart == 0)
      centreStart = now;


    // Fire laser after stable alignment
    if (!firing &&
        !targetComplete &&
        now - centreStart >= CENTRE_STABLE_MS) {

      firing = true;
      laserStart = now;

      laserOn();

      Serial.println(">>> TARGET ALIGNED - LASER ON");
    }


    // Complete 2 second dwell
    if (firing &&
        now - laserStart >= LASER_TIME_MS) {

      laserOff();

      firing = false;
      targetComplete = true;

      Serial.println(">>> 2 SECOND TARGET HIT COMPLETE");
    }
  }


  // ==========================================================
  // NOT CENTRED
  // ==========================================================

  else {

    centreStart = 0;


    // If alignment is lost during laser dwell,
    // cancel the attempt immediately
    if (firing) {

      firing = false;

      laserOff();

      Serial.println(">>> ALIGNMENT LOST - LASER OFF");
    }


    // --------------------------------------------------------
    // LEFT SENSOR
    // --------------------------------------------------------

    if (left && !right) {

      targetComplete = false;

      turnLeft();
    }


    // --------------------------------------------------------
    // RIGHT SENSOR
    // --------------------------------------------------------

    else if (right && !left) {

      targetComplete = false;

      turnRight();
    }


    // --------------------------------------------------------
    // No clear direction
    // --------------------------------------------------------

    else {

      stopMotor();
    }
  }


  // Beacon disappeared completely -> ready for next target

  if (!left && !centre && !right) {

    targetComplete = false;
  }


  // ==========================================================
  // SERIAL MONITOR
  // ==========================================================

  if (now - lastPrint >= PRINT_MS) {

    lastPrint = now;

    Serial.print("LEFT:");
    Serial.print(left);

    Serial.print("  CENTRE:");
    Serial.print(centre);

    Serial.print("  RIGHT:");
    Serial.print(right);

    Serial.print("  |  ");

    if (firing)
      Serial.print("LASER FIRING");

    else if (centre)
      Serial.print("ALIGNED");

    else if (left && !right)
      Serial.print("TURNING LEFT");

    else if (right && !left)
      Serial.print("TURNING RIGHT");

    else
      Serial.print("WAITING");

    Serial.println();
  }
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void turnLeft() {

  laserOff();

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);

  } else {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  }
}


void turnRight() {

  laserOff();

  if (!REVERSE_MOTOR) {

    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);

  } else {

    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }
}


void stopMotor() {

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}


// ============================================================
// LASER CONTROL
// ============================================================

void laserOn() {

  digitalWrite(LASER_PIN, HIGH);
}


void laserOff() {

  digitalWrite(LASER_PIN, LOW);
}
