// Current Sentinel wiring: laser D4; DRV8874 IN1 D5, IN2 D6.
// Motor inputs both HIGH select brake in the existing PMODE HIGH setup.
void setup() {
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(4, OUTPUT);
}
void loop() {}
