#include <IRremote.hpp>

const int irPin = 2;
const int light = 3;

void setup() {
  Serial.begin(9600);

  IrReceiver.begin(irPin);

  pinMode(light, OUTPUT);

  Serial.println("IR Receiver Started");
}

void loop() {

  if (IrReceiver.decode()) {

    Serial.println("----------------------");

    Serial.print("HEX Data: 0x");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);

    Serial.print("Protocol: ");
    Serial.println(getProtocolString(IrReceiver.decodedIRData.protocol));

    Serial.print("Expected carrier: ");

    switch (IrReceiver.decodedIRData.protocol) {

      case NEC:
      case SAMSUNG:
      case LG:
      case JVC:
        Serial.println("38000 Hz");
        break;

      case SONY:
        Serial.println("40000 Hz");
        break;

      default:
        Serial.println("Unknown");
        break;
    }

    Serial.println("----------------------");

    IrReceiver.resume();
  }

  digitalWrite(light, HIGH);
  delay(500);

  digitalWrite(light, LOW);
  delay(500);
}
