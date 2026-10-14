#include "SentinelController.h"

SentinelController sentinel;

void setup() {
  sentinel.begin();
}

void loop() {
  sentinel.update();
}
