#include <EEPROM.h>

#include <AccelStepper.h>
#include "orm.h"

ORM orm;

void setup() {
  digitalWrite(10,1);
  orm.setup();
}

void loop() {
  orm.ospSerialLoop();
}
