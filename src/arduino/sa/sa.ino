#include <AccelStepper.h>
#include "orm.h"

ORM orm;

void setup() {

  orm.setup();
}

void loop() {
  orm.ospSerialLoop();
}
