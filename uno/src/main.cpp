#include <Arduino.h>
#include "SoftwareSerial.h"
#include "sending.h"

SoftwareSerial toMega(10, 11);

void setup() {
  Serial.begin(9600);
  toMega.begin(115200);
}

void loop() {
  Serial.println(msg::header);
  delay(1000);

  // need to be able to send:
  // drive X dir, pinion forward, pinion back
  // conveyor on
  // press button
  // estop, turn X
}