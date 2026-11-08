/*
  tmdrake-bilge

  Top-float pump timer for a dehumidifier tank. Wiring, timeouts, and
  the serial screen are in README.md. Shared names are in bilge.h.

  pump.cpp       float debounce and the pump states
  odometer.cpp   run count, runtime, EEPROM
  screen.cpp     live screen and the one-key menu
*/

#include "bilge.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(FLOAT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  loadOdo();
  seedFloat(millis(), digitalRead(FLOAT_PIN) == HIGH);

  Serial.println(F("tmdrake-bilge 115200  ? for menu"));
}

void loop() {
  unsigned long now = millis();
  bool rose = false;
  bool fell = false;
  debounceFloat(now, rose, fell);
  servicePump(now, rose, fell);
  if (Serial.available() > 0) handleKey((char)Serial.read(), now);
  serviceScreen(now);
}
