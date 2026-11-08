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
  Serial.begin(SERIAL_BAUD);                 // 115200, so the screen does not stall the timer
  pinMode(FLOAT_PIN, INPUT);                 // pin 2, no pullup: the float drives it HIGH when full
  pinMode(MOTOR_PIN, OUTPUT);                // pin 13, the onboard LED follows the pump relay
  digitalWrite(MOTOR_PIN, LOW);              // hold the pump off until we know the float state

  loadOdo();                                 // restore runs and committed runtime from EEPROM
  seedFloat(millis(), digitalRead(FLOAT_PIN) == HIGH);  // one raw read; a full tank starts the pump

  Serial.println(F("tmdrake-bilge 115200  ? for menu"));  // boot banner, before the repeating screen
}

void loop() {
  unsigned long now = millis();              // one timestamp for this pass, wrap-safe in the helpers
  bool rose = false;                         // set if the float just became stably FULL
  bool fell = false;                         // set if the float just became stably DOWN
  debounceFloat(now, rose, fell);            // ignore edges shorter than 50 ms
  servicePump(now, rose, fell);              // turn the motor and the timers from that edge
  if (Serial.available() > 0) handleKey((char)Serial.read(), now);  // one key per pass, never waits
  serviceScreen(now);                        // send only what the UART buffer can take
}
