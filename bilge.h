/*
  Shared names for the dehumidifier pump timer.

  The float is on the top of the tank. HIGH means full. The switch
  cannot see the bottom, so "empty" is a guess made after the float
  drops. See README.md for wiring and the serial screen.
*/

#pragma once

#include <Arduino.h>

const int FLOAT_PIN = 2;            // active HIGH when the top float is up
const int MOTOR_PIN = LED_BUILTIN;  // pin 13 follows the pump; drive a relay, not the motor

// How long to keep pumping after the float drops. The switch cannot see
// empty. Three test runs of 3 minutes emptied the tank, so one real
// drop is 9 minutes. Every factor is unsigned long so the product
// cannot overflow a 16-bit AVR int.
const unsigned long EMPTY_RUNON_MS = 9UL * 60UL * 1000UL;

// Hard stop for one continuous motor-on period, including those 9
// minutes. Fifteen minutes leaves room for the time the float stays
// up before it drops, and still stops a stuck switch.
const unsigned long MAX_RUN_MS = 15UL * 60UL * 1000UL;

const unsigned long DEBOUNCE_MS = 50UL;   // a new float level must hold this long before it counts
const unsigned long GLITCH_MS = 20UL;     // a shorter spike does not cancel that wait
const unsigned long SCREEN_MS = 1000UL;   // live screen refresh, also on every state change
const uint8_t EMPTY_RING = 16;            // last N empty-guess errors kept for the average
const unsigned long SERIAL_BAUD = 115200UL;

// A compile error here means the safety cap would cut off a normal empty.
static_assert(MAX_RUN_MS > EMPTY_RUNON_MS,
              "max run must outlast the empty guess");

enum RunState {
  STATE_IDLE = 0,      // motor off, waiting for the float to reach the top
  STATE_PUMPING,       // float is FULL, motor on, 15 minute cap running
  STATE_EMPTY_RUNON,   // float dropped, motor still on for the 9 minute empty
  STATE_FAULT          // cap tripped, motor latched off until the float stays down
};

enum FireReason {
  FIRE_NONE = 0,       // no timer has finished since boot
  FIRE_EMPTY,          // the 9 minute empty reached its deadline
  FIRE_EARLY,          // the float returned to the top before that deadline
  FIRE_MAX             // the 15 minute cap won
};

// A timer is due when (now - startedAt) >= duration. Unsigned subtraction
// stays correct when millis() wraps at 49.7 days. Do not compare two
// absolute deadlines, and do not treat 0 as "not armed".
struct Timer {
  bool armed;                  // false means this timer is not running
  unsigned long startedAt;     // millis() when it was armed
  unsigned long duration;      // how long it should run, in milliseconds
};

// Summary of the last 16 empty-guess errors. n == 0 means none yet.
struct EmptyStats {
  uint8_t n;         // how many samples are in the ring, at most 16
  long last;         // most recent error, milliseconds, positive means late
  long average;      // integer average of the samples
  long lowest;       // most early (most negative) sample
  long highest;      // most late sample
};

// Pump condition. pump.cpp owns the transitions.
extern RunState state;     // which row of the state machine we are in
extern bool motorOn;       // true while pin 13 is HIGH
extern bool stableFull;    // debounced float: true means water is at the top

extern Timer maxRun;       // 15 minute cap, armed when a cycle starts
extern Timer emptyRun;     // 9 minute empty, armed when the float drops
extern Timer faultClear;   // how long the float must stay down to leave FAULT

extern unsigned long motorOnAt;  // millis() when this continuous run turned the motor on
extern unsigned long topSince;   // millis() when the float last became FULL
extern unsigned long atTopMs;    // how long the float stayed FULL before it dropped

// After an empty guess, how long until the float reaches the top again.
extern bool waitingRefill;            // true from the guess until the next FULL
extern unsigned long inferredEmptyAt; // millis() when the guess turned the motor off
extern bool refillValid;              // true once a refill interval has been measured
extern unsigned long refillMs;        // that completed interval

extern unsigned int cycles;      // empty guesses that finished, not saved in EEPROM
extern unsigned int earlyCount;  // times the float came back before the guess
extern unsigned int faultCount;  // times the 15 minute cap tripped

// Most recent time the pump turned off, or the cap tripped.
extern bool lastValid;              // false until the first fire
extern FireReason lastReason;       // which timer event that was
extern long lastError;              // milliseconds late (positive) or early (negative)
extern unsigned long lastAtTop;     // at-top time copied at the moment of the fire
extern unsigned long lastRan;       // how long the motor had been on at that moment
extern bool lastFloatStillFull;     // cap tripped while the float was still up
extern bool lastHoldArmed;          // cap tripped during the 9 minute empty
extern unsigned long lastHoldLeft;  // guess time still remaining when the cap won

// Committed motor time. The in-progress run is not included here.
extern unsigned long runCount;      // motor starts, saved to EEPROM at each start
extern unsigned long totalRunMs;    // finished run time only, saved when the motor stops
extern unsigned long thisRunStart;  // millis() of the current ON edge, not saved
extern unsigned long lastRunMs;     // duration of the last finished run
extern bool haveLastRun;            // false until one run has finished

unsigned long timerLeft(const Timer& t, unsigned long now);  // 0 if the timer is already due

void loadOdo();                                          // read EEPROM, ignore it if the magic mismatches
unsigned long odometerMs(unsigned long now);             // saved time plus the current run, if the motor is on
void setMotor(bool on, unsigned long now);               // change the pin, then save the count or the added time

void seedFloat(unsigned long now, bool full);            // boot reading, no fake edge
void testPumpDown(unsigned long now);                   // pretend the float just dropped; run the 9 minute empty
void debounceFloat(unsigned long now, bool& rose, bool& fell);  // rose/fell set only on an accepted edge
void servicePump(unsigned long now, bool rose, bool fell);      // one step of the state machine
EmptyStats emptyErrorStats();                            // average of the empty-guess errors only

void requestScreen();                                    // rebuild the live block on the next send
void handleKey(char c, unsigned long now);               // ?, o, or s; other bytes are ignored
void serviceScreen(unsigned long now);                   // non-blocking send of the live block
