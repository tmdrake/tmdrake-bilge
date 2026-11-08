/*
  Shared names for the dehumidifier pump timer.

  The float is on the top of the tank. HIGH means full. The switch
  cannot see the bottom, so "empty" is a guess made after the float
  drops. See README.md for wiring and the serial screen.
*/

#pragma once

#include <Arduino.h>

const int FLOAT_PIN = 2;            // active HIGH: float up, tank at the top
const int MOTOR_PIN = LED_BUILTIN;  // pin 13 follows the pump. Use a relay.

// Pump-down after the top float drops. The motor still needs about
// 3 minutes to empty the water below the switch.
const unsigned long EMPTY_RUNON_MS = 3UL * 60UL * 1000UL;

// Cap on one continuous run, including that pump-down. Longer than a
// normal empty, so it does not cut the 3 minutes short.
const unsigned long MAX_RUN_MS = 10UL * 60UL * 1000UL;

const unsigned long DEBOUNCE_MS = 50UL;
const unsigned long SCREEN_MS = 1000UL;
const uint8_t EMPTY_RING = 16;
const unsigned long SERIAL_BAUD = 115200UL;

static_assert(MAX_RUN_MS > EMPTY_RUNON_MS,
              "max run must outlast the empty guess");

enum RunState {
  STATE_IDLE = 0,
  STATE_PUMPING,
  STATE_EMPTY_RUNON,
  STATE_FAULT
};

enum FireReason {
  FIRE_NONE = 0,
  FIRE_EMPTY,
  FIRE_EARLY,
  FIRE_MAX
};

// A timer is due when (now - startedAt) >= duration. That unsigned
// subtraction stays correct across the millis() wrap. Do not compare
// two absolute deadlines, and do not use 0 as an "not armed" flag.
struct Timer {
  bool armed;
  unsigned long startedAt;
  unsigned long duration;
};

struct EmptyStats {
  uint8_t n;
  long last;
  long average;
  long lowest;
  long highest;
};

// Pump condition. pump.cpp owns the transitions.
extern RunState state;
extern bool motorOn;
extern bool stableFull;

extern Timer maxRun;
extern Timer emptyRun;
extern Timer faultClear;

extern unsigned long motorOnAt;
extern unsigned long topSince;
extern unsigned long atTopMs;

// Time from an empty guess until the float reaches the top again.
extern bool waitingRefill;
extern unsigned long inferredEmptyAt;
extern bool refillValid;
extern unsigned long refillMs;

extern unsigned int cycles;
extern unsigned int earlyCount;
extern unsigned int faultCount;

// Most recent time the pump turned off, or the cap tripped.
extern bool lastValid;
extern FireReason lastReason;
extern long lastError;
extern unsigned long lastAtTop;
extern unsigned long lastRan;
extern bool lastFloatStillFull;
extern bool lastHoldArmed;
extern unsigned long lastHoldLeft;

// Committed motor time. The in-progress run is not included here.
extern unsigned long runCount;
extern unsigned long totalRunMs;
extern unsigned long thisRunStart;
extern unsigned long lastRunMs;
extern bool haveLastRun;

unsigned long timerLeft(const Timer& t, unsigned long now);

void loadOdo();
unsigned long odometerMs(unsigned long now);
void setMotor(bool on, unsigned long now);

void seedFloat(unsigned long now, bool full);
void debounceFloat(unsigned long now, bool& rose, bool& fell);
void servicePump(unsigned long now, bool rose, bool fell);
EmptyStats emptyErrorStats();

void requestScreen();
void handleKey(char c, unsigned long now);
void serviceScreen(unsigned long now);
