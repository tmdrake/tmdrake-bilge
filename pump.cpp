/*
  Float input and the pump state machine.

  IDLE        float is FULL          -> PUMPING, arm the 10 minute cap
  PUMPING     float drops            -> EMPTY_RUNON, arm the 3 minute guess
  PUMPING     cap expires            -> FAULT, motor off
  EMPTY_RUNON guess expires          -> IDLE, motor off, tank called empty
  EMPTY_RUNON float rises            -> PUMPING, cap is NOT restarted
  EMPTY_RUNON cap expires            -> FAULT, motor off
  FAULT       float rises again      -> PUMPING, new cap (it went down, then up)
  FAULT       float stays FULL       -> stay off (stuck switch, no new edge)
  FAULT       float down for 3 min   -> IDLE

  A rise during the empty guess does not restart the cap. A rise after
  a fault does: the float had to go down before it could come back up.
*/

#include "bilge.h"

static bool arming = false;               // a level different from stableFull is being timed
static bool armLevel = false;             // that pending level
static unsigned long armSince = 0;         // when the pending level first appeared
static bool opposing = false;             // a spike against the pending level is being timed
static unsigned long opposeSince = 0;      // when that spike started

static long emptySamples[EMPTY_RING];  // error of each finished 3 minute guess, milliseconds
static uint8_t emptyCount = 0;         // how many of those slots are filled, up to 16
static uint8_t emptyNext = 0;          // slot that will be overwritten next

static bool timerDue(const Timer& t, unsigned long now) {
  // Unsigned subtract wraps correctly when millis() rolls past 2^32.
  return t.armed && (unsigned long)(now - t.startedAt) >= t.duration;
}

unsigned long timerLeft(const Timer& t, unsigned long now) {
  unsigned long elapsed = (unsigned long)(now - t.startedAt);  // same wrap-safe elapsed time
  if (elapsed >= t.duration) return 0;                         // already due: nothing left
  return t.duration - elapsed;                                 // milliseconds still to go
}

// Positive: the output changed after the deadline. Negative: it stopped early.
static long timerError(unsigned long firedAt, unsigned long startedAt,
                       unsigned long duration) {
  return (long)(firedAt - startedAt) - (long)duration;  // both values are minutes, so they fit in a long
}

static void disarmTimer(Timer& t) {
  t.armed = false;  // startedAt is left alone; it is ignored while disarmed
}

static void armTimer(Timer& t, unsigned long now, unsigned long duration) {
  t.armed = true;
  t.startedAt = now;        // deadline is this instant plus duration, not a stored end time
  t.duration = duration;
}

static void pushEmptyError(long errorMs) {
  emptySamples[emptyNext] = errorMs;                              // overwrite the oldest once full
  emptyNext = (uint8_t)((emptyNext + 1) % EMPTY_RING);            // advance, wrapping at 16
  if (emptyCount < EMPTY_RING) emptyCount++;                      // grow until the ring is full
}

EmptyStats emptyErrorStats() {
  EmptyStats stats = {0, 0, 0, 0, 0};              // n == 0 tells the screen there is no average yet
  stats.n = emptyCount;
  if (emptyCount == 0) return stats;               // do not read samples that were never written

  long sum = 0;
  stats.lowest = emptySamples[0];                  // seed min and max from the first stored sample
  stats.highest = emptySamples[0];
  for (uint8_t i = 0; i < emptyCount; i++) {
    long sample = emptySamples[i];                 // every filled slot is a real guess, in any order
    sum += sample;
    if (sample < stats.lowest) stats.lowest = sample;
    if (sample > stats.highest) stats.highest = sample;
  }
  stats.average = sum / (long)emptyCount;         // truncates toward zero; these are millisecond errors
  stats.last = emptySamples[(emptyNext + EMPTY_RING - 1) % EMPTY_RING];  // the slot just written
  return stats;
}

static void rememberFire(FireReason reason, long errorMs, unsigned long now,
                         bool floatStillFull, bool holdArmed,
                         unsigned long holdLeft) {
  lastValid = true;                                         // the screen may show this fire
  lastReason = reason;                                      // empty guess, float back, or cap
  lastError = errorMs;                                      // late is positive, early is negative
  lastAtTop = atTopMs;                                      // frozen copy of how long the float was up
  lastRan = (unsigned long)(now - motorOnAt);               // how long the motor had been on
  lastFloatStillFull = floatStillFull;                      // used when the cap hits with the float up
  lastHoldArmed = holdArmed;                                // used when the cap hits during the guess
  lastHoldLeft = holdLeft;                                  // guess time still left at the cap
  requestScreen();                                          // show the new fire without waiting a second
}

// A new cycle only. Do not call this when the float returns mid-run,
// or the 10 minute cap would start over.
static void startCycle(unsigned long now) {
  if (waitingRefill) {                                              // the tank was called empty and is full again
    refillMs = (unsigned long)(now - inferredEmptyAt);              // how long that took
    refillValid = true;                                            // the screen can show the number
    waitingRefill = false;                                         // the live "since guess" clock stops
  }
  armTimer(maxRun, now, MAX_RUN_MS);        // 10 minutes from this start, not from a later bounce
  disarmTimer(emptyRun);                    // no empty guess until the float actually drops
  disarmTimer(faultClear);                  // a new cycle leaves the fault latch behind
  motorOnAt = now;                          // whole-run clock, including the later 3 minute guess
  topSince = now;                           // "at top" clock starts as the float reaches FULL
  atTopMs = 0;                              // not known until the float drops
  setMotor(true, now);                      // pin on, run count saved
  state = STATE_PUMPING;
}

static void beginEmptyGuess(unsigned long now) {
  atTopMs = (unsigned long)(now - topSince);  // how long the water stayed at the top switch
  armTimer(emptyRun, now, EMPTY_RUNON_MS);    // 3 more minutes to empty what the float cannot see
  state = STATE_EMPTY_RUNON;                  // motor stays on; the cap stays armed
}

static void floatReturned(unsigned long now) {
  long errorMs = timerError(now, emptyRun.startedAt, emptyRun.duration);  // negative: guess cancelled early
  rememberFire(FIRE_EARLY, errorMs, now, true, false, 0);                  // water is back at the top
  earlyCount++;                                                            // not mixed into the empty-guess average
  disarmTimer(emptyRun);                                                   // the 3 minute guess is over
  topSince = now;                                                          // a new "at top" interval starts
  atTopMs = 0;
  state = STATE_PUMPING;                                                   // maxRun is left running from the original start
}

static void guessEmpty(unsigned long now) {
  long errorMs = timerError(now, emptyRun.startedAt, emptyRun.duration);  // small positive means the loop was a bit late
  rememberFire(FIRE_EMPTY, errorMs, now, false, false, 0);
  pushEmptyError(errorMs);                    // this is the only event that enters the average
  cycles++;                                   // one completed empty guess; not saved across power loss
  disarmTimer(emptyRun);
  disarmTimer(maxRun);                        // the cycle is over, so the cap is too
  setMotor(false, now);                       // pin off, and this run's minutes are saved
  inferredEmptyAt = now;                      // start the "how long until full again" clock
  waitingRefill = true;
  state = STATE_IDLE;
}

static void tripMax(unsigned long now) {
  bool holdArmed = emptyRun.armed;                                 // true if we were still in the 3 minute guess
  unsigned long holdLeft = holdArmed ? timerLeft(emptyRun, now) : 0;
  if (state == STATE_PUMPING) {
    atTopMs = (unsigned long)(now - topSince);                     // float never dropped, so close the at-top clock now
  }
  long errorMs = timerError(now, maxRun.startedAt, maxRun.duration);  // how late the cap itself fired
  rememberFire(FIRE_MAX, errorMs, now, stableFull, holdArmed, holdLeft);
  faultCount++;
  disarmTimer(emptyRun);                    // neither remaining timer may turn the motor back on
  disarmTimer(maxRun);
  disarmTimer(faultClear);
  setMotor(false, now);                     // latched off, and the partial run's minutes are saved
  state = STATE_FAULT;
  if (!stableFull) armTimer(faultClear, now, EMPTY_RUNON_MS);  // float already down: start the 3 minute clear
}

static void serviceFault(unsigned long now) {
  if (stableFull) {
    disarmTimer(faultClear);                 // float popped back up: the clear must start over
    return;                                  // stay in FAULT, motor stays off
  }
  if (!faultClear.armed) {
    armTimer(faultClear, now, EMPTY_RUNON_MS);  // float just went down: require a full 3 quiet minutes
    return;
  }
  if (timerDue(faultClear, now)) {
    disarmTimer(faultClear);                 // the quiet period finished
    state = STATE_IDLE;                     // the next rise may start a new cycle
  }
}

void seedFloat(unsigned long now, bool full) {
  stableFull = full;          // accept the boot reading without waiting 50 ms
  arming = false;             // do not turn that boot reading into a second edge
  if (full) startCycle(now);  // tank already at the top: pump on, cap armed
}

void debounceFloat(unsigned long now, bool& rose, bool& fell) {
  bool rawFull = digitalRead(FLOAT_PIN) == HIGH;  // one sample of pin 2

  if (!arming) {
    if (rawFull == stableFull) return;  // still at the level we already accepted
    arming = true;                      // the switch moved: start the confirm timer
    armLevel = rawFull;
    armSince = now;
    opposing = false;
    return;
  }

  if (rawFull == armLevel) {
    opposing = false;  // a one-sample spike does not throw away the confirm time
    if ((unsigned long)(now - armSince) >= DEBOUNCE_MS) {
      stableFull = armLevel;            // the new level held for the full debounce
      arming = false;
      if (armLevel) rose = true;        // water reached the top
      else fell = true;                 // water left the top
    }
    return;
  }

  // Sample went back toward the old level. Wait before cancelling the edge,
  // or contact bounce makes a real trip start its 50 ms over forever.
  if (!opposing) {
    opposing = true;
    opposeSince = now;
    return;
  }
  if ((unsigned long)(now - opposeSince) >= GLITCH_MS) {
    arming = false;  // the new level did not hold; keep the accepted one
  }
}

void servicePump(unsigned long now, bool rose, bool fell) {
  switch (state) {
    case STATE_IDLE:
      // Level as well as edge. A rise accepted in the same pass as the
      // empty guess would otherwise leave us idle while the tank is full.
      if (rose || stableFull) startCycle(now);
      break;

    case STATE_PUMPING:
      if (timerDue(maxRun, now)) tripMax(now);       // cap first, so a stuck float cannot run forever
      else if (fell) beginEmptyGuess(now);           // float dropped: keep pumping for 3 minutes
      break;

    case STATE_EMPTY_RUNON:
      // The cap wins over the empty guess. A float rise wins over the
      // guess in the same pass: water is at the top, so the tank is not empty.
      if (timerDue(maxRun, now)) tripMax(now);
      else if (rose) floatReturned(now);
      else if (timerDue(emptyRun, now)) guessEmpty(now);  // 3 minutes done: call the tank empty
      break;

    case STATE_FAULT:
      // A rise is a new trip: the float went down, then came back up.
      // A stuck HIGH never rises, so the latch still holds for that case.
      if (rose) startCycle(now);
      else serviceFault(now);
      break;
  }
}
