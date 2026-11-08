/*
  Float input and the pump state machine.

  IDLE        float rises            -> PUMPING, arm the 10 minute cap
  PUMPING     float drops            -> EMPTY_RUNON, arm the 3 minute guess
  PUMPING     cap expires            -> FAULT, motor off
  EMPTY_RUNON guess expires          -> IDLE, motor off, tank called empty
  EMPTY_RUNON float rises            -> PUMPING, cap is NOT restarted
  EMPTY_RUNON cap expires            -> FAULT, motor off
  FAULT       float down for 3 min   -> IDLE

  The cap is not restarted when the float returns. A bouncing float
  would otherwise run the motor forever.
*/

#include "bilge.h"

static bool candidateFull = false;
static unsigned long candidateSince = 0;

static long emptySamples[EMPTY_RING];
static uint8_t emptyCount = 0;
static uint8_t emptyNext = 0;

static bool timerDue(const Timer& t, unsigned long now) {
  return t.armed && (unsigned long)(now - t.startedAt) >= t.duration;
}

unsigned long timerLeft(const Timer& t, unsigned long now) {
  unsigned long elapsed = (unsigned long)(now - t.startedAt);
  if (elapsed >= t.duration) return 0;
  return t.duration - elapsed;
}

// Positive: the output changed after the deadline. Negative: it stopped early.
static long timerError(unsigned long firedAt, unsigned long startedAt,
                       unsigned long duration) {
  return (long)(firedAt - startedAt) - (long)duration;
}

static void disarmTimer(Timer& t) {
  t.armed = false;
}

static void armTimer(Timer& t, unsigned long now, unsigned long duration) {
  t.armed = true;
  t.startedAt = now;
  t.duration = duration;
}

static void pushEmptyError(long errorMs) {
  emptySamples[emptyNext] = errorMs;
  emptyNext = (uint8_t)((emptyNext + 1) % EMPTY_RING);
  if (emptyCount < EMPTY_RING) emptyCount++;
}

EmptyStats emptyErrorStats() {
  EmptyStats stats = {0, 0, 0, 0, 0};
  stats.n = emptyCount;
  if (emptyCount == 0) return stats;

  long sum = 0;
  stats.lowest = emptySamples[0];
  stats.highest = emptySamples[0];
  for (uint8_t i = 0; i < emptyCount; i++) {
    long sample = emptySamples[i];
    sum += sample;
    if (sample < stats.lowest) stats.lowest = sample;
    if (sample > stats.highest) stats.highest = sample;
  }
  stats.average = sum / (long)emptyCount;
  stats.last = emptySamples[(emptyNext + EMPTY_RING - 1) % EMPTY_RING];
  return stats;
}

static void rememberFire(FireReason reason, long errorMs, unsigned long now,
                         bool floatStillFull, bool holdArmed,
                         unsigned long holdLeft) {
  lastValid = true;
  lastReason = reason;
  lastError = errorMs;
  lastAtTop = atTopMs;
  lastRan = (unsigned long)(now - motorOnAt);
  lastFloatStillFull = floatStillFull;
  lastHoldArmed = holdArmed;
  lastHoldLeft = holdLeft;
  requestScreen();
}

// A new cycle only. Do not call this when the float returns mid-run.
static void startCycle(unsigned long now) {
  if (waitingRefill) {
    refillMs = (unsigned long)(now - inferredEmptyAt);
    refillValid = true;
    waitingRefill = false;
  }
  armTimer(maxRun, now, MAX_RUN_MS);
  disarmTimer(emptyRun);
  motorOnAt = now;
  topSince = now;
  atTopMs = 0;
  setMotor(true, now);
  state = STATE_PUMPING;
}

static void beginEmptyGuess(unsigned long now) {
  atTopMs = (unsigned long)(now - topSince);
  armTimer(emptyRun, now, EMPTY_RUNON_MS);
  state = STATE_EMPTY_RUNON;
}

static void floatReturned(unsigned long now) {
  long errorMs = timerError(now, emptyRun.startedAt, emptyRun.duration);
  rememberFire(FIRE_EARLY, errorMs, now, true, false, 0);
  earlyCount++;
  disarmTimer(emptyRun);
  topSince = now;
  atTopMs = 0;
  state = STATE_PUMPING;
}

static void guessEmpty(unsigned long now) {
  long errorMs = timerError(now, emptyRun.startedAt, emptyRun.duration);
  rememberFire(FIRE_EMPTY, errorMs, now, false, false, 0);
  pushEmptyError(errorMs);
  cycles++;
  disarmTimer(emptyRun);
  disarmTimer(maxRun);
  setMotor(false, now);
  inferredEmptyAt = now;
  waitingRefill = true;
  state = STATE_IDLE;
}

static void tripMax(unsigned long now) {
  bool holdArmed = emptyRun.armed;
  unsigned long holdLeft = holdArmed ? timerLeft(emptyRun, now) : 0;
  if (state == STATE_PUMPING) {
    atTopMs = (unsigned long)(now - topSince);
  }
  long errorMs = timerError(now, maxRun.startedAt, maxRun.duration);
  rememberFire(FIRE_MAX, errorMs, now, stableFull, holdArmed, holdLeft);
  faultCount++;
  disarmTimer(emptyRun);
  disarmTimer(maxRun);
  disarmTimer(faultClear);
  setMotor(false, now);
  state = STATE_FAULT;
  if (!stableFull) armTimer(faultClear, now, EMPTY_RUNON_MS);
}

static void serviceFault(unsigned long now) {
  if (stableFull) {
    disarmTimer(faultClear);
    return;
  }
  if (!faultClear.armed) {
    armTimer(faultClear, now, EMPTY_RUNON_MS);
    return;
  }
  if (timerDue(faultClear, now)) {
    disarmTimer(faultClear);
    state = STATE_IDLE;
  }
}

void seedFloat(unsigned long now, bool full) {
  stableFull = full;
  candidateFull = full;
  candidateSince = now;
  if (full) startCycle(now);
}

void debounceFloat(unsigned long now, bool& rose, bool& fell) {
  bool rawFull = digitalRead(FLOAT_PIN) == HIGH;
  if (rawFull != candidateFull) {
    candidateFull = rawFull;
    candidateSince = now;
    return;
  }
  if (candidateFull != stableFull &&
      (unsigned long)(now - candidateSince) >= DEBOUNCE_MS) {
    stableFull = candidateFull;
    if (stableFull) rose = true;
    else fell = true;
  }
}

void servicePump(unsigned long now, bool rose, bool fell) {
  switch (state) {
    case STATE_IDLE:
      if (rose) startCycle(now);
      break;

    case STATE_PUMPING:
      if (timerDue(maxRun, now)) tripMax(now);
      else if (fell) beginEmptyGuess(now);
      break;

    case STATE_EMPTY_RUNON:
      // The cap wins over the empty guess. A float rise wins over the
      // guess in the same pass: water is at the top, so the tank is not empty.
      if (timerDue(maxRun, now)) tripMax(now);
      else if (rose) floatReturned(now);
      else if (timerDue(emptyRun, now)) guessEmpty(now);
      break;

    case STATE_FAULT:
      serviceFault(now);
      break;
  }
}
