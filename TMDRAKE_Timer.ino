/*
  tmdrake-bilge

  Dehumidifier tank pump timer. The float switch is on the top of the
  tank, active HIGH when the water reaches it. HIGH means full. The
  switch cannot see the bottom, so "empty" is a guess made by a timer
  after the float drops.

  Pump (motor) output:
    - Float rises: pump ON. The tank is at the top.
    - Float drops: keep pumping for EMPTY_RUNON_MS (3 minutes). That is
      how long the motor needs to empty the water below the switch.
      Then call the tank empty and turn the pump OFF. Empty is still a
      guess: this float cannot see the bottom.
    - MAX_RUN_MS caps one continuous run, including that 3 minute
      pump-down. A float stuck up, or a pump that never pulls the level
      off the top, must not run the motor forever. The cap latches
      FAULT until the float has been down for a full pump-down interval.

  The max-run clock is not restarted if the float pops back up while
  the pump is already on. Restarting it would let a bouncing float
  run the motor forever.

  millis() wraps every 49.7 days. Timers store a start and a duration
  and test (now - start) >= duration. That unsigned subtraction stays
  correct across the wrap. Do not compare two absolute deadlines, and
  do not use 0 as an "not armed" sentinel.

  Serial diagnostic screen: 115200 baud. It records how far the empty
  guess fired from its deadline, and how long the tank took to reach
  the top again after a guess. A very short refill means the 3 minute
  pump-down finished while water was still high.

  Keys, one letter, no enter:
    ?  menu (pauses the live screen)
    o  how many times the motor ran, and the run-time odometer
    s  live screen again
  The odometer is since power-up. It is not stored in EEPROM.

  Pin 13 is the onboard LED. Drive the real pump with a relay or
  MOSFET. Put a pulldown on the float pin so a disconnected switch
  reads LOW (not full). An Uno has no internal pulldown, and a pullup
  would fight this active-HIGH float.
*/

const int FLOAT_PIN = 2;             // active HIGH: float up, tank at the top
const int MOTOR_PIN = LED_BUILTIN;   // pin 13, relay/MOSFET, not the pump itself

// Pump-down after the top float drops. The switch has opened, and the
// motor still needs about 3 minutes to empty the tank below it.
const unsigned long EMPTY_RUNON_MS = 3UL * 60UL * 1000UL;

// Forever-cap on one continuous run, including the 3 minute pump-down.
// Longer than a normal empty, so it does not cut that pump-down short.
const unsigned long MAX_RUN_MS = 10UL * 60UL * 1000UL;

const unsigned long DEBOUNCE_MS = 50UL;
const unsigned long SCREEN_MS = 1000UL;
const uint8_t EMPTY_RING = 16;

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

struct Timer {
  bool armed;
  unsigned long startedAt;
  unsigned long duration;
};

// True when a timer has reached its duration. Wrap-safe.
static bool timerDue(const Timer& t, unsigned long now) {
  return t.armed && (unsigned long)(now - t.startedAt) >= t.duration;
}

static unsigned long timerLeft(const Timer& t, unsigned long now) {
  unsigned long elapsed = (unsigned long)(now - t.startedAt);
  if (elapsed >= t.duration) return 0;
  return t.duration - elapsed;
}

// Positive: output changed after the deadline. Negative: stopped early.
static long timerError(unsigned long firedAt, unsigned long startedAt,
                       unsigned long duration) {
  return (long)(firedAt - startedAt) - (long)duration;
}

static RunState state = STATE_IDLE;
static bool motorOn = false;
static bool stableFull = false;
static bool candidateFull = false;
static unsigned long candidateSince = 0;

static Timer maxRun = {false, 0, 0};
static Timer emptyRun = {false, 0, 0};
static Timer faultClear = {false, 0, 0};

static unsigned long motorOnAt = 0;
static unsigned long topSince = 0;
static unsigned long atTopMs = 0;

static bool waitingRefill = false;
static unsigned long inferredEmptyAt = 0;
static bool refillValid = false;
static unsigned long refillMs = 0;

static unsigned int cycles = 0;
static unsigned int earlyCount = 0;
static unsigned int faultCount = 0;

static bool lastValid = false;
static FireReason lastReason = FIRE_NONE;
static long lastError = 0;
static unsigned long lastAtTop = 0;
static unsigned long lastRan = 0;
static bool lastFloatStillFull = false;
static bool lastHoldArmed = false;
static unsigned long lastHoldLeft = 0;

static long emptyRing[EMPTY_RING];
static uint8_t emptyRingCount = 0;
static uint8_t emptyRingNext = 0;

static char screen[768];
static uint16_t screenLen = 0;
static uint16_t screenPos = 0;
static uint16_t buildPos = 0;
static unsigned long lastScreenAt = 0;
static bool screenStarted = false;
static RunState paintedState = STATE_IDLE;
static bool screenPending = true;
static bool liveScreen = true;

// Since power-up. totalRunMs does not include the in-progress run.
// Each piece is added with unsigned subtraction, so a millis() wrap
// during a run does not lose or inflate that run. The sum itself
// wraps after 49.7 days of motor-on time.
static unsigned long runCount = 0;
static unsigned long totalRunMs = 0;
static unsigned long thisRunStart = 0;
static unsigned long lastRunMs = 0;
static bool haveLastRun = false;

static unsigned long odometerMs(unsigned long now) {
  unsigned long total = totalRunMs;
  if (motorOn) total += (unsigned long)(now - thisRunStart);
  return total;
}

static void setMotor(bool on, unsigned long now) {
  if (on == motorOn) return;
  if (motorOn) {
    unsigned long ran = (unsigned long)(now - thisRunStart);
    totalRunMs += ran;
    lastRunMs = ran;
    haveLastRun = true;
  }
  if (on) {
    runCount++;
    thisRunStart = now;
  }
  motorOn = on;
  digitalWrite(MOTOR_PIN, on ? HIGH : LOW);
}

static void disarm(Timer& t) {
  t.armed = false;
}

static void arm(Timer& t, unsigned long now, unsigned long duration) {
  t.armed = true;
  t.startedAt = now;
  t.duration = duration;
}

static void pushEmptyError(long errorMs) {
  emptyRing[emptyRingNext] = errorMs;
  emptyRingNext = (uint8_t)((emptyRingNext + 1) % EMPTY_RING);
  if (emptyRingCount < EMPTY_RING) emptyRingCount++;
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
  screenPending = true;
}

// New cycle only. Do not call this when the float returns during a run.
static void startCycle(unsigned long now) {
  if (waitingRefill) {
    refillMs = (unsigned long)(now - inferredEmptyAt);
    refillValid = true;
    waitingRefill = false;
  }
  arm(maxRun, now, MAX_RUN_MS);
  disarm(emptyRun);
  motorOnAt = now;
  topSince = now;
  atTopMs = 0;
  setMotor(true, now);
  state = STATE_PUMPING;
}

static void beginEmptyGuess(unsigned long now) {
  atTopMs = (unsigned long)(now - topSince);
  arm(emptyRun, now, EMPTY_RUNON_MS);
  state = STATE_EMPTY_RUNON;
}

static void floatReturned(unsigned long now) {
  long errorMs = timerError(now, emptyRun.startedAt, emptyRun.duration);
  rememberFire(FIRE_EARLY, errorMs, now, true, false, 0);
  earlyCount++;
  disarm(emptyRun);
  topSince = now;
  atTopMs = 0;
  state = STATE_PUMPING;
}

static void guessEmpty(unsigned long now) {
  long errorMs = timerError(now, emptyRun.startedAt, emptyRun.duration);
  rememberFire(FIRE_EMPTY, errorMs, now, false, false, 0);
  pushEmptyError(errorMs);
  cycles++;
  disarm(emptyRun);
  disarm(maxRun);
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
  disarm(emptyRun);
  disarm(maxRun);
  disarm(faultClear);
  setMotor(false, now);
  state = STATE_FAULT;
  if (!stableFull) arm(faultClear, now, EMPTY_RUNON_MS);
}

static void serviceFault(unsigned long now) {
  if (stableFull) {
    disarm(faultClear);
    return;
  }
  if (!faultClear.armed) {
    arm(faultClear, now, EMPTY_RUNON_MS);
    return;
  }
  if (timerDue(faultClear, now)) {
    disarm(faultClear);
    state = STATE_IDLE;
  }
}

static void debounce(unsigned long now, bool& rose, bool& fell) {
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

static void service(unsigned long now, bool rose, bool fell) {
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
      // guess when both are true in the same pass: water is at the top,
      // so the tank is not empty.
      if (timerDue(maxRun, now)) tripMax(now);
      else if (rose) floatReturned(now);
      else if (timerDue(emptyRun, now)) guessEmpty(now);
      break;

    case STATE_FAULT:
      serviceFault(now);
      break;
  }
}

static void putChar(char c) {
  if (buildPos + 1 < sizeof(screen)) screen[buildPos++] = c;
}

static void putP(const char* p) {
  for (;;) {
    char c = (char)pgm_read_byte(p++);
    if (c == '\0') break;
    putChar(c);
  }
}

static void putUL(unsigned long v) {
  char tmp[10];
  uint8_t n = 0;
  if (v == 0) {
    putChar('0');
    return;
  }
  while (v > 0 && n < sizeof(tmp)) {
    tmp[n++] = (char)('0' + (v % 10UL));
    v /= 10UL;
  }
  while (n > 0) putChar(tmp[--n]);
}

static void putSigned(long v) {
  if (v < 0) {
    putChar('-');
    putUL((unsigned long)(-v));
    return;
  }
  putUL((unsigned long)v);
}

static void put2(unsigned int v) {
  putChar((char)('0' + (v / 10U) % 10U));
  putChar((char)('0' + (v % 10U)));
}

static void putDur(unsigned long ms) {
  unsigned long sec = ms / 1000UL;
  unsigned long hours = sec / 3600UL;
  unsigned int mins = (unsigned int)((sec / 60UL) % 60UL);
  unsigned int secs = (unsigned int)(sec % 60UL);
  if (hours > 0) {
    putUL(hours);
    putChar(':');
    put2(mins);
    putChar(':');
    put2(secs);
  } else {
    put2(mins);
    putChar(':');
    put2(secs);
  }
}

static void putErr(long v) {
  if (v > 0) putChar('+');
  putSigned(v);
  putP(PSTR(" ms "));
  if (v > 0) putP(PSTR("LATE"));
  else if (v < 0) putP(PSTR("EARLY"));
  else putP(PSTR("ON TIME"));
}

static void putTimerLine(const char* label, const Timer& t, unsigned long now) {
  putP(label);
  if (!t.armed) {
    putP(PSTR("  --\r\n"));
    return;
  }
  putChar(' ');
  putDur(timerLeft(t, now));
  putP(PSTR(" / "));
  putDur(t.duration);
  putP(PSTR("\r\n"));
}

static void buildScreen(unsigned long now) {
  buildPos = 0;
  putP(PSTR("====== tmdrake-bilge ======\r\n"));
  putP(PSTR("top float, HIGH = full\r\n"));
  putP(PSTR("up "));
  putDur(now);
  putP(PSTR("  (wraps 49.7d)\r\n"));
  putP(PSTR("runs "));
  putUL(runCount);
  putP(PSTR("  odo "));
  putDur(odometerMs(now));
  putP(PSTR("\r\n"));
  putP(PSTR("empties "));
  putUL(cycles);
  putP(PSTR("  faults "));
  putUL(faultCount);
  putP(PSTR("\r\n"));

  putP(PSTR("state "));
  switch (state) {
    case STATE_IDLE: putP(PSTR("IDLE")); break;
    case STATE_PUMPING: putP(PSTR("PUMPING")); break;
    case STATE_EMPTY_RUNON: putP(PSTR("EMPTY-RUNON")); break;
    case STATE_FAULT: putP(PSTR("FAULT")); break;
  }
  putP(PSTR("\r\n"));

  putP(PSTR("float "));
  if (stableFull) putP(PSTR("FULL  at top\r\n"));
  else putP(PSTR("DOWN  below top\r\n"));
  putP(PSTR("motor "));
  putP(motorOn ? PSTR("ON\r\n") : PSTR("OFF\r\n"));

  if (state == STATE_PUMPING) {
    putP(PSTR("at top "));
    putDur((unsigned long)(now - topSince));
    putP(PSTR("\r\n"));
  } else if (atTopMs > 0 || state == STATE_EMPTY_RUNON) {
    putP(PSTR("at top "));
    putDur(atTopMs);
    putP(PSTR("\r\n"));
  }

  if (state == STATE_PUMPING || state == STATE_EMPTY_RUNON) {
    putTimerLine(PSTR("empty"), emptyRun, now);
    putTimerLine(PSTR("max  "), maxRun, now);
  }
  if (state == STATE_FAULT) {
    if (stableFull) putP(PSTR("clear wait, float FULL\r\n"));
    else putTimerLine(PSTR("clear"), faultClear, now);
  }

  putP(PSTR("------ last fire ------\r\n"));
  if (!lastValid) {
    putP(PSTR("none yet\r\n"));
  } else {
    switch (lastReason) {
      case FIRE_EMPTY: putP(PSTR("empty guess ")); break;
      case FIRE_EARLY: putP(PSTR("float back ")); break;
      case FIRE_MAX: putP(PSTR("max-run cap ")); break;
      default: putP(PSTR("fire ")); break;
    }
    putErr(lastError);
    putP(PSTR("\r\n"));
    putP(PSTR("at top "));
    putDur(lastAtTop);
    putP(PSTR("  ran "));
    putDur(lastRan);
    putP(PSTR("\r\n"));
    if (lastReason == FIRE_MAX) {
      if (lastFloatStillFull) putP(PSTR("float still FULL\r\n"));
      else if (lastHoldArmed) {
        putP(PSTR("hold left "));
        putDur(lastHoldLeft);
        putP(PSTR("\r\n"));
      }
    }
  }

  putP(PSTR("------ empty timer ------\r\n"));
  if (emptyRingCount == 0) {
    putP(PSTR("n=0  no empty guess yet\r\n"));
  } else {
    long sum = 0;
    long mn = emptyRing[0];
    long mx = emptyRing[0];
    for (uint8_t i = 0; i < emptyRingCount; i++) {
      long e = emptyRing[i];
      sum += e;
      if (e < mn) mn = e;
      if (e > mx) mx = e;
    }
    long avg = sum / (long)emptyRingCount;
    putP(PSTR("last "));
    putErr(emptyRing[(emptyRingNext + EMPTY_RING - 1) % EMPTY_RING]);
    putP(PSTR("\r\n"));
    putP(PSTR("avg "));
    putErr(avg);
    putP(PSTR("\r\n"));
    putP(PSTR("min "));
    putSigned(mn);
    putP(PSTR("  max "));
    if (mx > 0) putChar('+');
    putSigned(mx);
    putP(PSTR("  n="));
    putUL(emptyRingCount);
    putP(PSTR("\r\n"));
  }
  putP(PSTR("float-back "));
  putUL(earlyCount);
  putP(PSTR("  max-run "));
  putUL(faultCount);
  putP(PSTR("\r\n"));

  putP(PSTR("refill "));
  if (refillValid) putDur(refillMs);
  else putP(PSTR("--"));
  putP(PSTR("\r\n"));
  putP(PSTR("since guess "));
  if (waitingRefill) putDur((unsigned long)(now - inferredEmptyAt));
  else putP(PSTR("--"));
  putP(PSTR("\r\n? menu   o odometer   s live\r\n\r\n"));

  screenLen = buildPos;
  screenPos = 0;
}

static void printDur(unsigned long ms) {
  unsigned long sec = ms / 1000UL;
  unsigned long hours = sec / 3600UL;
  unsigned int mins = (unsigned int)((sec / 60UL) % 60UL);
  unsigned int secs = (unsigned int)(sec % 60UL);
  if (hours > 0) {
    Serial.print(hours);
    Serial.print(':');
  }
  if (mins < 10) Serial.print('0');
  Serial.print(mins);
  Serial.print(':');
  if (secs < 10) Serial.print('0');
  Serial.print(secs);
}

static void printMenu() {
  Serial.println(F("====== menu ======"));
  Serial.println(F("s  live diagnostic screen"));
  Serial.println(F("o  motor runs and odometer"));
  Serial.println(F("?  this menu"));
  Serial.println(F("Live screen is paused."));
  Serial.println();
}

static void printOdometer(unsigned long now) {
  Serial.println(F("====== odometer ======"));
  Serial.print(F("runs "));
  Serial.println(runCount);
  Serial.print(F("total run "));
  printDur(odometerMs(now));
  Serial.println(F("  since power-up"));
  Serial.print(F("this run "));
  if (motorOn) printDur((unsigned long)(now - thisRunStart));
  else Serial.print(F("--"));
  Serial.println();
  Serial.print(F("last run "));
  if (haveLastRun) printDur(lastRunMs);
  else Serial.print(F("--"));
  Serial.println();
  Serial.print(F("empties "));
  Serial.print(cycles);
  Serial.print(F("  float-back "));
  Serial.print(earlyCount);
  Serial.print(F("  faults "));
  Serial.println(faultCount);
  Serial.println(F("? menu   s live"));
  Serial.println();
}

static void handleKey(char c, unsigned long now) {
  if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
  if (c == 's') {
    screenPos = screenLen;
    liveScreen = true;
    screenPending = true;
    screenStarted = false;
    return;
  }
  // Ignore anything else so line noise cannot pause the live screen.
  if (c != '?' && c != 'o' && c != 'h' && c != 'm') return;
  screenPos = screenLen;
  liveScreen = false;
  Serial.println();
  if (c == 'o') printOdometer(now);
  else printMenu();
}

static void serviceScreen(unsigned long now) {
  if (!liveScreen) return;
  if (state != paintedState) screenPending = true;
  if (screenPos >= screenLen) {
    bool due = !screenStarted ||
               (unsigned long)(now - lastScreenAt) >= SCREEN_MS;
    if (due || screenPending) {
      buildScreen(now);
      screenStarted = true;
      lastScreenAt = now;
      paintedState = state;
      screenPending = false;
    }
  }

  if (screenPos >= screenLen) return;
  int space = Serial.availableForWrite();
  if (space <= 0) return;
  uint16_t left = (uint16_t)(screenLen - screenPos);
  if ((uint16_t)space > left) space = (int)left;
  size_t wrote = Serial.write((const uint8_t*)screen + screenPos, (size_t)space);
  screenPos = (uint16_t)(screenPos + wrote);
}

void setup() {
  Serial.begin(115200);
  pinMode(FLOAT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  unsigned long now = millis();
  bool full = digitalRead(FLOAT_PIN) == HIGH;
  stableFull = full;
  candidateFull = full;
  candidateSince = now;
  if (full) startCycle(now);

  Serial.println(F("tmdrake-bilge 115200  ? for menu"));
}

void loop() {
  unsigned long now = millis();
  bool rose = false;
  bool fell = false;
  debounce(now, rose, fell);
  service(now, rose, fell);
  if (Serial.available() > 0) handleKey((char)Serial.read(), now);
  serviceScreen(now);
}
