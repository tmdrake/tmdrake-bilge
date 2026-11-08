/*
  Serial diagnostic screen and the one-key menu.

  The live block is rebuilt on a state change and about once a second,
  then sent only as fast as the transmit buffer can take it. A print
  cannot hold the pump past a deadline.
*/

#include "bilge.h"

static char screen[768];
static uint16_t screenLen = 0;
static uint16_t screenPos = 0;
static uint16_t buildPos = 0;
static unsigned long lastScreenAt = 0;
static bool screenStarted = false;
static RunState paintedState = STATE_IDLE;
static bool screenPending = true;
static bool liveScreen = true;

void requestScreen() {
  screenPending = true;
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

static void putStateName() {
  switch (state) {
    case STATE_IDLE: putP(PSTR("IDLE")); break;
    case STATE_PUMPING: putP(PSTR("PUMPING")); break;
    case STATE_EMPTY_RUNON: putP(PSTR("EMPTY-RUNON")); break;
    case STATE_FAULT: putP(PSTR("FAULT")); break;
  }
}

static void putLastFire() {
  putP(PSTR("------ last fire ------\r\n"));
  if (!lastValid) {
    putP(PSTR("none yet\r\n"));
    return;
  }

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
  if (lastReason != FIRE_MAX) return;
  if (lastFloatStillFull) {
    putP(PSTR("float still FULL\r\n"));
  } else if (lastHoldArmed) {
    putP(PSTR("hold left "));
    putDur(lastHoldLeft);
    putP(PSTR("\r\n"));
  }
}

static void putEmptyStats() {
  putP(PSTR("------ empty timer ------\r\n"));
  EmptyStats stats = emptyErrorStats();
  if (stats.n == 0) {
    putP(PSTR("n=0  no empty guess yet\r\n"));
  } else {
    putP(PSTR("last "));
    putErr(stats.last);
    putP(PSTR("\r\n"));
    putP(PSTR("avg "));
    putErr(stats.average);
    putP(PSTR("\r\n"));
    putP(PSTR("min "));
    putSigned(stats.lowest);
    putP(PSTR("  max "));
    if (stats.highest > 0) putChar('+');
    putSigned(stats.highest);
    putP(PSTR("  n="));
    putUL(stats.n);
    putP(PSTR("\r\n"));
  }
  putP(PSTR("float-back "));
  putUL(earlyCount);
  putP(PSTR("  max-run "));
  putUL(faultCount);
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
  putStateName();
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

  putLastFire();
  putEmptyStats();

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
  Serial.println(F("  saved"));
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

void handleKey(char c, unsigned long now) {
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

void serviceScreen(unsigned long now) {
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
