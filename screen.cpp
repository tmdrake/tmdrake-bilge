/*
  Serial diagnostic screen and the one-key menu.

  The live block is rebuilt on a state change and about once a second,
  then sent only as fast as the transmit buffer can take it. A print
  cannot hold the pump past a deadline.

  Text is drawn into `screen` with the put* helpers, which keep the
  format strings in flash. The menu uses Serial.print directly because
  it is one short block typed by a person.
*/

#include "bilge.h"

static char screen[768];                 // one full frame; the Uno TX buffer is only 64 bytes
static uint16_t screenLen = 0;           // bytes of the frame that are valid
static uint16_t screenPos = 0;           // how many of those bytes have been handed to the UART
static uint16_t buildPos = 0;            // write cursor while a frame is being built
static unsigned long lastScreenAt = 0;   // millis() of the last rebuild
static bool screenStarted = false;       // false until the first frame, so boot does not wait 1 s
static RunState paintedState = STATE_IDLE;  // state shown in the frame now on the wire
static bool screenPending = true;        // rebuild as soon as the previous frame has finished sending
static bool liveScreen = true;           // false while the menu is holding the display still

void requestScreen() {
  screenPending = true;  // a fire happened; show it on the next chance without waiting for the 1 s tick
}

static void putChar(char c) {
  if (buildPos + 1 < sizeof(screen)) screen[buildPos++] = c;  // drop the tail rather than overrun the buffer
}

static void putP(const char* p) {
  for (;;) {
    char c = (char)pgm_read_byte(p++);  // PSTR strings live in flash, not in the 2 KB of RAM
    if (c == '\0') break;               // C string end
    putChar(c);
  }
}

static void putUL(unsigned long v) {
  char tmp[10];                         // an unsigned long is at most 10 decimal digits
  uint8_t n = 0;
  if (v == 0) {
    putChar('0');                       // the loop below would emit nothing for zero
    return;
  }
  while (v > 0 && n < sizeof(tmp)) {
    tmp[n++] = (char)('0' + (v % 10UL));  // digits come out least-significant first
    v /= 10UL;
  }
  while (n > 0) putChar(tmp[--n]);      // reverse them into the frame
}

static void putSigned(long v) {
  if (v < 0) {
    putChar('-');                       // show the sign ourselves; the digits are unsigned
    putUL((unsigned long)(-v));         // errors here are minutes, never LONG_MIN
    return;
  }
  putUL((unsigned long)v);
}

static void put2(unsigned int v) {
  putChar((char)('0' + (v / 10U) % 10U));  // tens digit, zero-padded
  putChar((char)('0' + (v % 10U)));        // ones digit
}

static void putDur(unsigned long ms) {
  unsigned long sec = ms / 1000UL;                            // drop the milliseconds
  unsigned long hours = sec / 3600UL;
  unsigned int mins = (unsigned int)((sec / 60UL) % 60UL);
  unsigned int secs = (unsigned int)(sec % 60UL);
  if (hours > 0) {
    putUL(hours);      // hours are not zero-padded; uptime can pass 99
    putChar(':');
    put2(mins);
    putChar(':');
    put2(secs);        // H:MM:SS
  } else {
    put2(mins);
    putChar(':');
    put2(secs);        // MM:SS while the value is under an hour
  }
}

static void putErr(long v) {
  if (v > 0) putChar('+');          // a late fire shows an explicit plus
  putSigned(v);                     // an early fire already carries the minus
  putP(PSTR(" ms "));
  if (v > 0) putP(PSTR("LATE"));         // pump turned off after the deadline
  else if (v < 0) putP(PSTR("EARLY"));   // stopped before the deadline
  else putP(PSTR("ON TIME"));            // error of exactly 0 ms
}

static void putTimerLine(const char* label, const Timer& t, unsigned long now) {
  putP(label);                              // "empty", "max  ", or "clear", already padded
  if (!t.armed) {
    putP(PSTR("  --\r\n"));                  // this timer is not running
    return;
  }
  putChar(' ');
  putDur(timerLeft(t, now));                // time remaining
  putP(PSTR(" / "));
  putDur(t.duration);                       // the full allowance, so remaining has a scale
  putP(PSTR("\r\n"));
}

static void putStateName() {
  switch (state) {
    case STATE_IDLE: putP(PSTR("IDLE")); break;                  // waiting for the float
    case STATE_PUMPING: putP(PSTR("PUMPING")); break;            // float is at the top
    case STATE_EMPTY_RUNON: putP(PSTR("EMPTY-RUNON")); break;    // 3 minute guess in progress
    case STATE_FAULT: putP(PSTR("FAULT")); break;                // 10 minute cap latched the motor off
  }
}

static void putLastFire() {
  putP(PSTR("------ last fire ------\r\n"));
  if (!lastValid) {
    putP(PSTR("none yet\r\n"));    // no timer has finished since boot
    return;
  }

  switch (lastReason) {
    case FIRE_EMPTY: putP(PSTR("empty guess ")); break;    // the 3 minutes ran out
    case FIRE_EARLY: putP(PSTR("float back ")); break;     // water returned to the top first
    case FIRE_MAX: putP(PSTR("max-run cap ")); break;      // the 10 minute cap won
    default: putP(PSTR("fire ")); break;
  }
  putErr(lastError);                 // how far that event was from its deadline
  putP(PSTR("\r\n"));
  putP(PSTR("at top "));
  putDur(lastAtTop);                 // how long the float had been up
  putP(PSTR("  ran "));
  putDur(lastRan);                   // how long the motor had been on
  putP(PSTR("\r\n"));
  if (lastReason != FIRE_MAX) return;          // the extra line is only for a cap trip
  if (lastFloatStillFull) {
    putP(PSTR("float still FULL\r\n"));        // the level never left the top
  } else if (lastHoldArmed) {
    putP(PSTR("hold left "));                  // the cap cut the 3 minute guess short
    putDur(lastHoldLeft);
    putP(PSTR("\r\n"));
  }
}

static void putEmptyStats() {
  putP(PSTR("------ empty timer ------\r\n"));
  EmptyStats stats = emptyErrorStats();        // finished guesses only, not float-backs or caps
  if (stats.n == 0) {
    putP(PSTR("n=0  no empty guess yet\r\n"));
  } else {
    putP(PSTR("last "));
    putErr(stats.last);                        // the guess that just finished
    putP(PSTR("\r\n"));
    putP(PSTR("avg "));
    putErr(stats.average);                     // mean of the last 16
    putP(PSTR("\r\n"));
    putP(PSTR("min "));
    putSigned(stats.lowest);                   // most early of those 16
    putP(PSTR("  max "));
    if (stats.highest > 0) putChar('+');       // match the plus used by putErr
    putSigned(stats.highest);                  // most late of those 16
    putP(PSTR("  n="));
    putUL(stats.n);                            // 1..16
    putP(PSTR("\r\n"));
  }
  putP(PSTR("float-back "));
  putUL(earlyCount);                           // cancelled guesses, kept out of the average
  putP(PSTR("  max-run "));
  putUL(faultCount);                           // cap trips, also kept out of the average
  putP(PSTR("\r\n"));
}

static void buildScreen(unsigned long now) {
  buildPos = 0;                                              // start a new frame
  putP(PSTR("====== tmdrake-bilge ======\r\n"));
  putP(PSTR("top float, HIGH = full\r\n"));                  // remind which way the switch reads
  putP(PSTR("up "));
  putDur(now);                                               // millis() uptime
  putP(PSTR("  (wraps 49.7d)\r\n"));                         // the clock rolls over after 2^32 ms
  putP(PSTR("runs "));
  putUL(runCount);                                           // EEPROM motor-start count
  putP(PSTR("  odo "));
  putDur(odometerMs(now));                                   // saved time plus the run in progress
  putP(PSTR("\r\n"));
  putP(PSTR("empties "));
  putUL(cycles);                                             // finished guesses since boot, not saved
  putP(PSTR("  faults "));
  putUL(faultCount);                                         // cap trips since boot, not saved
  putP(PSTR("\r\n"));

  putP(PSTR("state "));
  putStateName();
  putP(PSTR("\r\n"));

  putP(PSTR("float "));
  if (stableFull) putP(PSTR("FULL  at top\r\n"));            // debounced level, not the raw pin
  else putP(PSTR("DOWN  below top\r\n"));
  putP(PSTR("motor "));
  putP(motorOn ? PSTR("ON\r\n") : PSTR("OFF\r\n"));

  if (state == STATE_PUMPING) {
    putP(PSTR("at top "));
    putDur((unsigned long)(now - topSince));                 // live clock while the float is up
    putP(PSTR("\r\n"));
  } else if (atTopMs > 0 || state == STATE_EMPTY_RUNON) {
    putP(PSTR("at top "));
    putDur(atTopMs);                                         // frozen when the float dropped
    putP(PSTR("\r\n"));
  }

  if (state == STATE_PUMPING || state == STATE_EMPTY_RUNON) {
    putTimerLine(PSTR("empty"), emptyRun, now);             // "--" during PUMPING, counting during the guess
    putTimerLine(PSTR("max  "), maxRun, now);               // the 10 minute cap
  }
  if (state == STATE_FAULT) {
    if (stableFull) putP(PSTR("clear wait, float FULL\r\n"));  // clear cannot start until the float drops
    else putTimerLine(PSTR("clear"), faultClear, now);         // quiet time left before another cycle is allowed
  }

  putLastFire();
  putEmptyStats();

  putP(PSTR("refill "));
  if (refillValid) putDur(refillMs);                       // last measured time from empty guess to FULL
  else putP(PSTR("--"));
  putP(PSTR("\r\n"));
  putP(PSTR("since guess "));
  if (waitingRefill) putDur((unsigned long)(now - inferredEmptyAt));  // live wait for the next FULL
  else putP(PSTR("--"));
  putP(PSTR("\r\n? menu   o odometer   s live\r\n\r\n"));  // blank line separates frames in a scrolling monitor

  screenLen = buildPos;   // this frame is complete
  screenPos = 0;          // nothing from it has been sent yet
}

static void printDur(unsigned long ms) {
  // Same H:MM:SS / MM:SS shape as putDur, written straight to Serial for the menu.
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
  Serial.println(F("s  live diagnostic screen"));  // resume the repeating block
  Serial.println(F("o  motor runs and odometer"));
  Serial.println(F("?  this menu"));
  Serial.println(F("Live screen is paused."));    // the pump logic keeps running
  Serial.println();
}

static void printOdometer(unsigned long now) {
  Serial.println(F("====== odometer ======"));
  Serial.print(F("runs "));
  Serial.println(runCount);                              // EEPROM count of motor starts
  Serial.print(F("total run "));
  printDur(odometerMs(now));                             // includes the run in progress
  Serial.println(F("  saved"));                          // the finished part is in EEPROM
  Serial.print(F("this run "));
  if (motorOn) printDur((unsigned long)(now - thisRunStart));
  else Serial.print(F("--"));                            // motor is off: no open run
  Serial.println();
  Serial.print(F("last run "));
  if (haveLastRun) printDur(lastRunMs);
  else Serial.print(F("--"));                            // no run has finished yet
  Serial.println();
  Serial.print(F("empties "));
  Serial.print(cycles);                                  // since boot only
  Serial.print(F("  float-back "));
  Serial.print(earlyCount);
  Serial.print(F("  faults "));
  Serial.println(faultCount);
  Serial.println(F("? menu   s live"));
  Serial.println();
}

void handleKey(char c, unsigned long now) {
  if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');  // accept S, O, M, H as well
  if (c == 's') {
    screenPos = screenLen;          // drop any half-sent frame so the new one starts clean
    liveScreen = true;              // the repeating block may run again
    screenPending = true;           // draw it immediately
    screenStarted = false;          // do not wait out the remainder of the 1 s period
    return;
  }
  // Ignore anything else so line noise cannot pause the live screen.
  if (c != '?' && c != 'o' && c != 'h' && c != 'm') return;
  screenPos = screenLen;            // abandon the live frame
  liveScreen = false;               // stop the 1 Hz redraw so this text stays put
  Serial.println();                 // start the menu on its own line
  if (c == 'o') printOdometer(now);
  else printMenu();                 // ?, h, and m all show the menu
}

void serviceScreen(unsigned long now) {
  if (!liveScreen) return;                              // menu is up; do not scroll it away
  if (state != paintedState) screenPending = true;     // a state change needs a new frame
  if (screenPos >= screenLen) {                        // previous frame is fully queued
    bool due = !screenStarted ||
               (unsigned long)(now - lastScreenAt) >= SCREEN_MS;  // or a second has passed
    if (due || screenPending) {
      buildScreen(now);                                // snapshot the numbers after the pump update
      screenStarted = true;
      lastScreenAt = now;
      paintedState = state;                            // this frame matches the state we just drew
      screenPending = false;
    }
  }

  if (screenPos >= screenLen) return;                  // nothing waiting, or the frame was just finished
  int space = Serial.availableForWrite();              // bytes the 64-byte TX buffer can take without blocking
  if (space <= 0) return;                              // UART still busy; the pump loop must not wait
  uint16_t left = (uint16_t)(screenLen - screenPos);
  if ((uint16_t)space > left) space = (int)left;       // do not read past the end of the frame
  size_t wrote = Serial.write((const uint8_t*)screen + screenPos, (size_t)space);
  screenPos = (uint16_t)(screenPos + wrote);          // advance by what the driver actually accepted
}
