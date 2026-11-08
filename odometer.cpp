/*
  Run count and motor-on time.

  Saved when the motor changes, not while it is running. EEPROM.put
  writes only the bytes that changed. A few pump cycles a day is far
  under the Uno's 100,000 writes per cell.

  A power cut during a run keeps the count (it was saved at start) and
  drops the unfinished minutes (those are saved at stop).
*/

#include <EEPROM.h>

#include "bilge.h"

const uint32_t ODO_MAGIC = 0x424C4731UL;  // 'BLG1': marks a record this sketch wrote
const int ODO_EEPROM_ADDR = 0;            // first byte of the 1 KB EEPROM

// Field order is the on-chip layout. Do not reorder or insert fields,
// or a board that already has a record will load the numbers in the wrong slots.
struct PersistedOdo {
  uint32_t magic;        // ODO_MAGIC, or anything else means "blank"
  uint32_t runCount;     // motor starts
  uint32_t totalRunMs;   // sum of finished runs
  uint32_t lastRunMs;    // duration of the latest finished run
  uint32_t haveLastRun;  // 1 after one run has finished, else 0
};

unsigned long odometerMs(unsigned long now) {
  unsigned long total = totalRunMs;                         // finished runs only
  if (motorOn) total += (unsigned long)(now - thisRunStart);  // plus the run still going
  return total;                                            // wraps after 49.7 days of motor-on time
}

void loadOdo() {
  PersistedOdo stored;
  EEPROM.get(ODO_EEPROM_ADDR, stored);     // read sizeof(PersistedOdo) bytes from address 0
  if (stored.magic != ODO_MAGIC) return;   // blank or foreign data: keep the zeros from startup
  runCount = stored.runCount;              // motor starts survive power loss
  totalRunMs = stored.totalRunMs;          // finished runtime survives power loss
  lastRunMs = stored.lastRunMs;            // last completed run, shown by the o key
  haveLastRun = stored.haveLastRun != 0;   // stored as 0 or 1, not as a C++ bool
}

static void saveOdo() {
  PersistedOdo stored;
  stored.magic = ODO_MAGIC;                              // so the next boot accepts the record
  stored.runCount = runCount;
  stored.totalRunMs = totalRunMs;
  stored.lastRunMs = lastRunMs;
  stored.haveLastRun = haveLastRun ? 1UL : 0UL;          // keep the slot a fixed 4 bytes
  EEPROM.put(ODO_EEPROM_ADDR, stored);                   // skips each byte that already matches
}

void setMotor(bool on, unsigned long now) {
  if (on == motorOn) return;                             // already in the requested state: do not count again
  if (motorOn) {                                         // ON to OFF: close out the run that just finished
    unsigned long ran = (unsigned long)(now - thisRunStart);  // wrap-safe length of this run
    totalRunMs += ran;                                   // add it to the saved odometer
    lastRunMs = ran;                                     // remember it as the last completed run
    haveLastRun = true;
  }
  if (on) {                                              // OFF to ON: this start counts immediately
    runCount++;                                          // kept even if power dies before the stop
    thisRunStart = now;                                  // the unfinished minutes are measured from here
  }
  motorOn = on;                                          // remember the new pin state
  digitalWrite(MOTOR_PIN, on ? HIGH : LOW);             // pin 13, and the relay on that pin
  saveOdo();                                             // write the count at start, or the added time at stop
}
