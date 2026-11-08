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

const uint32_t ODO_MAGIC = 0x424C4731UL;  // BLG1
const int ODO_EEPROM_ADDR = 0;

// Field order is the on-chip layout. Do not reorder or insert fields.
struct PersistedOdo {
  uint32_t magic;
  uint32_t runCount;
  uint32_t totalRunMs;
  uint32_t lastRunMs;
  uint32_t haveLastRun;
};

unsigned long odometerMs(unsigned long now) {
  unsigned long total = totalRunMs;
  if (motorOn) total += (unsigned long)(now - thisRunStart);
  return total;
}

void loadOdo() {
  PersistedOdo stored;
  EEPROM.get(ODO_EEPROM_ADDR, stored);
  if (stored.magic != ODO_MAGIC) return;
  runCount = stored.runCount;
  totalRunMs = stored.totalRunMs;
  lastRunMs = stored.lastRunMs;
  haveLastRun = stored.haveLastRun != 0;
}

static void saveOdo() {
  PersistedOdo stored;
  stored.magic = ODO_MAGIC;
  stored.runCount = runCount;
  stored.totalRunMs = totalRunMs;
  stored.lastRunMs = lastRunMs;
  stored.haveLastRun = haveLastRun ? 1UL : 0UL;
  EEPROM.put(ODO_EEPROM_ADDR, stored);
}

void setMotor(bool on, unsigned long now) {
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
  saveOdo();
}
