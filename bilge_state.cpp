// Starting values. loadOdo() overwrites the odometer fields if EEPROM
// holds a record written by this sketch.

#include "bilge.h"

RunState state = STATE_IDLE;   // motor off until the float reads FULL
bool motorOn = false;          // pin 13 starts LOW in setup(), before any cycle
bool stableFull = false;       // seedFloat() replaces this with the boot reading

Timer maxRun = {false, 0, 0};      // not armed until a cycle starts
Timer emptyRun = {false, 0, 0};    // not armed until the float drops
Timer faultClear = {false, 0, 0};  // not armed until a cap trip with the float down

unsigned long motorOnAt = 0;  // meaningless until the motor turns on
unsigned long topSince = 0;   // meaningless until the float is FULL
unsigned long atTopMs = 0;    // 0 means "the float has not dropped this cycle"

bool waitingRefill = false;        // no empty guess has finished yet
unsigned long inferredEmptyAt = 0;
bool refillValid = false;          // the refill line shows "--" until the first one
unsigned long refillMs = 0;

unsigned int cycles = 0;      // cleared by reset; not stored in EEPROM
unsigned int earlyCount = 0;
unsigned int faultCount = 0;

bool lastValid = false;                 // the "last fire" block shows "none yet"
FireReason lastReason = FIRE_NONE;
long lastError = 0;
unsigned long lastAtTop = 0;
unsigned long lastRan = 0;
bool lastFloatStillFull = false;
bool lastHoldArmed = false;
unsigned long lastHoldLeft = 0;

unsigned long runCount = 0;      // loadOdo() replaces these when the magic matches
unsigned long totalRunMs = 0;
unsigned long thisRunStart = 0;
unsigned long lastRunMs = 0;
bool haveLastRun = false;
