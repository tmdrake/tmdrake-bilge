#include "bilge.h"

RunState state = STATE_IDLE;
bool motorOn = false;
bool stableFull = false;

Timer maxRun = {false, 0, 0};
Timer emptyRun = {false, 0, 0};
Timer faultClear = {false, 0, 0};

unsigned long motorOnAt = 0;
unsigned long topSince = 0;
unsigned long atTopMs = 0;

bool waitingRefill = false;
unsigned long inferredEmptyAt = 0;
bool refillValid = false;
unsigned long refillMs = 0;

unsigned int cycles = 0;
unsigned int earlyCount = 0;
unsigned int faultCount = 0;

bool lastValid = false;
FireReason lastReason = FIRE_NONE;
long lastError = 0;
unsigned long lastAtTop = 0;
unsigned long lastRan = 0;
bool lastFloatStillFull = false;
bool lastHoldArmed = false;
unsigned long lastHoldLeft = 0;

unsigned long runCount = 0;
unsigned long totalRunMs = 0;
unsigned long thisRunStart = 0;
unsigned long lastRunMs = 0;
bool haveLastRun = false;
