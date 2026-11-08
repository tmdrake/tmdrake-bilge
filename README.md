# tmdrake-bilge

Pump timer for a dehumidifier tank. The float switch is on the top of the tank. It closes when the water reaches the top. It cannot see empty.

The sketch is `TMDRAKE_Timer.ino`. Serial monitor at **115200** baud.

## What the float means

HIGH on pin 2 means the float is up: the tank is full. The pump turns on.

When the float drops, the water left is below the switch, where this sensor cannot see it. The pump stays on for `EMPTY_RUNON_MS` (3 minutes). That is the run that empties the tank. Then the pump turns off and the sketch calls the tank empty. That call is still a guess: there is no bottom sensor.

`MAX_RUN_MS` (10 minutes) is the long timeout. One continuous run, including the 3 minute pump-down, cannot last longer than this. Ten minutes is past a normal empty, so the cap does not cut that pump-down short. A float stuck up, or a pump that never pulls the level off the top, latches the motor off (`FAULT`) until the float has been down for a full pump-down interval.

The 10 minute clock does not restart if the float bounces back up while the pump is already running.

Both limits are `unsigned long` milliseconds at the top of the sketch. `MAX_RUN_MS` has to stay longer than a normal pump-down plus `EMPTY_RUNON_MS`.

## Serial menu

115200 baud, one letter, no enter. The live screen pauses while the menu is open so the text stays put. The pump logic keeps running.

| Key | Action |
| --- | --- |
| `?` | Menu |
| `o` | How many times the motor has run, total run time, this run, last run |
| `s` | Live diagnostic screen |

`runs` counts each time the motor turns on. `odo` / `total run` is the sum of those runs since power-up. Neither value is kept in EEPROM. A power cycle starts them over. The total wraps after 49.7 days of motor-on time, not 49.7 days of wall clock.

The same `runs` and `odo` lines are on the live screen.

## How far the timer fired

The screen times each empty guess against its deadline.

- `+N ms LATE` means the pump turned off after the deadline. After this rewrite that should be a few milliseconds, not a line of serial delay.
- `float back` / `EARLY` means the float reached the top again before the guess finished. The tank was not empty. The millisecond figure is how early the float cancelled the guess.
- `max-run cap` means the forever-limit won. The motor is latched off.

`refill` is how long the float stayed down after an empty guess, until water reached the top again. A refill of a few seconds means the guess stopped the pump while the tank was still high. `since guess` counts that wait live.

`at top` is how long the float stayed up while the pump ran. A long time there means the pump is not pulling the level off the top switch.

The empty-timer average uses the last 16 empty guesses only. Float-back and max-run trips are counted beside it and are not mixed into the average.

Uptime comes from `millis()` and wraps at about 49.7 days. Cycle and fault counts wrap at 65535.

## Wiring

| Signal | Pin | Notes |
| --- | --- | --- |
| Top float | 2 | Active HIGH when the tank is full. Add a pulldown to ground so a loose wire reads LOW. |
| Pump | 13 | Onboard LED follows the pump. Use a relay or MOSFET. Do not hang the motor on the pin. |

Pin 13 stays the output so existing LED wiring still works.

## Timer wrap

`millis()` wraps every 2^32 milliseconds. The old sketch stored `timerEnd = now + duration` and treated the timer as running while `now < timerEnd`. When a falling edge landed in the last 3 minutes before the wrap, `timerEnd` wrapped to a small number, the comparison failed, and the pump turned off immediately.

Timers now store a start time, a duration, and an armed flag. A timer is due when `(now - start) >= duration`. Unsigned subtraction stays valid across the wrap. Zero is not used as a sentinel.

The loop does not call `delay()`. The float is debounced for 50 ms without blocking. The screen is sent only as fast as the serial buffer can take it, so a print cannot hold the pump past a deadline.

## Screen

One block on state change, and about once a second. Lines are not printed on every pass.

```
====== tmdrake-bilge ======
top float, HIGH = full
up 00:14  (wraps 49.7d)
runs 4  odo 12:06
empties 3  faults 0
state EMPTY-RUNON
float DOWN  below top
motor ON
at top 00:42
empty 02:18 / 03:00
max   08:36 / 10:00
------ last fire ------
empty guess +2 ms LATE
at top 00:40  ran 03:40
------ empty timer ------
last +2 ms ON TIME
avg +4 ms ON TIME
min -1  max +18  n=3
float-back 1  max-run 0
refill 18:12
since guess --
? menu   o odometer   s live
```

`ON TIME` is an error of 0 ms. Any positive error is `LATE`. Any negative error is `EARLY`.
