# krell architecture

How the sketch is put together. The README says what the module does and why; this says where the
code is and which invariants hold. `krell.ino` is ~615 lines, single file, no interrupts.

## Shape

Everything is cooperative. `loop()` (`krell.ino:605`) runs flat out; the engine is gated to 4 kHz
at the bottom of it, and everything above the gate runs at full loop rate — many kHz.

```
loop()
  serviceADC()      one ADC channel per pass, round robin A0..A3, 3/4 IIR
  servicePots()     adcVal[0..2] -> potVal[0..2], unless frozen
  serviceButton()   debounce, gestures, and serviceShift() while down
  servicePickup()   only while the button is UP
  serviceLED()      flashes > shift display > envelope
  EEPROM flush      2 s after the last markDirty()
  --- 250 us gate ---
  outputTick()      the engine: one step of rise / fall / rest
```

Two clocks. `millis()` drives everything human-scale (debounce, hold thresholds, flashes, the
EEPROM delay). `micros()` drives the engine tick, with `lastTickUs += TICK_US` so the period does
not drift, and a catch-up clamp if the loop ever falls four ticks behind.

Timer0 and `millis()` are left alone. Timer1 (F2, F3) and Timer2 (F4, LED) are reconfigured to
62.5 kHz fast PWM in `configurePWM()` (`:409`), which is why `analogWrite()` must never be used.

## The signal path

```
POT1 ──┐
       ├─ baseIndex() ─┐                        every tick, live
F1  ───┘               │
                       ├─ +skewIdx (rise) ─┬─ incFromIndex() ─ phase += inc ─┐
riseOff/fallOff ───────┤  -skewIdx (fall)  │                                 │
POT3 ─ REST_SPAN ──────┘  (rest only)      │                                 │
                                           │                    riseShape() / fallShape()
                                           │                                 │
                                           └─────────────────────────── env ─┴─ OCR2A  (F4)
                                                                              └─ LED
rollCycle() at the start of each rise:
  POT2 ─ timeGain ─ randOffset() x3 ─ riseOff, fallOff, restOff   (latched for the cycle)
  POT2 ─ cvGain   ─ rollCv() ─ quantCode() ─ OCR1B                (F3, held for the cycle)
                             └─ pulseTicks = 40 ─ OCR1A           (F2, 10 ms)
```

**The central trick**: times are held as an *index* into an exponential table, never as
milliseconds. Randomising, skewing and the rest offset are all plain adds in that index, and all
three are therefore log-uniform / symmetric-in-ratio for free. `incFromIndex()` (`:231`) is the only
place the index becomes a rate.

## State ownership

| State | Written by | Read by |
|---|---|---|
| `adcVal[0..3]` | `serviceADC()` | `servicePots()`, `serviceShift()`, `servicePickup()`, `baseIndex()` (F1 only) |
| `potVal[0..2]` | `servicePots()` | the engine only — `baseIndex()`, `randOffset()`, `rollCv()`, `currentInc()`, `outputTick()` |
| `frozen/armed/potEntry/potSeen/entrySign` | the shift layer | the shift layer |
| `gCurve gSkew gBal gScale` | `serviceButton()`, `serviceShift()`, `loadSettings()` | everywhere |
| `skewIdx timeGain cvGain` | `applySkew()`, `applyBalance()` | the engine |
| `phase stage env riseOff fallOff restOff cycleCv pulseTicks` | `outputTick()`, `rollCycle()` | `outputTick()`, `serviceLED()` |

**The `adcVal` / `potVal` split is the load-bearing one.** The engine must read `potVal` and the
shift layer must read `adcVal`; getting this backwards means either the shift layer cannot see the
knob move, or turning a knob in the shift layer also changes the engine. Precomputing `skewIdx`,
`timeGain` and `cvGain` keeps the per-tick path to adds and one table lookup.

## The state machine

`stage` is `S_RISE → S_FALL → S_REST → S_RISE`, advanced in `outputTick()` (`:575`) by a 32-bit
phase accumulator that ends a stage **on the wrap**. The largest increment in `INC_TAB` is 2³²/20,
so a stage can never wrap inside a single tick, and the wrap is therefore a reliable end-of-stage.

`rollCycle()` (`:553`) is the **only** entry point for a cycle. It is called from `setup()`, from
the re-roll gesture, and from the two wrap sites in `outputTick()`. Everything for the cycle is
decided there, which is what guarantees F2 and F3 change on the same edge.

`REST_OFF` at the bottom of POT3 skips `S_REST` entirely, so the fall runs straight into the next
rise — and that is the setting where F2 lands on the literal end of the cycle.

## The button

One flag per press, both cleared on the press edge:

- `vlongFired` — the 1.5 s re-roll has already fired
- `shiftUsed` — this hold was spent on a pot

```
press    -> btnDownMs = now; vlongFired = false; shiftUsed = false; freezePots()
held     -> serviceShift() every pass; at 1.5 s, if !vlongFired && !shiftUsed, re-roll
release  -> releasePots(); if !vlongFired && !shiftUsed, cycle the curve
```

The curve tap lives on the **release** with no short-press threshold, so any release that has not
already fired is a tap. That is why `shiftUsed` is required rather than merely tidy: without it,
every shift gesture would also bump the curve on release.

## Invariants

Break any of these and the module misbehaves in a way that is hard to hear as a bug:

1. **`servicePickup()` runs only while the button is up.** `entrySign[]` is set at release; running
   it during a hold reads a stale sign and unfreezes a pot mid-edit.
2. **A pot must travel `ARM_DELTA` before it takes effect.** This is what makes "a hold and release
   cannot change anything" true, which the README states as a guarantee.
3. **F1 is never frozen.** A patched CV keeps bending the envelope while the user edits.
4. **`rollCycle()` is the only place a cycle starts**, so F2 and F3 never drift apart.
5. **`quantCode()` breaks ties on codes, not semitone numbers.** Rounding to a whole semitone loses
   the fraction that decides which of the two equidistant candidates is nearer. The failure is
   silent and can land a whole octave away.
6. **Every EEPROM default is the pre-shift-layer behaviour**, so bumping the stamp cannot change how
   an existing user's module sounds.

## Budget

5458 B flash (17 %), 109 B RAM (5 %), of 30720 / 2048.

The engine tick is the only hard deadline: 250 µs at 16 MHz is 4000 cycles, and `outputTick()` is
a handful of adds, one 32-bit interpolated table read and two shape lookups. No floating point, no
division in the tick path, no `analogRead()` after `setup()`.

## Verification

`tools/build.sh krell` compiles `--clean --warnings all` and prints the flash/RAM figures.

There is no committed test harness — the repo has none, by the single-file-sketch constraint. The
host sim used to check this firmware stubs the AVR registers and `#include`s `krell.ino` unmodified,
then asserts over the quantiser (all 256 codes × 6 scales: in-scale, monotonic, and exactly the
nearest allowed code), the skew and balance maths, the freeze/pickup machine, the button gestures
and the EEPROM round trip. Rebuild it in a scratch directory when changing any of those; it is
about 200 lines and it caught the tie-breaking bug in invariant 5.
