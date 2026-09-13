# MOD1 UI elements

A catalogue of the panel-interface building blocks used across the six firmwares here, with the
code, the constants, and which sketches each one comes from.

**These are copy-paste elements, not a library.** Every sketch in this repo is a single `.ino` so
that it works in the Arduino IDE unchanged, which means there is no shared header to include. The
convention is to copy the idiom in verbatim, keep the constant names and values identical, and
change only the payload. `ledSet()` exists as six copies on purpose. When you change one of these
for good reason, change it here too and say which sketches were left behind.

The reason the values are worth keeping identical is that the modules sit in the same rack. A user
who learns that a pot needs a deliberate turn before it does anything, or that a moved knob has to
be turned back to where it was, should not have to relearn it per module.

## Index

**Input**
- [1. Non-blocking ADC round-robin](#1-non-blocking-adc-round-robin) — read pots without `analogRead()`
- [2. Button debouncer](#2-button-debouncer) — the skeleton all six share
- [3. Gesture vocabulary](#3-gesture-vocabulary) — tap, short/long release, fires-while-held
- [4. Shift layer A: latching](#4-shift-layer-a-latching) — hold + turn, one pot carries two tables
- [5. Shift layer B: modal edit page](#5-shift-layer-b-modal-edit-page) — hold to open a second page
- [6. Arming and focus](#6-arming-and-focus) — a pot does nothing until it has moved
- [7. Freeze and pickup](#7-freeze-and-pickup) — nothing jumps when you let go
- [8. Zones with hysteresis](#8-zones-with-hysteresis) — one pot, N discrete choices
- [9. CV takeover](#9-cv-takeover) — inferring whether a jack is patched

**Output**
- [10. `ledSet()`](#10-ledset) — PWM brightness, and the zero that is not zero
- [11. Flash patterns](#11-flash-patterns) — non-blocking N-flash confirmation
- [12. Brightness vocabulary](#12-brightness-vocabulary) — what 255, 40 and 24 mean
- [13. Triangle fade](#13-triangle-fade) — gamma-corrected, for showing a rate

**State**
- [14. Debounced EEPROM save](#14-debounced-eeprom-save) — and the version stamp trap

## Constants

Shared across every sketch that has the corresponding element. Same name, same value.

| Constant | Value | What it is |
|---|---|---|
| `DEBOUNCE_MS` | 30 ms | Button debounce. Universal. |
| `HOLD_MS` | 300 ms | Open a modal edit page |
| `PRESS_SHORT_MS` | 400 ms | Release under this is a tap, over it is the second gesture |
| `PRESS_LONG_MS` | 1000 ms | Fires while held |
| `PRESS_VLONG_MS` | 1500 ms | Fires while held, the disruptive one (reset, re-roll, mode) |
| `EE_SAVE_DELAY_MS` | 2000 ms | Quiet period before an EEPROM write |
| `ARM_DELTA` | 24 | Pot travel before it takes effect in a shift layer |
| `DIV_POT_HYST` | 24 | The same idea against a latched reference |
| `PICKUP_WINDOW` | 12 | How close counts as catching an old position |
| `ZONE_HYST` | 12 | Travel past a zone boundary before it flips |
| `MOVE_DELTA` | 3 | "This is the pot I am holding", for LED focus |
| `CV_POT_TAKEOVER` | 8 | Pot travel that reclaims control from a CV jack |
| `CV_PRESENT_LO` | 20 | ~0.1 V; below this a jack reads as unpatched |
| `EDIT_IDLE_LED` | 24 | Brightness meaning "in the layer, nothing touched yet" |

---

## 1. Non-blocking ADC round-robin

*All six.* `analogRead()` blocks for ~100 µs, which a 2–4 kHz engine tick cannot afford. Start a
conversion, come back for it later, move to the next channel. The 3/4 IIR is what stops a pot
resting on a zone boundary from dithering between two values.

```c
static inline void serviceADC() {
  if (ADCSRA & (1 << ADSC)) return;         // still converting
  uint16_t v = ADC;
  adcVal[adcCh] = (uint16_t)((adcVal[adcCh] * 3UL + v) >> 2);
  adcCh = (uint8_t)((adcCh + 1) & 3);       // A0..A3 are mux channels 0..3
  ADMUX = (1 << REFS0) | adcCh;
  ADCSRA |= (1 << ADSC);
}
```

Prime `adcVal[]` with real `analogRead()` calls in `setup()` before taking the converter over, or
the first cycle runs from zero. Set `DIDR0` to disable the digital input buffers on the analog pins.

**Whether to filter a CV input is a judgement call.** `krell` filters F1 because it is read every
tick and feeds the slope of a stage, so converter noise would put a flutter on every envelope;
`envelope_follower` takes F1 raw because it is the signal being measured.

## 2. Button debouncer

*All six, byte-identical.* Level-based, not edge-based: `btnStable` is the debounced level, and
other code tests `btnStable == LOW` to mean "the button is down right now".

```c
uint32_t now = millis();
uint8_t  r = digitalRead(PIN_BUTTON);

if (r != btnLastRead) { btnLastRead = r; btnChangedMs = now; }
else if (r != btnStable && (now - btnChangedMs) >= DEBOUNCE_MS) {
  btnStable = r;
  if (r == LOW) { btnDownMs = now; /* clear the per-press flags here */ }
  else          { /* release action */ }
}
```

## 3. Gesture vocabulary

There is **no double-tap** anywhere in this repo, deliberately — it costs a timeout on every single
tap. Three shapes are in use:

**Release-time branching** (`clepz`) — one button, two switches:

```c
} else if (!vlongFired && !btnDivUsed) {      // release without having already fired
  uint32_t held = now - btnDownMs;
  if (held < PRESS_SHORT_MS) { /* bright flashes  */ }
  else                       { /* dim flashes     */ }
}
```

**Fires while held** (`clepz`, `envelope_follower`, `krell`) — the action happens at the threshold,
not on release, so you feel it land. A `Fired` flag makes it once-per-press:

```c
if (btnStable == LOW && !vlongFired && (now - btnDownMs) >= PRESS_VLONG_MS) {
  vlongFired = true;
  /* the disruptive action */
  startPattern(1, 255, 250, 100);
}
```

**Structural suppression** (`triple_lfo`, `smooth_random`, `keyframe_morpher`) — there is simply no
release action, so a tap cannot do anything and no flag is needed.

Pick the flag approach only when the button already has a tap action to protect.

## 4. Shift layer A: latching

*`clepz`, `krell`.* Hold the button and turn a pot; the pot means something else while you do. Use
this when the alternate meaning is **a different value of the same kind of thing** and the pot's
normal job can survive being touched — or when you freeze it, as in element 7.

Four things matter here, and three of them are easy to miss:

```c
static void serviceDivider() {
  uint16_t pot = adcVal[1];
  int16_t  d   = (int16_t)pot - (int16_t)divPotLast;
  if (d > -DIV_POT_HYST && d < DIV_POT_HYST) return;   // 1. deadband vs a LATCHED reference
  divPotLast = pot;

  bool odd = (btnStable == LOW);                       // 2. debounced LEVEL, not an edge
  if (odd) btnDivUsed = true;                          // 3. this press is spent

  uint8_t idx = divSlotForPot(pot);
  if (idx == gDivIdx && odd == gDivOdd) return;
  gDivIdx = idx;
  gDivOdd = odd;                                       // 4. the CHOICE latches past the release
  divCounter = 0;
  if (extClock || odd) startPattern((uint8_t)(idx + 1), odd ? 40 : 255, 60, 120);
}
```

`btnDivUsed` is cleared on press and tested in **both** the release branch and the fires-while-held
branch. Without it, every shift move also triggers whatever the button does on its own.

The contract to document, from `clepz/README.md`: *a hold you spend on a pot cancels the gesture the
release would otherwise fire; a hold where you touch nothing still behaves as a normal hold.*

## 5. Shift layer B: modal edit page

*`triple_lfo`, `smooth_random`, `keyframe_morpher`.* Hold `HOLD_MS` to open a page where the pots
mean something else entirely, with an LED state to say you are in it. Use this when the alternate
meanings are **a different kind of thing** (waveform, algorithm, stored scene) and there are several
of them.

```c
static void serviceButton() {
  /* ... debouncer ... */
    if (r == LOW) btnDownMs = now;      // a release shorter than HOLD_MS does nothing
    else if (editing) endEdit();
  /* ... */
  if (btnStable == LOW && !editing && (now - btnDownMs) >= HOLD_MS) beginEdit();
}
```

**Stacking a second gesture on top** (`smooth_random`): a longer hold *with no pot armed* is a third
action. `anyArmed()` is the inverse of `btnDivUsed` — it asks whether this hold was spent.

```c
if (editing && !longFired && (now - btnDownMs) >= LORENZ_HOLD_MS && !anyArmed()) { ... }
```

Note the escape-hatch rule that comes with it: in the mode you toggle into, make sure no pot can
arm, so a stray nudge can never trap the user in it.

## 6. Arming and focus

*`triple_lfo`, `smooth_random`, `keyframe_morpher`, `krell`.* Two thresholds doing two different
jobs. `ARM_DELTA` (24) decides whether the pot takes effect at all; `MOVE_DELTA` (3) tracks which
pot is under the user's hand, for the LED.

```c
for (uint8_t ch = 0; ch < 3; ch++) {
  int16_t d = (int16_t)adcVal[ch] - (int16_t)potEntry[ch];
  if (!armed[ch]) {
    if (d < ARM_DELTA && d > -ARM_DELTA) continue;
    armed[ch] = true;
    lastTouched = (int8_t)ch;
  }
  int16_t moved = (int16_t)adcVal[ch] - (int16_t)potSeen[ch];
  if (moved > MOVE_DELTA || moved < -MOVE_DELTA) {
    potSeen[ch] = adcVal[ch];
    lastTouched = (int8_t)ch;
  }
  /* apply adcVal[ch] to the shift parameter */
}
```

This is what makes the guarantee possible: **holding the button and releasing it cannot change
anything.** There is no way to open a shift layer and have a pot's resting position silently
overwrite a stored value. Say so in the README; users do not trust it until they read it.

**Re-arm when the target changes.** `keyframe_morpher` re-arms POT3 whenever POT2 selects a
different channel, so moving on to another channel does not immediately stamp POT3's current
position onto it.

## 7. Freeze and pickup

*`triple_lfo`, `smooth_random`, `krell`.* Needed whenever a pot's normal job is live while the
shift layer has it. Add a `potVal[]` layer between `adcVal[]` and everything that reads a pot; the
engine reads `potVal[]`, and the shift layer reads `adcVal[]`.

```c
static void freezePots() {                  // on press
  for (uint8_t ch = 0; ch < 3; ch++) {
    potEntry[ch] = adcVal[ch];
    potSeen[ch]  = adcVal[ch];
    armed[ch] = false;
    frozen[ch] = true;                      // potVal stops tracking and holds its value
  }
  lastTouched = -1;
}

static void releasePots() {                 // on release
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (!frozen[ch]) continue;
    if (armed[ch]) entrySign[ch] = (adcVal[ch] >= potEntry[ch]) ? 1 : -1;
    else           frozen[ch] = false;      // untouched: live again at once
  }
}

static void servicePots() {                 // every loop
  for (uint8_t ch = 0; ch < 3; ch++) if (!frozen[ch]) potVal[ch] = adcVal[ch];
}

static void servicePickup() {               // every loop, only while the button is UP
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (!frozen[ch]) continue;
    int16_t d = (int16_t)adcVal[ch] - (int16_t)potEntry[ch];
    if (d <= PICKUP_WINDOW && d >= -PICKUP_WINDOW) frozen[ch] = false;
    else if ((entrySign[ch] > 0) != (d > 0))       frozen[ch] = false;
  }
}
```

The rule: **a pot you moved keeps its old value until you turn it back through where it started; a
pot you did not touch goes live at once.** Nothing jumps.

Two traps. `servicePickup()` must not run while the button is down — `entrySign[]` is only set at
release, and a stale one will unfreeze a pot mid-edit. And do not freeze CV inputs; a patched CV
should keep working while the user edits.

## 8. Zones with hysteresis

*`triple_lfo`, `smooth_random`, `keyframe_morpher`, `krell`.* One pot, N discrete choices, without
dithering on the boundaries. The hysteresis is applied against the **current** selection, so the
boundary moves depending on which way you came.

```c
static uint8_t zoneFor(uint8_t ch, uint16_t raw) {
  uint8_t cur = gAlgo[ch];
  uint8_t z = (uint8_t)(((uint32_t)raw * NUM_ALGOS) >> 10);
  if (z == cur) return cur;
  uint16_t bound = (uint16_t)((((uint32_t)(z > cur ? z : cur)) << 10) / NUM_ALGOS);
  if (z > cur) return (raw >= bound + ZONE_HYST) ? z : cur;
  return (raw + ZONE_HYST <= bound) ? z : cur;
}
```

Clamp `z` to `N - 1` if `N` does not divide 1024 evenly.

## 9. CV takeover

*`clepz`, `keyframe_morpher`.* The MOD1 has no switched jacks, so "is something patched here" has
to be inferred. Only needed when a CV **replaces** a pot. If the CV can simply **add** to the pot,
do that instead — `krell`'s F1 does, and it has no inference to get wrong and no rule to learn.

```c
static void serviceCvTakeover() {
  uint32_t now = millis();
  if (adcVal[5] > CV_PRESENT_LO) {
    cvActive  = true;
    cvQuietMs = now;
    cvPotLast = adcVal[0];      // track the pot while the CV drives, so only a LATER turn counts
    return;
  }
  if (!cvActive) return;
  int16_t d = (int16_t)adcVal[0] - (int16_t)cvPotLast;
  if (d > CV_POT_TAKEOVER || d < -CV_POT_TAKEOVER) cvActive = false;
  else if ((now - cvQuietMs) >= CV_IDLE_MS)        cvActive = false;
}
```

A rise past ~0.1 V latches the jack in and it **keeps** control when the CV falls back to 0 V, so
the bottom of the CV range stays reachable. Moving the pot, or three seconds of quiet, hands it back.

## 10. `ledSet()`

*All six, byte-identical.* `OCR2B = 0` still leaks a 1/256 sliver, so the LED never looks off.
Detach the compare output instead. Never use `analogWrite()` — these sketches own the timers.

```c
static inline void ledSet(uint8_t v) {
  if (v == 0) {
    TCCR2A &= (uint8_t)~(1 << COM2B1);
    PORTD  &= (uint8_t)~(1 << PD3);
  } else {
    TCCR2A |= (1 << COM2B1);
    OCR2B = v;
  }
}
```

The LED sits behind 10k, so it is dim to begin with; the full 0–255 range is usable.

## 11. Flash patterns

*`clepz`, `envelope_follower`, `keyframe_morpher`, `krell`.* Non-blocking N-flash confirmation.
Count carries the value, brightness carries which parameter.

```c
static void startPattern(uint8_t pulses, uint8_t bright, uint16_t onMs, uint16_t offMs) {
  patLeft = pulses; patBright = bright; patOnMs = onMs; patOffMs = offMs;
  patOn = true; patNextMs = millis() + onMs;
}
```

Drive it from the top of `serviceLED()`, where it pre-empts everything and returns:

```c
if (patLeft) {
  if ((int32_t)(now - patNextMs) >= 0) {
    if (patOn) { patOn = false; patNextMs = now + patOffMs; patLeft--; }
    else       { patOn = true;  patNextMs = now + patOnMs; }
  }
  ledSet(patLeft ? (patOn ? patBright : 0) : 0);
  return;
}
```

Flash the **index + 1**, so the first slot is one flash rather than none.

| Timing | Used for |
|---|---|
| `(60, 120)` | short confirmation, a value changed |
| `(200, 160)` | the dim/secondary confirmation |
| `(250, 100)` | one emphatic flash, something disruptive happened |

`smooth_random` uses a simpler `ledBlink(times)` with a fixed `BLINK_MS` of 120 instead; `triple_lfo`
has no flash helper at all, because its LED is always a live waveform preview.

## 12. Brightness vocabulary

One LED carries several switches by pairing a flash **count** with a **brightness**.

| Value | Meaning |
|---|---|
| `255` | the primary parameter, or a gate high |
| `40` | the secondary parameter — *"bright flashes mean mode, dim flashes mean direction"* |
| `24` (`EDIT_IDLE_LED`) | in a shift layer, nothing touched yet |
| `0` | off, with the detach above |

Two rules worth keeping: **nothing is ever fully dark in a mode display**, so a dim LED never reads
as a dead one; and within a firmware, a brightness always means the same family of thing.

## 13. Triangle fade

*`smooth_random`.* For showing a *rate* rather than a value. The squaring matters — the eye is
roughly square-law, so a linear duty ramp shoots up and then sits near full for most of the sweep
instead of reading as a steady fade. This was a deliberate fix, not a flourish.

```c
static inline uint8_t triangleBrightness(uint16_t ms, uint8_t periodShift) {
  uint16_t half = (uint16_t)(1u << (periodShift - 1));
  uint16_t t    = (uint16_t)(ms & ((1u << periodShift) - 1));
  uint16_t up   = (t < half) ? t : (uint16_t)(2 * half - 1 - t);
  uint8_t  lin  = (uint8_t)(((uint32_t)up << 8) >> (periodShift - 1));
  return (uint8_t)(((uint16_t)lin * lin) >> 8);
}
```

`periodShift` 10 is ~1 Hz, 8 is ~4 Hz. For a plain 2 Hz blink a raw `millis()` bit test is enough
and needs no state: `ledSet((ms & 256) ? EDIT_IDLE_LED : 0);`

## 14. Debounced EEPROM save

*All six.* Every change restarts a 2 s window, so a burst of taps writes once. `EEPROM.update()`
skips the write when the byte is unchanged.

```c
static void markDirty() { eeDirty = true; eeDirtyMs = millis(); }

// in loop()
if (eeDirty && (millis() - eeDirtyMs) > EE_SAVE_DELAY_MS) {
  eeDirty = false;
  EEPROM.update(EE_ADDR_MODE, gMode);
  /* ... one update() per stored byte ... */
}
```

**Use a distinctive version stamp, not `1`.** A chip that has carried another firmware from this
repo can easily have a small integer sitting at address 1, and that will read as a valid stamp and
load garbage as settings. `keyframe_morpher` and `krell` use `0x7A` / `0x7B`.

```c
if (EEPROM.read(EE_ADDR_VER) != EE_VERSION) {
  /* write every default, then the stamp */
}
uint8_t c = EEPROM.read(EE_ADDR_MODE);
gMode = (c < NUM_MODES) ? c : (uint8_t)DEFAULT_MODE;   // range-check anyway
```

The stamp compares on **inequality**, so bumping it re-defaults an existing chip. When you add
stored bytes to a shipped firmware, bump the stamp — and choose the new defaults so the module
behaves as it did before, or a user's module will change under them on an update.
