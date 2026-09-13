# krell

A **self-generating random envelope** for the HAGIWO MOD1. The module patches itself: one random
number sets the rise time, a second sets the fall time, a third sets how long to wait before firing
again, and then it re-rolls. Set it going and leave it.

This is the [Krell patch](https://en.wikipedia.org/wiki/Forbidden_Planet) — a Buchla 266 Source of
Uncertainty wired into a Maths, named after the *Forbidden Planet* score, and the heart of Mutable
Peaks' random modes. It is a technique rather than anyone's product.

Every other firmware here needs something patched into it before anything happens. `clepz` wants a
clock, `envelope_follower` wants a signal, the LFOs want somewhere to go. This one is the thing
that makes a patch move on its own, and it hands you three outputs to move it with: an envelope, a
trigger, and a pitch.

## Panel

```
POT1  A0   time range, ~5 ms .. ~10 s      F1  A3   time CV in, up to ~5 octaves faster
POT2  A1   randomness amount               F2  D9   cycle pulse out, 10 ms
POT3  A2   rest / density                  F3  D10  this cycle's random CV   0-5 V
BUTTON D4  tap = curve, hold 1.5 s         F4  D11  envelope out             0-5 V
           = re-roll and restart now
LED   D3   follows the envelope, plus the curve confirmation flashes
```

## The patch it makes

F4 into a VCA, F3 into an oscillator's pitch, F2 into whatever else wants a trigger. That is a
voice that plays itself, and the three knobs are how fast, how random, and how often.

One cycle is **rise, fall, rest**. Entering the rise rolls four numbers at once — a rise time, a
fall time, a rest time, and the CV that F3 holds for the whole cycle — and fires F2.

## POT1 — time range

Sets how long a stage takes, about **5 ms fully clockwise to 10 seconds fully anticlockwise**,
exponentially. Fully clockwise with no rest the module cycles at about 95 Hz and is a rough
audio-rate oscillator; fully anticlockwise one envelope takes twenty seconds to rise and fall.

POT1 is read **live inside a stage**, not latched at the start of the cycle. Turn it during a slow
rise and the rise speeds up as you turn, the way the time knobs on a Maths do. Only the random
offsets are latched.

## POT2 — randomness

**One uncertainty knob for the whole module.** It is the width of all four distributions at once:
the rise time, the fall time, the rest, and the F3 pitch all scatter by the same amount.

- **Fully anticlockwise, nothing is random.** The rise and fall are exactly the POT1 time, the rest
  is exactly the POT3 time, and F3 sits at a steady 2.5 V. This is a legitimate setting, not a
  broken one — it is a plain repeating AD envelope and a plain metronome, and it is what you want
  when the module is the clock for something else.
- **Fully clockwise, nothing repeats.** The times scatter by about **±4 octaves** and F3 covers the
  full 0–5 V.

Because POT2 moves the pitch and the timing together, turning it up is audibly "more chaos"
everywhere rather than one parameter getting looser. That is the tradeoff for having three knobs
instead of six; see Caveats.

## POT3 — rest and density

How long the module waits at 0 V after the fall before starting the next cycle. The rest is
**proportional to the current stage time**, not an absolute delay: measured on the host, one third
of the travel gives a rest about **0.5×** a stage, two thirds about **2×**, and fully clockwise
about **8.5×**. So sweeping POT1 across its whole range keeps the density musically coherent
instead of turning a fast setting into a stutter and a slow one into silence.

**The bottom of the travel is a hard "no rest".** Below about 12 counts the rest state is skipped
entirely and the fall runs straight into the next rise — a Maths left cycling. That is also the
setting where F2 lands on the literal end of the cycle, since the end of the fall and the start of
the next rise are the same instant.

## F1 — time CV

Adds to POT1, up to about **5 octaves faster** at 5 V. Like POT1 it is read live, so an envelope
already in flight bends. An LFO here makes the density breathe; a second envelope here makes fast
gestures cluster inside slow ones.

Unlike the CV inputs in `clepz` and `keyframe_morpher`, F1 here **adds to the pot rather than
replacing it**, so there is no "is it patched" inference to get wrong and no takeover rule to
learn. POT1 stays the floor of the range and the CV pushes up from wherever you left it.

## F2 and F3 — the trigger and the pitch

**F2 fires a 10 ms pulse at the start of every rise**, which is the same instant F3 takes its new
value. That pairing is the point: a sample-and-hold or a sequencer clocked from F2 reads the
matching F3 value rather than the previous cycle's. Verified on the host across eight knob settings
— exactly one pulse per cycle, and F3 never changes on a tick without an F2 edge.

Firing at the start of the rise rather than at the end of the fall only differs when POT3 is up. At
POT3 fully anticlockwise the two instants coincide, so the literal Krell end-of-cycle behaviour is
still there when you want it.

F3 is an **independent roll**, not a copy of one of the time values. Tying it to the rise time
would have been more literal, but it would also go constant the moment POT2 came down, and a pitch
source that vanishes when you ask for steady timing is not worth having.

## Button — curve and re-roll

**A tap cycles the envelope curve**, confirmed by bright flashes:

| | flashes | rise | fall |
|---|---|---|---|
| **EXP** | 1 | fast off the bottom, easing into the top | fast initial drop, long tail |
| **LIN** | 2 | straight ramp | straight ramp |
| **LOG** | 3 | slow start, accelerating into the top | holds up, then plummets |

EXP is the percussive one and the default. LOG swells. The curve is saved to EEPROM two seconds
after you stop changing it.

**Holding the button for 1.5 seconds re-rolls and restarts the cycle immediately**, with one long
flash. This is the escape hatch for slow settings: when POT1 is near the bottom and a ten second
rest has just started, you do not have to wait it out to hear a different set of numbers. Same
gesture as the very-long press in `clepz`.

## LED

Follows the envelope, so you can see the shape the curve is making and read the density at a
glance. The curve confirmation flashes take priority over it.

## Implementation notes

- **The randomness is applied to an exponential table index, not to a time in milliseconds.** The
  offset is uniform in the index, which makes it **log uniform in time** — as likely to land half
  as long as twice as long. Randomising milliseconds instead would put almost every roll at the
  long end and sound like jitter rather than like a Krell patch. It is also the cheap way to do it:
  scaling a time means a 32×16 multiply, whereas offsetting an index is an add, in the same shape
  as `rateIndexFrom()` in `smooth_random`.
- **One PROGMEM table gives all six curves.** `EXP_TAB[65]` holds a single saturating exponential,
  normalised to hit both ends exactly. Read forwards it is the EXP rise; subtracted from full scale
  it is the EXP fall; read backwards it is the LOG pair. The interpolation is the
  table-plus-linear-interpolation idiom from `envelope_follower`'s `REL_COEF` and `smooth_random`'s
  `INC_TAB`, here with a 16-bit phase and a shift of 10, so `p >> 10` tops out at 63 and the
  `[i + 1]` read never runs off the array.
- **The curve endpoints land one LSB short** — `expCurve(65535)` is 65534, because the
  interpolation fraction never quite reaches 1. That is 0.08 mV of a 5 V output and `65534 >> 8` is
  still 255, and the engine writes the exact endpoints at the stage boundaries anyway, so it never
  reaches the jacks. All three curves were checked monotonic across all 65536 phase values.
- **The engine runs at 4 kHz**, faster than the 2 kHz of `clepz` and `keyframe_morpher`. At 2 kHz
  the shortest attack would be ten steps. The cost is that a stage can only end on a tick boundary:
  measured on the host, that rounds the period up by **4.3 % at the very fastest setting**, 0.13 %
  at a 63 ms stage, and nothing measurable below that. A wrong-by-4 % rate at 95 Hz is inaudible;
  a ten-step attack would not have been.
- **A 32-bit phase accumulator per stage, ending on the wrap**, as in `triple_lfo`. The largest
  increment in the table is 2³²/20, so a stage can never wrap inside a single tick.
- F1 is de-jittered with the same 3/4 IIR as the pots, unlike in `envelope_follower` where it is
  taken raw. Here it is read every tick and feeds the slope of whatever stage is running, so
  converter noise would put a flutter on every envelope.
- **The EEPROM version stamp is `0x7A`, not `1`**, for the reason `keyframe_morpher` documents: a
  chip carrying one of the other MOD1 firmwares can easily have a small integer sitting at
  address 1, which would read as a valid stamp.
- The xorshift generator and its A6 seeding are the ones from `clepz` and `triple_lfo` verbatim.
- No floating point, no interrupts, no `analogRead()` after `setup()`, Timer0 and `millis()` left
  alone. **3722 bytes of flash (12 %) and 68 bytes of RAM (3 %)**.

## Caveats

- **POT2 is one knob over four things.** You cannot have wild pitch with metronomic timing, or
  loose timing with a fixed pitch. That is a deliberate trade — with three pots, one coherent
  "amount of uncertainty" beats two of the four dimensions being controllable and two being stuck —
  but it is the first thing you will want more knobs for.
- **A wide POT2 with POT1 near either end clamps against the end of the table**, which skews the
  distribution instead of widening it. At POT1 fully anticlockwise with POT2 fully clockwise, every
  offset that would have made the envelope slower is clipped away, so the module runs about twice
  as fast as the knob suggests. Keep POT1 nearer the middle when POT2 is up.
- **There is no trigger or gate input.** All four jacks are spoken for, so the module cannot be
  slaved to a clock and cannot be played from a keyboard — it free-runs or it does nothing. If you
  want an envelope you can trigger, that is a different firmware.
- **At the fastest settings the 10 ms F2 pulse is a large fraction of the cycle** and reads as a
  lumpy gate rather than a trigger. The pulse is not shortened to match, because 10 ms is already
  near the floor of what the board's 1k/1 µF output filter will pass at full height.
- **That same filter rounds off attacks shorter than about 3 ms**, so the top of POT1's travel is
  softer than the numbers say. It also means the ~95 Hz oscillator at the very top is well
  attenuated, which is why the range stops there rather than going further.
- **Outputs are 0–5 V**, like everything else on the MOD1. The envelope is unipolar and there is no
  inverted output; patch `envelope_follower` if you need ducking.
- **No end-of-rise output.** Maths gives you both EOR and EOC; there is only one spare jack here
  and the trigger-plus-pitch pair is the more useful thing to put on it.
