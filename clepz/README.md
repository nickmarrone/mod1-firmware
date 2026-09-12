# clepz

A step CV / random / smooth-noise LFO generator for the HAGIWO MOD1, inspired by the *interface* of
Noise Engineering's Clep Diaz. Independent implementation, not affiliated with or endorsed by Noise
Engineering, and not derived from their code.

## Fitting six jacks and two switches onto a MOD1

Clep Diaz has two 3-position switches, an encoder with a push, and six jacks. MOD1 has one button,
three pots and four jacks. The compression:

| Clep Diaz | Here |
|---|---|
| `step/rand/lfo` switch | **short press** on the button |
| `up/updn/dn` switch | **long press** on the button |
| Count encoder | **POT1** |
| Encoder tap (mute) | POT1 fully counter-clockwise — 0 steps, exactly as "no LEDs lit = muted" |
| LED counter | one LED: it follows the output, and shows the count hint described below |
| `Clk` | **F1** |
| `Rst` | **F2** |
| `CV` | **F3** |
| `Uni` | **F4** |
| `BOC` | dropped — no jack left; the LED flashes at the beginning of each cycle instead |
| `Bi` | dropped — the MOD1 has no negative rail, so a bipolar output is not possible |

That leaves two spare pots, which pick up the two things the original gets from elsewhere in a
case: a clock when you have not patched one, and slew.

## Panel

```
POT1  A0   step count  (LFO: amplitude)     F1  A3   clock in
POT2  A1   tempo, or clock divider          F2  A4   reset in
POT3  A2   slew / glide                     F3  A5   CV in -> step count
BUTTON D4  short = mode, long = direction   F4  D11  CV out, 0-5 V
           very long = reset / re-roll
           held + POT2 = odd divisions
LED   D3   output level + cycle + status
```

### POT1 — count / amplitude

In **Step** and **Random**, 0 to 16 steps. Fully counter-clockwise is 0 = **muted** (output held at
0 V, LED off). In **Up/Down** the 16 steps ping-pong into a cycle of up to 30. In **LFO**, POT1 is
the amplitude instead, scaling the output up from 0 V.

Sixteen positions rather than the original's thirty-two: a single-turn pot cannot reliably land on
one count in thirty-two, and the counts that matter musically are almost all under sixteen.

### POT2 — tempo *or* divider

With **nothing patched to F1**, POT2 is the internal tempo: **30 to 960 BPM**, exponential. Three
seconds after the last external clock edge the module falls back to it automatically.

With a **clock patched to F1**, the same pot becomes a clock divider, and it carries two tables:

| Turn POT2 | Divisions |
|---|---|
| on its own | ÷**1, 2, 4, 8, 16, 32** — six slots across the travel |
| with the **button held down** | ÷**1, 3, 5, 7, 11, 17** |

The choice **latches** when you turn the pot, so the odd divisions stay selected after you let the
button go. Turning POT2 again on its own drops back to the powers of two. The LED confirms each
change with *slot number* flashes: **bright** for the powers of two, **dim** for the odd ones.

Holding the button to reach the odd table cancels the gesture that the release would otherwise
fire, so dialling a division never changes the mode or the direction by accident. A hold where you
do not touch POT2 still behaves as a normal long press.

Note that the pot changes job the moment a clock arrives — set it after you patch.

### POT3 — slew

Fully counter-clockwise the steps are hard-edged. Turning it up glides between steps, up to a time
constant of one whole step at the top. In LFO mode it adds extra lag on top of the LFO's own shape.

## Button

| Gesture | Action | LED confirmation |
|---|---|---|
| short, under 400 ms | mode: Step → Random → LFO | 1 / 2 / 3 **bright** short flashes |
| long, 0.4–1.5 s | direction: Up → Up/Down → Down | 1 / 2 / 3 **dim** long flashes |
| very long, over 1.5 s | reset to the start of the cycle, and re-roll the random values | one long bright flash |
| any length, while turning POT2 | clock divider from the odd table | 1–6 **dim** short flashes |

Bright flashes mean mode, dim flashes mean direction — that is how one LED carries two switches.
Mode and direction are written to EEPROM two seconds after you stop changing them.

## Modes

- **Step** — a staircase of `count` evenly spaced values from 0 V to 5 V, played in the chosen
  direction. Up runs 0…n-1 and wraps, Down runs n-1…0, Up/Down ping-pongs.
- **Random** — the same index walk, but the step *values* are random, and they are re-rolled at the
  start of every cycle, so each pass differs.
- **LFO** — a smooth noise LFO timed by the clock: a new random target each clock, glided across the
  measured clock period. POT1 sets amplitude. Direction sets the symmetry of the wave, as on the
  original: **Up** makes rises gentler (and falls correspondingly steeper), **Down** makes falls
  gentler, **Up/Down** is symmetrical. The beginning-of-cycle LED flash is random in this mode.

## Jacks

- **F1 — clock in.** Rising edge, 2 ms debounce. F1 is the MOD1's only DC-coupled fast input, which
  is why the clock lives here.
- **F2 — reset in.** Jumps to the start of the cycle. Because F2 has a 1 µF cap to ground, this is
  read as an *analog* value with hysteresis (high above ~2.0 V, low below ~1.0 V) rather than as a
  digital pin. **Triggers shorter than about 1 ms may not clear the threshold** — use a gate or a
  normal-length trigger. The very-long button press does the same thing by hand.
- **F3 — CV in.** 0–5 V adds up to 15 steps to whatever POT1 is set to (adds amplitude in LFO mode).
  The same 1 µF cap that hurts F2 is exactly right here: it is already a CV smoother.
- **F4 — CV out.** 0–5 V unipolar, 1k output impedance. Not quantized.

## LED

In priority order: mode/direction confirmation flashes, then the count hint, then the
beginning-of-cycle flash, then it follows the output level.

The **count hint** is Clep Diaz's, kept: for 800 ms after you change the count, the LED glows
**bright if the count divides by 4**, **dim if it divides by 3**, and stays **off otherwise**. It
is a fast way to land on a count that will line up with what else is running.

## Implementation notes

- Only Timer2 is reconfigured (62.5 kHz fast PWM on D11 and the LED). **D9 and D10 are deliberately
  left as inputs** — on the MOD1 they share nets with A4 and A5, so driving them would fight the
  reset and CV inputs.
- 2 kHz output tick. Non-blocking round-robin ADC over A0, A1, A2 and A5, with the reset channel A4
  sampled every other slot so reset latency stays around 200 µs. The pot and CV channels get a light
  IIR to kill ADC dither; the reset channel is read raw.
- Measured worst case on a simulated ATmega328P at 16 MHz: **95 µs per `loop()` pass**, so clock
  jitter is under 0.1 ms and the tick has five times the headroom it needs.

## Patch to start with

Mode Step, direction Up, a clock into F1, F4 into a filter cutoff. Turn POT1 up from zero and watch
the count hint. Then hold the button to flip to Up/Down, and turn POT3 up to smear the staircase
into a shape.
