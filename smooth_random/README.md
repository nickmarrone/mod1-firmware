# smooth_random

Three smooth random voltages for the HAGIWO MOD1, each channel with its own algorithm. Nothing
ever steps or jumps: every algorithm here is continuous, so these outputs are safe on filter
cutoffs, VCA levels, wavefolders, FM index — anywhere a sample-and-hold would click.

## Panel

```
POT1  A0   channel 1 rate          F1  A3   global rate CV in
POT2  A1   channel 2 rate          F2  D9   channel 1 out   0-5 V
POT3  A2   channel 3 rate          F3  D10  channel 2 out   0-5 V
BUTTON D4  hold to edit algorithms F4  D11  channel 3 out   0-5 V
LED   D3   channel 1 output, or the algorithm while editing
```

Each pot sets the rate of its own channel, exponentially from **0.01 Hz (one move every 100
seconds) to 20 Hz**. The three channels are independent random streams — they never lock together.

**F1 is a global rate CV.** 0–5 V adds up to about **+4 octaves** on top of whatever the pots are
set to, applied to all three channels at once. Patch an envelope or a slow LFO in and the whole
module speeds up and settles together while keeping the relative rates the pots set.

## The four algorithms

| # | Name | Character |
|---|---|---|
| 0 | **DRIFT** | Picks a random destination, glides there on a smooth S-curve, arrives, rests, picks another. Uses the full 0–5 V range and has a clear sense of *arriving somewhere*. |
| 1 | **WANDER** | Brownian motion with inertia. No destination and no rest — it just meanders, and it stays near where it already was. Bounces off the rails instead of sticking to them. |
| 2 | **TURBULENCE** | Three octaves of DRIFT summed (1×, 2.75×, 7.25× the pot rate at 1, ½, ¼ amplitude). Slow drift with fine detail riding on top — the busiest of the four. |
| 3 | **HOLD** | Sample-and-hold that glides instead of stepping: it crosses to the new value over the first quarter of the period, then holds. Stepped feel, no clicks. |

## Setting an algorithm

**Hold the button.** A short tap does nothing at all.

While it is held, each pot picks its own channel's algorithm. The pot's travel is divided into four
equal zones, in the table order above, so fully anticlockwise is drift and fully clockwise is hold.

- **A pot does nothing until it has moved**, about 24 ADC counts. Holding the button without
  turning anything cannot change an algorithm by accident.
- **The three rates freeze** at the values they had when you pressed. The channels keep running at
  their old rates while you sweep, so you hear each algorithm's character as you land on it.
  Anything patched into F1 still modulates them.
- **The LED shows the algorithm** of the channel whose pot you last moved: steady dim is drift, a
  slow fade is wander, a fast fade is turbulence, steady full on is hold. Until you move a pot it
  blinks dim at about 2 Hz, which is how you know you are in edit mode. Nothing in edit mode is
  ever fully dark, so a dim LED never reads as a dead one.

When you let go, a channel whose pot you moved **keeps the rate it had before the edit**. That pot
stays inert until you turn it back through where it started, at which point it takes over again.
Channels you did not touch never stop tracking. Nothing jumps, and nothing is retimed behind your
back.

All three algorithms are saved to EEPROM, about two seconds after the last change so that sweeping
across three zone boundaries costs one write rather than three.

## LORENZ takes the whole module

LORENZ is not one of the four, because it cannot be. The other algorithms are three separate
generators, one per pot. LORENZ is a single chaotic system whose three state variables x, y and z
feed the three outputs, so the channels are **related but never identical** — they are three views
of one trajectory. It needs every pot and every output at once.

So it gets a gesture of its own. **Hold the button past about a second and a half without touching
a pot**, and LORENZ toggles. The LED blinks twice going in and once coming out. Touching a pot
during the hold cancels it, since that is the per-channel edit gesture instead — except in LORENZ
itself, where no pot can cancel anything, so a stray nudge can never trap you in the mode.

While it runs the pots change meaning:

| | LORENZ |
|---|---|
| POT1 | master speed (0.0015 – 3 Hz — slower than the other algorithms; a chaotic attractor wants room to breathe) |
| POT2 | ρ, 20 → 60 |
| POT3 | β, 1 → 4 |
| F1 CV | still scales the master speed |

Classic Lorenz values (ρ = 28, β = 8/3) sit at roughly 20 % on POT2 and 55 % on POT3. Outputs 1 and
2 (x and y) sweep the full range and cross often; output 3 (z) is the one that spikes when the
trajectory switches lobes.

The per-channel algorithms are remembered the whole time and come straight back when you leave. So
are the rates: the pot positions from before LORENZ are held, and a pot you turned to shape the
attractor stays inert on the way out until you turn it back through where it was — the same pickup
rule an edit uses. Both the algorithms and the LORENZ flag survive a power cycle.

## Implementation notes

- 1 kHz generator tick, 16-bit fixed point internally, 8-bit out through 62.5 kHz PWM on Timer1
  (D9/D10) and Timer2 (D11), then the MOD1's own ~159 Hz reconstruction filter. Timer0 and
  `millis()` are untouched.
- Rates come from a 33-entry exponential PROGMEM table with linear interpolation, so there is no
  `pow()` in the tick.
- The ADC runs free of `analogRead()`: a non-blocking round-robin over A0–A3 that never stalls the
  tick for the 112 µs an `analogRead` would cost, lightly smoothed so a pot resting on a zone
  boundary stays put.
- One freeze-and-pickup mechanism serves both the edit gesture and the LORENZ handoff. LORENZ is
  the only consumer that reads the pots live, because there they really are its own controls.
- Randomness is a xorshift32 seeded from the unconnected A6/A7 pins plus a counter rolled in EEPROM
  byte 1, so two power-ups do not produce the same sequence.
- Measured worst case on a simulated ATmega328P at 16 MHz: **462 µs per `loop()` pass** (LORENZ)
  against the 1000 µs tick — about 54 % headroom. Three channels of TURBULENCE is nine segment
  steps, exactly what the old global TURBULENCE already ran, so per-channel dispatch does not move
  that ceiling.
- Upgrading from an earlier build resets all three channels to DRIFT once. A version stamp in
  EEPROM settles it, because the old layout's single global type byte is indistinguishable from a
  per-channel one.

## Caveats

- Output 3 in LORENZ can move fast when the attractor switches lobes; that is the attractor, not a
  glitch, and the board's output filter rounds it off.
- At the very top of the rate range TURBULENCE and WANDER put real energy above the 159 Hz output
  filter, so what comes out of the jack is gentler than what the code computes. That is intended.
