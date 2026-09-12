# smooth_random

Three independent smooth random voltages for the HAGIWO MOD1. Nothing ever steps or jumps: every
algorithm here is continuous, so these outputs are safe on filter cutoffs, VCA levels, wavefolders,
FM index — anywhere a sample-and-hold would click.

## Panel

```
POT1  A0   channel 1 rate          F1  A3   global rate CV in
POT2  A1   channel 2 rate          F2  D9   channel 1 out   0-5 V
POT3  A2   channel 3 rate          F3  D10  channel 2 out   0-5 V
BUTTON D4  next algorithm          F4  D11  channel 3 out   0-5 V
LED   D3   which algorithm
```

Each pot sets the rate of its own channel, exponentially from **0.01 Hz (one move every 100
seconds) to 20 Hz**. The three channels are independent random streams — they never lock together.

**F1 is a global rate CV.** 0–5 V adds up to about **+4 octaves** on top of whatever the pots are
set to, applied to all three channels at once. Patch an envelope or a slow LFO in and the whole
module speeds up and settles together while keeping the relative rates the pots set.

The button steps through five algorithms. The choice is saved to EEPROM, so it survives a power
cycle.

## The five algorithms

| # | Name | LED | Character |
|---|---|---|---|
| 0 | **DRIFT** | off | Picks a random destination, glides there on a smooth S-curve, arrives, rests, picks another. Uses the full 0–5 V range and has a clear sense of *arriving somewhere*. |
| 1 | **WANDER** | slow triangle fade (~1 Hz) | Brownian motion with inertia. No destination and no rest — it just meanders, and it stays near where it already was. Bounces off the rails instead of sticking to them. |
| 2 | **TURBULENCE** | fast triangle fade (~4 Hz) | Three octaves of DRIFT summed (1×, 2.75×, 7.25× the pot rate at 1, ½, ¼ amplitude). Slow drift with fine detail riding on top — the busiest of the five. |
| 3 | **LORENZ** | steady dim | A Lorenz attractor. See the note below — this one uses the pots differently. |
| 4 | **HOLD** | steady on | Sample-and-hold that glides instead of stepping: it crosses to the new value over the first quarter of the period, then holds. Stepped feel, no clicks. |

### LORENZ is the odd one out

The other four modes are three separate generators, one per pot. LORENZ is a single chaotic system
whose three state variables x, y, z feed the three outputs, so the channels are **related but never
identical** — they are three views of one trajectory. That means the pots change meaning:

| | LORENZ |
|---|---|
| POT1 | master speed (0.0015 – 3 Hz — slower than the other modes; a chaotic attractor wants room to breathe) |
| POT2 | ρ, 20 → 60 |
| POT3 | β, 1 → 4 |
| F1 CV | still scales the master speed |

Classic Lorenz values (ρ = 28, β = 8/3) sit at roughly 20 % on POT2 and 55 % on POT3. Outputs 1 and
2 (x and y) sweep the full range and cross often; output 3 (z) is the one that spikes when the
trajectory switches lobes.

## Implementation notes

- 1 kHz generator tick, 16-bit fixed point internally, 8-bit out through 62.5 kHz PWM on Timer1
  (D9/D10) and Timer2 (D11), then the MOD1's own ~159 Hz reconstruction filter. Timer0 and
  `millis()` are untouched.
- Rates come from a 33-entry exponential PROGMEM table with linear interpolation, so there is no
  `pow()` in the tick.
- The ADC runs free of `analogRead()`: a non-blocking round-robin over A0–A3 that never stalls the
  tick for the 112 µs an `analogRead` would cost.
- Randomness is a xorshift32 seeded from the unconnected A6/A7 pins plus a counter rolled in EEPROM
  byte 1, so two power-ups do not produce the same sequence.
- Measured worst case on a simulated ATmega328P at 16 MHz: **462 µs per `loop()` pass** (LORENZ)
  against the 1000 µs tick — about 54 % headroom.

## Caveats

- Output 3 in LORENZ mode can move fast when the attractor switches lobes; that is the attractor,
  not a glitch, and the board's output filter rounds it off.
- At the very top of the rate range TURBULENCE and WANDER put real energy above the 159 Hz output
  filter, so what comes out of the jack is gentler than what the code computes. That is intended.
